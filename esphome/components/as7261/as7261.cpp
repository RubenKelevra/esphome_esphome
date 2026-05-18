#include "as7261.h"

#include <cstring>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::as7261 {

static const char *const TAG = "as7261";

void AS7261Component::setup() {
  if (this->int_pin_ != nullptr) {
    this->int_pin_->setup();
  }
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->release_reset_pulse_();
  }
}

void AS7261Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AS7261 UART color sensor:");
  LOG_PIN("  INT Pin: ", this->int_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  ESP_LOGCONFIG(TAG, "  Precision mode: %s", YESNO(this->precision_mode_));
  if (this->manual_exposure_) {
    ESP_LOGCONFIG(TAG, "  Exposure: manual, gain %s, integration time %u", this->gain_to_string_(),
                  this->integration_time_);
  } else {
    ESP_LOGCONFIG(TAG, "  Exposure: automatic");
  }
  this->check_uart_settings(115200);
  LOG_UPDATE_INTERVAL(this);
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "CCT", this->cct_sensor_);
  LOG_SENSOR("  ", "Calculated CIE 1960 Duv", this->calculated_duv_sensor_);
  LOG_SENSOR("  ", "Lux", this->lux_sensor_);
  LOG_SENSOR("  ", "OKLab L", this->oklab_l_sensor_);
  LOG_SENSOR("  ", "OKLab a", this->oklab_a_sensor_);
  LOG_SENSOR("  ", "OKLab b", this->oklab_b_sensor_);
  LOG_SENSOR("  ", "OKLCH L", this->oklch_l_sensor_);
  LOG_SENSOR("  ", "OKLCH C", this->oklch_c_sensor_);
  LOG_SENSOR("  ", "OKLCH h", this->oklch_h_sensor_);
  LOG_SENSOR("  ", "Device temperature", this->device_temperature_sensor_);
  LOG_SENSOR("  ", "Vendor CIE 1976 DUV", this->duv_cie1976_sensor_);
  LOG_SENSOR("  ", "Near-IR percent", this->near_ir_percent_sensor_);
  LOG_SENSOR("  ", "Raw clear", this->raw_clear_sensor_);
  LOG_SENSOR("  ", "Raw dark", this->raw_dark_sensor_);
  LOG_SENSOR("  ", "Raw near-IR", this->raw_near_ir_sensor_);
  LOG_SENSOR("  ", "Calibrated X", this->x_sensor_);
  LOG_SENSOR("  ", "Calibrated Y", this->y_sensor_);
  LOG_SENSOR("  ", "Calibrated Z", this->z_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Firmware version", this->firmware_version_text_sensor_);
#endif
}

void AS7261Component::loop() { this->poll_transport_(); }

void AS7261Component::update() {
  if (this->transport_busy_()) {
    return;
  }

  if (!this->begin_at_command_("AT")) {
    ESP_LOGW(TAG, "Unable to start AS7261 transport shell command");
  }
}

bool AS7261Component::begin_at_command_(const char *command, uint32_t timeout_ms) {
  if (this->transport_busy_()) {
    return false;
  }
  if (command == nullptr || command[0] == '\0') {
    ESP_LOGE(TAG, "Refusing to send an empty AS7261 AT command");
    return false;
  }

  size_t command_length = 0;
  while (command[command_length] != '\0' && command_length < COMMAND_BUFFER_LENGTH - 1) {
    this->command_buffer_[command_length] = command[command_length];
    command_length++;
  }
  if (command[command_length] != '\0') {
    this->command_buffer_[0] = '\0';
    ESP_LOGE(TAG, "AS7261 AT command is too long");
    return false;
  }
  this->command_buffer_[command_length] = '\0';

  this->clear_transport_buffers_();
  this->drain_uart_();
  this->transport_state_ = TransportState::WAITING_RESPONSE;
  this->last_transport_result_ = TransportResult::NONE;
  this->transport_started_millis_ = millis();
  this->transport_timeout_ms_ = timeout_ms;

  this->write_str(this->command_buffer_);
  this->write_str("\r\n");
  ESP_LOGD(TAG, "AS7261 AT command started: %s", this->command_buffer_);
  return true;
}

void AS7261Component::poll_transport_() {
  if (this->transport_state_ == TransportState::RESET_ASSERTED) {
    if (millis() - this->reset_started_millis_ >= RESET_PULSE_MS) {
      this->release_reset_pulse_();
    }
    return;
  }

  if (this->transport_state_ != TransportState::WAITING_RESPONSE) {
    return;
  }

  while (this->available() > 0) {
    uint8_t byte;
    if (!this->read_byte(&byte)) {
      break;
    }
    this->handle_uart_byte_(byte);
    if (this->transport_state_ != TransportState::WAITING_RESPONSE) {
      return;
    }
  }

  if (millis() - this->transport_started_millis_ >= this->transport_timeout_ms_) {
    this->complete_transport_(TransportResult::TIMEOUT);
  }
}

void AS7261Component::handle_uart_byte_(uint8_t byte) {
  if (byte == '\r') {
    return;
  }
  if (byte == '\n') {
    if (this->line_length_ > 0) {
      this->finish_response_line_();
    }
    return;
  }
  if (byte >= 0x7F) {
    byte = '?';
  }
  if (this->line_length_ >= LINE_BUFFER_LENGTH - 1) {
    this->line_buffer_[LINE_BUFFER_LENGTH - 1] = '\0';
    this->complete_transport_(TransportResult::OVERFLOW);
    return;
  }
  this->line_buffer_[this->line_length_++] = static_cast<char>(byte);
}

void AS7261Component::finish_response_line_() {
  this->line_buffer_[this->line_length_] = '\0';
  ESP_LOGVV(TAG, "AS7261 RX: %s", this->line_buffer_);

  const bool ok_response = std::strcmp(this->line_buffer_, "OK") == 0;
  const bool error_response = std::strncmp(this->line_buffer_, "ERROR", 5) == 0;
  if (!this->append_response_line_(this->line_buffer_)) {
    this->line_length_ = 0;
    this->complete_transport_(TransportResult::OVERFLOW);
    return;
  }
  this->line_length_ = 0;

  if (ok_response) {
    this->complete_transport_(TransportResult::OK);
  } else if (error_response) {
    this->complete_transport_(TransportResult::ERROR);
  }
}

bool AS7261Component::append_response_line_(const char *line) {
  size_t line_length = 0;
  while (line[line_length] != '\0') {
    line_length++;
  }

  const size_t separator_length = this->response_length_ > 0 ? 1 : 0;
  if (this->response_length_ + separator_length + line_length >= RESPONSE_BUFFER_LENGTH) {
    return false;
  }
  if (separator_length != 0) {
    this->response_buffer_[this->response_length_++] = '\n';
  }
  for (size_t i = 0; i < line_length; i++) {
    this->response_buffer_[this->response_length_++] = line[i];
  }
  this->response_buffer_[this->response_length_] = '\0';
  this->response_line_count_++;
  return true;
}

void AS7261Component::complete_transport_(TransportResult result) {
  this->last_transport_result_ = result;
  this->transport_state_ = TransportState::IDLE;
  this->line_length_ = 0;

  switch (result) {
    case TransportResult::OK:
      ESP_LOGD(TAG, "AS7261 AT command %s completed with %u response line(s)", this->command_buffer_,
               this->response_line_count_);
      break;
    case TransportResult::ERROR:
      ESP_LOGW(TAG, "AS7261 AT command %s returned error: %s", this->command_buffer_, this->response_buffer_);
      break;
    case TransportResult::TIMEOUT:
      ESP_LOGW(TAG, "AS7261 AT command %s timed out after %u ms", this->command_buffer_,
               static_cast<unsigned>(this->transport_timeout_ms_));
      break;
    case TransportResult::OVERFLOW:
      ESP_LOGW(TAG, "AS7261 AT command %s response exceeded %zu bytes", this->command_buffer_, RESPONSE_BUFFER_LENGTH);
      break;
    case TransportResult::NONE:
      ESP_LOGD(TAG, "AS7261 AT command %s completed without a terminal result", this->command_buffer_);
      break;
  }
}

void AS7261Component::clear_transport_buffers_() {
  this->line_length_ = 0;
  this->response_length_ = 0;
  this->response_line_count_ = 0;
  this->line_buffer_[0] = '\0';
  this->response_buffer_[0] = '\0';
}

void AS7261Component::drain_uart_() {
  uint8_t buffer[32];
  size_t available = this->available();
  while (available > 0) {
    const size_t to_read = available > sizeof(buffer) ? sizeof(buffer) : available;
    if (!this->read_array(buffer, to_read)) {
      return;
    }
    available = this->available();
  }
}

void AS7261Component::start_reset_pulse_(const char *reason) {
  if (this->reset_pin_ == nullptr || this->transport_state_ == TransportState::RESET_ASSERTED) {
    return;
  }
  this->clear_transport_buffers_();
  this->reset_pin_->digital_write(true);
  this->reset_started_millis_ = millis();
  this->transport_state_ = TransportState::RESET_ASSERTED;
  if (reason != nullptr) {
    ESP_LOGD(TAG, "AS7261 reset pulse started: %s", reason);
  }
}

void AS7261Component::release_reset_pulse_() {
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->digital_write(false);
  }
  if (this->transport_state_ == TransportState::RESET_ASSERTED) {
    this->transport_state_ = TransportState::IDLE;
  }
}

const char *AS7261Component::transport_result_to_string_(TransportResult result) {
  switch (result) {
    case TransportResult::NONE:
      return "none";
    case TransportResult::OK:
      return "ok";
    case TransportResult::ERROR:
      return "error";
    case TransportResult::TIMEOUT:
      return "timeout";
    case TransportResult::OVERFLOW:
      return "overflow";
    default:
      return "unknown";
  }
}

const char *AS7261Component::gain_to_string_() const {
  switch (this->gain_) {
    case AS7261_GAIN_1X:
      return "1X";
    case AS7261_GAIN_3_7X:
      return "3.7X";
    case AS7261_GAIN_16X:
      return "16X";
    case AS7261_GAIN_64X:
      return "64X";
    default:
      return "unknown";
  }
}

}  // namespace esphome::as7261
