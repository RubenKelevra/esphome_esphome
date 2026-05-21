#include "as7261.h"

#include <cmath>
#include <cstdio>
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

void AS7261Component::loop() {
  this->poll_transport_();
  this->poll_command_sequence_();
  this->poll_frame_trigger_();
}

void AS7261Component::update() {
  if (this->component_busy_()) {
    return;
  }
  if (this->manual_exposure_state_ == ManualExposureCommandState::PENDING) {
    if (!this->start_manual_exposure_commands_()) {
      ESP_LOGW(TAG, "Unable to start AS7261 manual exposure commands");
    }
    return;
  }

  if (!this->start_diagnostic_readout_()) {
    ESP_LOGW(TAG, "Unable to start AS7261 diagnostic readout");
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

void AS7261Component::poll_command_sequence_() {
  if (!this->sequence_active_() || this->transport_busy_()) {
    return;
  }
  this->handle_finished_sequence_step_();
}

bool AS7261Component::start_command_sequence_(const CommandSequenceStep *steps, size_t step_count) {
  if (steps == nullptr || step_count == 0 || step_count > COMMAND_SEQUENCE_LENGTH || this->transport_busy_() ||
      this->sequence_active_()) {
    return false;
  }

  for (size_t i = 0; i < step_count; i++) {
    if (steps[i].command == nullptr || steps[i].command[0] == '\0') {
      return false;
    }
    this->command_sequence_[i] = steps[i];
  }
  this->sequence_step_count_ = step_count;
  this->sequence_step_index_ = 0;
  this->sequence_status_ = SequenceStatus::RUNNING;

  if (this->start_current_sequence_step_()) {
    return true;
  }

  this->finish_command_sequence_(SequenceStatus::FAILED);
  return false;
}

bool AS7261Component::start_current_sequence_step_() {
  if (!this->sequence_active_() || this->sequence_step_index_ >= this->sequence_step_count_) {
    return false;
  }

  const CommandSequenceStep &step = this->command_sequence_[this->sequence_step_index_];
  this->diagnostic_state_ = step.diagnostic_state;
  if (this->begin_at_command_(step.command)) {
    return true;
  }
  this->diagnostic_state_ = DiagnosticState::IDLE;
  return false;
}

void AS7261Component::handle_finished_sequence_step_() {
  if (this->sequence_step_index_ >= this->sequence_step_count_) {
    this->finish_command_sequence_(SequenceStatus::COMPLETED);
    return;
  }

  const CommandSequenceStep &step = this->command_sequence_[this->sequence_step_index_];
  const TransportResult result = this->last_transport_result_;
  if (step.diagnostic_state != DiagnosticState::IDLE) {
    this->handle_finished_diagnostic_command_(step.diagnostic_state, result);
  }
  if (this->sequence_owner_ == SequenceOwner::CALIBRATED_FRAME_READOUT &&
      !this->handle_finished_calibrated_frame_command_(this->sequence_step_index_, result)) {
    this->finish_command_sequence_(SequenceStatus::FAILED);
    return;
  }

  if (result != TransportResult::OK && step.failure_policy == SequenceFailurePolicy::STOP) {
    this->finish_command_sequence_(result == TransportResult::TIMEOUT    ? SequenceStatus::TIMEOUT
                                   : result == TransportResult::OVERFLOW ? SequenceStatus::OVERFLOW
                                                                         : SequenceStatus::FAILED);
    return;
  }

  this->sequence_step_index_++;
  this->diagnostic_state_ = DiagnosticState::IDLE;
  if (this->sequence_step_index_ >= this->sequence_step_count_) {
    this->finish_command_sequence_(SequenceStatus::COMPLETED);
    return;
  }

  if (!this->start_current_sequence_step_()) {
    this->finish_command_sequence_(SequenceStatus::FAILED);
  }
}

void AS7261Component::finish_command_sequence_(SequenceStatus status) {
  const SequenceOwner owner = this->sequence_owner_;
  this->sequence_status_ = status;
  this->sequence_step_count_ = 0;
  this->sequence_step_index_ = 0;
  this->diagnostic_state_ = DiagnosticState::IDLE;
  this->sequence_owner_ = SequenceOwner::NONE;
  ESP_LOGD(TAG, "AS7261 command sequence finished with %s", this->sequence_status_to_string_(status));

  switch (owner) {
    case SequenceOwner::DIAGNOSTIC_READOUT:
      this->handle_finished_diagnostic_readout_(status);
      break;
    case SequenceOwner::MANUAL_EXPOSURE:
      this->handle_finished_manual_exposure_commands_(status);
      break;
    case SequenceOwner::FRAME_TRIGGER:
      this->handle_finished_frame_trigger_(status);
      break;
    case SequenceOwner::RAW_FRAME_READOUT:
      this->handle_finished_raw_frame_readout_(status);
      break;
    case SequenceOwner::CALIBRATED_FRAME_READOUT:
      this->handle_finished_calibrated_frame_readout_(status);
      break;
    case SequenceOwner::NONE:
      break;
  }
}

bool AS7261Component::start_manual_exposure_commands_() {
  if (!this->manual_exposure_ || this->component_busy_()) {
    return false;
  }

  const int gain_command_length = std::snprintf(this->manual_exposure_commands_[0], COMMAND_BUFFER_LENGTH, "ATGAIN=%u",
                                                static_cast<unsigned>(gain_to_at_value_(this->gain_)));
  const int integration_time_command_length =
      std::snprintf(this->manual_exposure_commands_[1], COMMAND_BUFFER_LENGTH, "ATINTTIME=%u",
                    static_cast<unsigned>(this->integration_time_));
  if (gain_command_length <= 0 || gain_command_length >= static_cast<int>(COMMAND_BUFFER_LENGTH) ||
      integration_time_command_length <= 0 ||
      integration_time_command_length >= static_cast<int>(COMMAND_BUFFER_LENGTH)) {
    this->manual_exposure_state_ = ManualExposureCommandState::FAILED;
    return false;
  }

  const CommandSequenceStep steps[] = {
      {this->manual_exposure_commands_[0], DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {this->manual_exposure_commands_[1], DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->manual_exposure_state_ = ManualExposureCommandState::RUNNING;
  this->sequence_owner_ = SequenceOwner::MANUAL_EXPOSURE;
  if (this->start_command_sequence_(steps, 2)) {
    return true;
  }
  this->manual_exposure_state_ = ManualExposureCommandState::FAILED;
  this->sequence_owner_ = SequenceOwner::NONE;
  return false;
}

void AS7261Component::handle_finished_manual_exposure_commands_(SequenceStatus status) {
  if (status == SequenceStatus::COMPLETED) {
    this->manual_exposure_state_ = ManualExposureCommandState::APPLIED;
    ESP_LOGD(TAG, "AS7261 manual exposure applied");
    return;
  }
  this->manual_exposure_state_ = ManualExposureCommandState::FAILED;
  ESP_LOGW(TAG, "AS7261 manual exposure commands failed with %s", this->sequence_status_to_string_(status));
}

bool AS7261Component::start_diagnostic_readout_() {
#ifdef USE_TEXT_SENSOR
  if (this->firmware_version_text_sensor_ != nullptr) {
    const CommandSequenceStep steps[] = {
        {"ATVERSW", DiagnosticState::FIRMWARE_VERSION, SequenceFailurePolicy::CONTINUE},
        {"ATTEMP", DiagnosticState::DEVICE_TEMPERATURE, SequenceFailurePolicy::STOP},
    };
    this->sequence_owner_ = SequenceOwner::DIAGNOSTIC_READOUT;
    if (this->start_command_sequence_(steps, 2)) {
      return true;
    }
    this->sequence_owner_ = SequenceOwner::NONE;
    return false;
  }
#endif
  const CommandSequenceStep steps[] = {
      {"ATTEMP", DiagnosticState::DEVICE_TEMPERATURE, SequenceFailurePolicy::STOP},
  };
  this->sequence_owner_ = SequenceOwner::DIAGNOSTIC_READOUT;
  if (this->start_command_sequence_(steps, 1)) {
    return true;
  }
  this->sequence_owner_ = SequenceOwner::NONE;
  return false;
}

void AS7261Component::handle_finished_diagnostic_readout_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    ESP_LOGW(TAG, "AS7261 diagnostic readout failed with %s", this->sequence_status_to_string_(status));
    return;
  }
  if (this->manual_exposure_ && this->manual_exposure_state_ != ManualExposureCommandState::APPLIED) {
    ESP_LOGW(TAG, "Skipping AS7261 one-shot trigger because manual exposure is not applied");
    return;
  }
  if (this->device_temperature_unsafe_) {
    ESP_LOGW(TAG, "Skipping AS7261 one-shot trigger while device temperature is unsafe");
    return;
  }
  if (!this->start_one_shot_frame_trigger_()) {
    ESP_LOGW(TAG, "Unable to start AS7261 one-shot frame trigger");
    this->frame_state_ = FrameState::ERROR;
  }
}

bool AS7261Component::start_one_shot_frame_trigger_() {
  if (this->frame_state_ == FrameState::TIMEOUT || this->frame_state_ == FrameState::ERROR) {
    this->clear_terminal_frame_state_();
  }

  if (this->component_busy_() || this->frame_state_ != FrameState::IDLE) {
    return false;
  }
  const CommandSequenceStep steps[] = {
      {"ATTCSMD=3", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->frame_state_ = FrameState::TRIGGER_RUNNING;
  this->sequence_owner_ = SequenceOwner::FRAME_TRIGGER;
  if (this->start_command_sequence_(steps, 1)) {
    return true;
  }
  this->sequence_owner_ = SequenceOwner::NONE;
  this->frame_state_ = FrameState::ERROR;
  return false;
}

void AS7261Component::handle_finished_frame_trigger_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    this->frame_state_ = FrameState::ERROR;
    ESP_LOGW(TAG, "AS7261 one-shot trigger failed with %s", this->sequence_status_to_string_(status));
    return;
  }
  if (this->int_pin_ == nullptr) {
    this->frame_state_ = FrameState::ERROR;
    ESP_LOGE(TAG, "AS7261 INT pin is not configured; cannot wait for frame completion");
    return;
  }
  this->frame_wait_started_millis_ = millis();
  this->frame_watchdog_timeout_ms_ = this->calculate_frame_watchdog_timeout_ms_();
  this->frame_state_ = FrameState::WAITING_INT;
  ESP_LOGD(TAG, "AS7261 one-shot trigger sent; waiting up to %u ms for INT",
           static_cast<unsigned>(this->frame_watchdog_timeout_ms_));
}

void AS7261Component::poll_frame_trigger_() {
  if (this->frame_state_ == FrameState::READY) {
    if (!this->start_raw_frame_readout_()) {
      this->raw_frame_ = RawFrame{};
      this->raw_frame_status_ = RawFrameStatus::FAILED;
      this->calibrated_frame_ = CalibratedFrame{};
      this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
      this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
      this->clear_derived_color_(DerivedColorStatus::FAILED);
      this->clear_terminal_frame_state_();
      ESP_LOGW(TAG, "Unable to start AS7261 raw frame readout");
    }
    return;
  }

  if (this->frame_state_ == FrameState::TIMEOUT) {
    this->raw_frame_ = RawFrame{};
    this->raw_frame_status_ = RawFrameStatus::TIMEOUT;
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::TIMEOUT;
    this->clear_calculated_duv_(CalculatedDuvStatus::TIMEOUT);
    this->clear_derived_color_(DerivedColorStatus::TIMEOUT);
    this->clear_terminal_frame_state_();
    return;
  }

  if (this->frame_state_ == FrameState::ERROR) {
    this->raw_frame_ = RawFrame{};
    this->raw_frame_status_ = RawFrameStatus::FAILED;
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
    this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
    this->clear_derived_color_(DerivedColorStatus::FAILED);
    this->clear_terminal_frame_state_();
    return;
  }

  if (this->frame_state_ != FrameState::WAITING_INT) {
    return;
  }
  if (this->frame_int_ready_()) {
    this->frame_state_ = FrameState::READY;
    ESP_LOGD(TAG, "AS7261 one-shot frame is ready");
    return;
  }
  if (millis() - this->frame_wait_started_millis_ >= this->frame_watchdog_timeout_ms_) {
    this->frame_state_ = FrameState::TIMEOUT;
    ESP_LOGW(TAG, "AS7261 one-shot frame timed out waiting for INT after %u ms",
             static_cast<unsigned>(this->frame_watchdog_timeout_ms_));
  }
}

bool AS7261Component::start_raw_frame_readout_() {
  if (this->transport_busy_() || this->sequence_active_() || this->frame_state_ != FrameState::READY) {
    return false;
  }

  const CommandSequenceStep steps[] = {
      {"ATDATA", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->raw_frame_ = RawFrame{};
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::INVALID;
  this->clear_calculated_duv_(CalculatedDuvStatus::INVALID);
  this->clear_derived_color_(DerivedColorStatus::INVALID);
  this->frame_state_ = FrameState::READOUT_RUNNING;
  this->raw_frame_status_ = RawFrameStatus::RUNNING;
  this->sequence_owner_ = SequenceOwner::RAW_FRAME_READOUT;
  if (this->start_command_sequence_(steps, 1)) {
    return true;
  }

  this->sequence_owner_ = SequenceOwner::NONE;
  this->raw_frame_status_ = RawFrameStatus::FAILED;
  this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
  this->clear_derived_color_(DerivedColorStatus::FAILED);
  this->clear_terminal_frame_state_();
  return false;
}

void AS7261Component::handle_finished_raw_frame_readout_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    this->raw_frame_ = RawFrame{};
    this->raw_frame_status_ = status == SequenceStatus::TIMEOUT    ? RawFrameStatus::TIMEOUT
                              : status == SequenceStatus::OVERFLOW ? RawFrameStatus::OVERFLOW
                                                                   : RawFrameStatus::FAILED;
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = status == SequenceStatus::TIMEOUT    ? CalibratedFrameStatus::TIMEOUT
                                     : status == SequenceStatus::OVERFLOW ? CalibratedFrameStatus::OVERFLOW
                                                                          : CalibratedFrameStatus::FAILED;
    this->clear_calculated_duv_(status == SequenceStatus::TIMEOUT    ? CalculatedDuvStatus::TIMEOUT
                                : status == SequenceStatus::OVERFLOW ? CalculatedDuvStatus::OVERFLOW
                                                                     : CalculatedDuvStatus::FAILED);
    this->clear_derived_color_(status == SequenceStatus::TIMEOUT    ? DerivedColorStatus::TIMEOUT
                               : status == SequenceStatus::OVERFLOW ? DerivedColorStatus::OVERFLOW
                                                                    : DerivedColorStatus::FAILED);
    this->clear_terminal_frame_state_();
    ESP_LOGW(TAG, "AS7261 raw frame readout failed with %s", this->sequence_status_to_string_(status));
    return;
  }

  RawFrame frame{};
  if (!parse_raw_frame_(this->first_response_value_(), &frame)) {
    this->raw_frame_ = RawFrame{};
    this->raw_frame_status_ = RawFrameStatus::MALFORMED;
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::MALFORMED;
    this->clear_calculated_duv_(CalculatedDuvStatus::MALFORMED);
    this->clear_derived_color_(DerivedColorStatus::MALFORMED);
    this->clear_terminal_frame_state_();
    ESP_LOGW(TAG, "Unable to parse AS7261 raw frame response: %s", this->response_buffer_);
    return;
  }

  this->raw_frame_ = frame;
  this->raw_frame_status_ = RawFrameStatus::VALID;
  this->clear_terminal_frame_state_();
  ESP_LOGD(TAG, "AS7261 raw frame stored: X=%u Y=%u Z=%u NIR=%u Dark=%u Clear=%u",
           static_cast<unsigned>(this->raw_frame_.x), static_cast<unsigned>(this->raw_frame_.y),
           static_cast<unsigned>(this->raw_frame_.z), static_cast<unsigned>(this->raw_frame_.near_ir),
           static_cast<unsigned>(this->raw_frame_.dark), static_cast<unsigned>(this->raw_frame_.clear));
  if (!this->start_calibrated_frame_readout_()) {
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
    this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
    this->clear_derived_color_(DerivedColorStatus::FAILED);
    ESP_LOGW(TAG, "Unable to start AS7261 calibrated frame readout");
  }
}

bool AS7261Component::start_calibrated_frame_readout_() {
  if (this->transport_busy_() || this->sequence_active_() || this->raw_frame_status_ != RawFrameStatus::VALID) {
    return false;
  }

  const CommandSequenceStep steps[] = {
      {"ATXYZC", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {"ATLUXC", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {"ATCCTC", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::RUNNING;
  this->clear_calculated_duv_(CalculatedDuvStatus::INVALID);
  this->clear_derived_color_(DerivedColorStatus::INVALID);
  this->sequence_owner_ = SequenceOwner::CALIBRATED_FRAME_READOUT;
  if (this->start_command_sequence_(steps, 3)) {
    return true;
  }

  this->sequence_owner_ = SequenceOwner::NONE;
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
  this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
  this->clear_derived_color_(DerivedColorStatus::FAILED);
  return false;
}

bool AS7261Component::handle_finished_calibrated_frame_command_(size_t step_index, TransportResult result) {
  if (result != TransportResult::OK) {
    return true;
  }

  const char *const value = this->first_response_value_();
  if (step_index == 0) {
    CalibratedFrame xyz{};
    if (!parse_calibrated_xyz_(value, &xyz)) {
      ESP_LOGW(TAG, "Unable to parse AS7261 calibrated XYZ response: %s", value == nullptr ? "<empty>" : value);
      this->calibrated_frame_ = CalibratedFrame{};
      this->calibrated_frame_status_ = CalibratedFrameStatus::MALFORMED;
      this->clear_calculated_duv_(CalculatedDuvStatus::MALFORMED);
      this->clear_derived_color_(DerivedColorStatus::MALFORMED);
      return false;
    }
    this->calibrated_frame_.x = xyz.x;
    this->calibrated_frame_.y = xyz.y;
    this->calibrated_frame_.z = xyz.z;
    return true;
  }

  float parsed = 0.0f;
  if (!parse_calibrated_value_(value, &parsed)) {
    ESP_LOGW(TAG, "Unable to parse AS7261 calibrated %s response: %s", step_index == 1 ? "lux" : "CCT",
             value == nullptr ? "<empty>" : value);
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::MALFORMED;
    this->clear_calculated_duv_(CalculatedDuvStatus::MALFORMED);
    this->clear_derived_color_(DerivedColorStatus::MALFORMED);
    return false;
  }
  if (step_index == 1) {
    this->calibrated_frame_.lux = parsed;
    return true;
  }
  if (step_index == 2) {
    this->calibrated_frame_.cct = parsed;
    return true;
  }

  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::MALFORMED;
  this->clear_calculated_duv_(CalculatedDuvStatus::MALFORMED);
  this->clear_derived_color_(DerivedColorStatus::MALFORMED);
  return false;
}

void AS7261Component::handle_finished_calibrated_frame_readout_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    if (this->calibrated_frame_status_ != CalibratedFrameStatus::MALFORMED) {
      this->calibrated_frame_ = CalibratedFrame{};
      this->calibrated_frame_status_ = status == SequenceStatus::TIMEOUT    ? CalibratedFrameStatus::TIMEOUT
                                       : status == SequenceStatus::OVERFLOW ? CalibratedFrameStatus::OVERFLOW
                                                                            : CalibratedFrameStatus::FAILED;
      this->clear_calculated_duv_(status == SequenceStatus::TIMEOUT    ? CalculatedDuvStatus::TIMEOUT
                                  : status == SequenceStatus::OVERFLOW ? CalculatedDuvStatus::OVERFLOW
                                                                       : CalculatedDuvStatus::FAILED);
      this->clear_derived_color_(status == SequenceStatus::TIMEOUT    ? DerivedColorStatus::TIMEOUT
                                 : status == SequenceStatus::OVERFLOW ? DerivedColorStatus::OVERFLOW
                                                                      : DerivedColorStatus::FAILED);
    }
    ESP_LOGW(TAG, "AS7261 calibrated frame readout failed with %s",
             this->calibrated_frame_status_to_string_(this->calibrated_frame_status_));
    return;
  }

  this->calibrated_frame_status_ = CalibratedFrameStatus::VALID;
  ESP_LOGD(TAG, "AS7261 calibrated frame stored: X=%f Y=%f Z=%f Lux=%f CCT=%f", this->calibrated_frame_.x,
           this->calibrated_frame_.y, this->calibrated_frame_.z, this->calibrated_frame_.lux,
           this->calibrated_frame_.cct);
  if (!this->derive_calculated_duv_()) {
    ESP_LOGW(TAG, "AS7261 calculated Duv derivation failed with %s",
             this->calculated_duv_status_to_string_(this->calculated_duv_status_));
  }
  if (!this->derive_oklab_oklch_()) {
    ESP_LOGW(TAG, "AS7261 OKLab/OKLCH derivation failed with %s",
             this->derived_color_status_to_string_(this->derived_color_status_));
  }
}

void AS7261Component::clear_calculated_duv_(CalculatedDuvStatus status) {
  this->calculated_duv_frame_ = CalculatedDuvFrame{};
  this->calculated_duv_status_ = status;
}

bool AS7261Component::derive_calculated_duv_() {
  if (this->calibrated_frame_status_ != CalibratedFrameStatus::VALID ||
      !calibrated_frame_valid_for_duv_(this->calibrated_frame_)) {
    this->clear_calculated_duv_(CalculatedDuvStatus::MALFORMED);
    return false;
  }

  Cie1960UcsPoint measured{};
  Cie1960UcsPoint planckian{};
  if (!cie1960_uv_from_xyz_(this->calibrated_frame_, &measured) ||
      !planckian_locus_uv_from_cct_(this->calibrated_frame_.cct, &planckian)) {
    this->clear_calculated_duv_(CalculatedDuvStatus::MALFORMED);
    return false;
  }

  const float delta_u = measured.u - planckian.u;
  const float delta_v = measured.v - planckian.v;
  const float distance = std::sqrt((delta_u * delta_u) + (delta_v * delta_v));
  if (!std::isfinite(distance)) {
    this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
    return false;
  }

  CalculatedDuvFrame calculated{};
  calculated.measured = measured;
  calculated.planckian = planckian;
  // Signed CIE 1960 Duv: positive above the CCT-derived Planckian reference v, negative below it.
  calculated.duv = delta_v >= 0.0f ? distance : -distance;
  if (!std::isfinite(calculated.duv)) {
    this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
    return false;
  }

  this->calculated_duv_frame_ = calculated;
  this->calculated_duv_status_ = CalculatedDuvStatus::VALID;
  ESP_LOGD(TAG, "AS7261 calculated CIE 1960 Duv stored: Duv=%f u=%f v=%f reference_u=%f reference_v=%f",
           this->calculated_duv_frame_.duv, this->calculated_duv_frame_.measured.u,
           this->calculated_duv_frame_.measured.v, this->calculated_duv_frame_.planckian.u,
           this->calculated_duv_frame_.planckian.v);
  return true;
}

bool AS7261Component::calibrated_frame_valid_for_duv_(const CalibratedFrame &frame) {
  return std::isfinite(frame.x) && std::isfinite(frame.y) && std::isfinite(frame.z) && std::isfinite(frame.cct) &&
         frame.x >= 0.0f && frame.y >= 0.0f && frame.z >= 0.0f && frame.cct >= PLANCKIAN_LOCUS_MIN_CCT_K &&
         frame.cct <= PLANCKIAN_LOCUS_MAX_CCT_K;
}

bool AS7261Component::cie1960_uv_from_xyz_(const CalibratedFrame &frame, Cie1960UcsPoint *point) {
  if (point == nullptr) {
    return false;
  }
  const float denominator = frame.x + (15.0f * frame.y) + (3.0f * frame.z);
  if (!std::isfinite(denominator) || denominator <= CIE_1960_UCS_DENOMINATOR_EPSILON) {
    return false;
  }

  Cie1960UcsPoint parsed{};
  parsed.u = (4.0f * frame.x) / denominator;
  parsed.v = (6.0f * frame.y) / denominator;
  if (!std::isfinite(parsed.u) || !std::isfinite(parsed.v)) {
    return false;
  }

  *point = parsed;
  return true;
}

bool AS7261Component::cie1960_uv_from_xy_(float x, float y, Cie1960UcsPoint *point) {
  if (point == nullptr || !std::isfinite(x) || !std::isfinite(y) || x < 0.0f || y < 0.0f) {
    return false;
  }
  const float denominator = (-2.0f * x) + (12.0f * y) + 3.0f;
  if (!std::isfinite(denominator) || denominator <= CIE_1960_UCS_DENOMINATOR_EPSILON) {
    return false;
  }

  Cie1960UcsPoint parsed{};
  parsed.u = (4.0f * x) / denominator;
  parsed.v = (6.0f * y) / denominator;
  if (!std::isfinite(parsed.u) || !std::isfinite(parsed.v)) {
    return false;
  }

  *point = parsed;
  return true;
}

bool AS7261Component::planckian_locus_uv_from_cct_(float cct, Cie1960UcsPoint *point) {
  if (point == nullptr || !std::isfinite(cct) || cct < PLANCKIAN_LOCUS_MIN_CCT_K || cct > PLANCKIAN_LOCUS_MAX_CCT_K) {
    return false;
  }

  const float inverse_cct = 1.0f / cct;
  const float inverse_cct2 = inverse_cct * inverse_cct;
  const float inverse_cct3 = inverse_cct2 * inverse_cct;
  const float x =
      cct <= PLANCKIAN_LOCUS_MID_CCT_K
          ? (-0.2661239e9f * inverse_cct3) - (0.2343580e6f * inverse_cct2) + (0.8776956e3f * inverse_cct) + 0.179910f
          : (-3.0258469e9f * inverse_cct3) + (2.1070379e6f * inverse_cct2) + (0.2226347e3f * inverse_cct) + 0.240390f;
  const float x2 = x * x;
  const float x3 = x2 * x;
  const float y =
      cct <= PLANCKIAN_LOCUS_LOW_CCT_K   ? (-1.1063814f * x3) - (1.34811020f * x2) + (2.18555832f * x) - 0.20219683f
      : cct <= PLANCKIAN_LOCUS_MID_CCT_K ? (-0.9549476f * x3) - (1.37418593f * x2) + (2.09137015f * x) - 0.16748867f
                                         : (3.0817580f * x3) - (5.87338670f * x2) + (3.75112997f * x) - 0.37001483f;
  return cie1960_uv_from_xy_(x, y, point);
}

void AS7261Component::clear_derived_color_(DerivedColorStatus status) {
  this->derived_color_frame_ = DerivedColorFrame{};
  this->derived_color_status_ = status;
}

bool AS7261Component::derive_oklab_oklch_() {
  if (this->calibrated_frame_status_ != CalibratedFrameStatus::VALID ||
      !calibrated_xyz_valid_for_oklab_(this->calibrated_frame_)) {
    this->clear_derived_color_(DerivedColorStatus::MALFORMED);
    return false;
  }

  const float normalized_x = this->calibrated_frame_.x / OKLAB_REFERENCE_ILLUMINANCE_LX;
  const float normalized_y = this->calibrated_frame_.y / OKLAB_REFERENCE_ILLUMINANCE_LX;
  const float normalized_z = this->calibrated_frame_.z / OKLAB_REFERENCE_ILLUMINANCE_LX;

  const float l =
      std::cbrt((0.8189330101f * normalized_x) + (0.3618667424f * normalized_y) - (0.1288597137f * normalized_z));
  const float m =
      std::cbrt((0.0329845436f * normalized_x) + (0.9293118715f * normalized_y) + (0.0361456387f * normalized_z));
  const float s =
      std::cbrt((0.0482003018f * normalized_x) + (0.2643662691f * normalized_y) + (0.6338517070f * normalized_z));

  DerivedColorFrame derived{};
  derived.oklab.l = (0.2104542553f * l) + (0.7936177850f * m) - (0.0040720468f * s);
  derived.oklab.a = (1.9779984951f * l) - (2.4285922050f * m) + (0.4505937099f * s);
  derived.oklab.b = (0.0259040371f * l) + (0.7827717662f * m) - (0.8086757660f * s);

  derived.oklch.l = derived.oklab.l;
  derived.oklch.c = std::sqrt((derived.oklab.a * derived.oklab.a) + (derived.oklab.b * derived.oklab.b));
  derived.oklch.h = OKLCH_ZERO_CHROMA_HUE_DEGREES;
  if (derived.oklch.c > 0.0f) {
    derived.oklch.h = std::atan2(derived.oklab.b, derived.oklab.a) * DEGREES_PER_RADIAN;
    if (derived.oklch.h < 0.0f) {
      derived.oklch.h += 360.0f;
    }
    if (derived.oklch.h >= 360.0f) {
      derived.oklch.h -= 360.0f;
    }
  }

  if (!std::isfinite(derived.oklab.l) || !std::isfinite(derived.oklab.a) || !std::isfinite(derived.oklab.b) ||
      !std::isfinite(derived.oklch.l) || !std::isfinite(derived.oklch.c) || !std::isfinite(derived.oklch.h)) {
    this->clear_derived_color_(DerivedColorStatus::FAILED);
    return false;
  }

  this->derived_color_frame_ = derived;
  this->derived_color_status_ = DerivedColorStatus::VALID;
  ESP_LOGD(TAG, "AS7261 derived color stored: OKLab L=%f a=%f b=%f OKLCH L=%f C=%f h=%f",
           this->derived_color_frame_.oklab.l, this->derived_color_frame_.oklab.a, this->derived_color_frame_.oklab.b,
           this->derived_color_frame_.oklch.l, this->derived_color_frame_.oklch.c, this->derived_color_frame_.oklch.h);
  return true;
}

bool AS7261Component::calibrated_xyz_valid_for_oklab_(const CalibratedFrame &frame) {
  return std::isfinite(frame.x) && std::isfinite(frame.y) && std::isfinite(frame.z) && frame.x >= 0.0f &&
         frame.y >= 0.0f && frame.z >= 0.0f && std::isfinite(OKLAB_REFERENCE_ILLUMINANCE_LX) &&
         OKLAB_REFERENCE_ILLUMINANCE_LX > 0.0f;
}

void AS7261Component::clear_terminal_frame_state_() {
  if (this->frame_state_ == FrameState::READY || this->frame_state_ == FrameState::READOUT_RUNNING ||
      this->frame_state_ == FrameState::TIMEOUT || this->frame_state_ == FrameState::ERROR) {
    this->frame_state_ = FrameState::IDLE;
  }
}

bool AS7261Component::frame_int_ready_() const { return this->int_pin_ != nullptr && this->int_pin_->digital_read(); }

uint32_t AS7261Component::calculate_frame_watchdog_timeout_ms_() const {
  const uint8_t integration_time = this->manual_exposure_ && this->integration_time_ != 0
                                       ? this->integration_time_
                                       : AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME;
  const uint32_t conversion_time_us = 2UL * static_cast<uint32_t>(integration_time) * AS7261_INTEGRATION_TIME_STEP_US;
  return ((conversion_time_us + 999UL) / 1000UL) + FRAME_WATCHDOG_MARGIN_MS;
}

void AS7261Component::handle_finished_diagnostic_command_(DiagnosticState state, TransportResult result) {
  if (result != TransportResult::OK) {
    ESP_LOGW(TAG, "AS7261 diagnostic command %s failed with %s", this->diagnostic_state_to_string_(state),
             this->transport_result_to_string_(result));
    return;
  }

  if (state == DiagnosticState::FIRMWARE_VERSION) {
    this->handle_firmware_version_response_();
    return;
  }

  if (state == DiagnosticState::DEVICE_TEMPERATURE) {
    this->handle_device_temperature_response_();
  }
}

void AS7261Component::handle_firmware_version_response_() {
#ifdef USE_TEXT_SENSOR
  if (this->firmware_version_text_sensor_ == nullptr) {
    return;
  }
  const char *const value = this->first_response_value_();
  if (value == nullptr || value[0] == '\0') {
    ESP_LOGW(TAG, "AS7261 firmware version response was empty");
    return;
  }
  this->firmware_version_text_sensor_->publish_state(value);
#endif
}

void AS7261Component::handle_device_temperature_response_() {
  const char *const value = this->first_response_value_();
  int16_t temperature_c = 0;
  bool invalid = false;
  if (!this->parse_device_temperature_(value, &temperature_c, &invalid)) {
    ESP_LOGW(TAG, "Unable to parse AS7261 device temperature response: %s", value == nullptr ? "<empty>" : value);
    this->device_temperature_valid_ = false;
    this->device_temperature_invalid_ = false;
#ifdef USE_SENSOR
    if (this->device_temperature_sensor_ != nullptr) {
      this->device_temperature_sensor_->publish_state(NAN);
    }
#endif
    return;
  }

  if (invalid) {
    this->device_temperature_valid_ = false;
    this->device_temperature_invalid_ = true;
    this->device_temperature_unsafe_ = false;
#ifdef USE_SENSOR
    if (this->device_temperature_sensor_ != nullptr) {
      this->device_temperature_sensor_->publish_state(NAN);
    }
#endif
    return;
  }

  this->last_device_temperature_c_ = temperature_c;
  this->device_temperature_valid_ = true;
  this->device_temperature_invalid_ = false;
  this->device_temperature_unsafe_ = static_cast<float>(temperature_c) >= DEVICE_TEMPERATURE_UNSAFE_C;
#ifdef USE_SENSOR
  if (this->device_temperature_sensor_ != nullptr) {
    this->device_temperature_sensor_->publish_state(static_cast<float>(temperature_c));
  }
#endif
  if (this->device_temperature_unsafe_) {
    ESP_LOGW(TAG, "AS7261 reported unsafe device temperature: %d C", static_cast<int>(temperature_c));
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

  if (this->finish_response_line_with_terminal_("OK", TransportResult::OK)) {
    return;
  }

  const char *const line_begin = this->trim_left_(this->line_buffer_);
  if (std::strncmp(line_begin, "ERROR", 5) == 0) {
    if (!this->append_response_line_(line_begin)) {
      this->line_length_ = 0;
      this->complete_transport_(TransportResult::OVERFLOW);
      return;
    }
    this->line_length_ = 0;
    this->complete_transport_(TransportResult::ERROR);
    return;
  }

  if (!this->append_response_line_(this->line_buffer_)) {
    this->line_length_ = 0;
    this->complete_transport_(TransportResult::OVERFLOW);
    return;
  }
  this->line_length_ = 0;
}

bool AS7261Component::finish_response_line_with_terminal_(const char *terminal, TransportResult result) {
  const size_t terminal_length = std::strlen(terminal);
  const char *const line_begin = this->trim_left_(this->line_buffer_);
  const char *const line_end = this->trim_right_(line_begin, line_begin + std::strlen(line_begin));
  if (line_end < line_begin + terminal_length ||
      std::strncmp(line_end - terminal_length, terminal, terminal_length) != 0) {
    return false;
  }
  if (line_end != line_begin + terminal_length && !this->is_space_(*(line_end - terminal_length - 1))) {
    return false;
  }

  const char *const value_end = this->trim_right_(line_begin, line_end - terminal_length);
  if (value_end > line_begin) {
    char value_line[LINE_BUFFER_LENGTH];
    const size_t value_length = static_cast<size_t>(value_end - line_begin);
    if (value_length >= sizeof(value_line)) {
      this->line_length_ = 0;
      this->complete_transport_(TransportResult::OVERFLOW);
      return true;
    }
    std::memcpy(value_line, line_begin, value_length);
    value_line[value_length] = '\0';
    if (!this->append_response_line_(value_line)) {
      this->line_length_ = 0;
      this->complete_transport_(TransportResult::OVERFLOW);
      return true;
    }
  }

  this->line_length_ = 0;
  this->complete_transport_(result);
  return true;
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

const char *AS7261Component::first_response_value_() {
  const char *const begin = this->trim_left_(this->response_buffer_);
  if (begin == nullptr) {
    this->line_buffer_[0] = '\0';
    return this->line_buffer_;
  }

  const char *line_end = begin;
  while (*line_end != '\0' && *line_end != '\n') {
    line_end++;
  }
  const char *const end = this->trim_right_(begin, line_end);
  const size_t length = static_cast<size_t>(end - begin);
  if (length == 0 || length >= LINE_BUFFER_LENGTH) {
    this->line_buffer_[0] = '\0';
    return this->line_buffer_;
  }

  std::memcpy(this->line_buffer_, begin, length);
  this->line_buffer_[length] = '\0';
  return this->line_buffer_;
}

bool AS7261Component::parse_device_temperature_(const char *text, int16_t *temperature_c, bool *invalid) {
  if (text == nullptr || temperature_c == nullptr || invalid == nullptr) {
    return false;
  }

  *temperature_c = 0;
  *invalid = false;
  uint8_t value = 0;
  if (!parse_unsigned_byte_(text, &value)) {
    return false;
  }
  if (value == 0xFF) {
    *invalid = true;
    return true;
  }
  *temperature_c = static_cast<int16_t>(value);
  return true;
}

bool AS7261Component::parse_unsigned_byte_(const char *text, uint8_t *value) {
  if (text == nullptr || value == nullptr) {
    return false;
  }

  const char *begin = trim_left_(text);
  const char *end = begin;
  while (*end != '\0' && *end != '\n') {
    end++;
  }
  end = trim_right_(begin, end);
  if (begin >= end) {
    return false;
  }

  uint8_t base = 10;
  if (end - begin >= 2 && begin[0] == '0' && (begin[1] == 'x' || begin[1] == 'X')) {
    begin += 2;
    base = 16;
  } else if (end - begin >= 1 && (begin[0] == 'b' || begin[0] == 'B')) {
    begin += 1;
    base = 2;
  }
  if (begin >= end) {
    return false;
  }

  uint16_t parsed = 0;
  if (!parse_unsigned_digits_(begin, end, base, &parsed) || parsed > 0xFF) {
    return false;
  }
  *value = static_cast<uint8_t>(parsed);
  return true;
}

bool AS7261Component::parse_raw_frame_(const char *text, RawFrame *frame) {
  if (text == nullptr || frame == nullptr) {
    return false;
  }

  RawFrame parsed{};
  const char *cursor = text;
  if (!parse_raw_frame_field_(&cursor, &parsed.x, true) || !parse_raw_frame_field_(&cursor, &parsed.y, true) ||
      !parse_raw_frame_field_(&cursor, &parsed.z, true) || !parse_raw_frame_field_(&cursor, &parsed.near_ir, true) ||
      !parse_raw_frame_field_(&cursor, &parsed.dark, true) || !parse_raw_frame_field_(&cursor, &parsed.clear, false)) {
    return false;
  }

  cursor = trim_left_(cursor);
  if (*cursor != '\0' && *cursor != '\n') {
    return false;
  }

  *frame = parsed;
  return true;
}

bool AS7261Component::parse_raw_frame_field_(const char **cursor, uint16_t *value, bool expect_separator) {
  if (cursor == nullptr || *cursor == nullptr || value == nullptr) {
    return false;
  }

  const char *const begin = trim_left_(*cursor);
  const char *end = begin;
  while (*end != '\0' && *end != '\n' && *end != ',') {
    end++;
  }
  const char *const value_end = trim_right_(begin, end);
  if (!parse_unsigned_u16_(begin, value_end, value)) {
    return false;
  }

  *cursor = end;
  return expect_separator ? consume_raw_frame_separator_(cursor) : true;
}

bool AS7261Component::consume_raw_frame_separator_(const char **cursor) {
  if (cursor == nullptr || *cursor == nullptr || **cursor != ',') {
    return false;
  }
  *cursor += 1;
  return true;
}

bool AS7261Component::parse_unsigned_u16_(const char *begin, const char *end, uint16_t *value) {
  if (begin == nullptr || end == nullptr || value == nullptr || begin >= end) {
    return false;
  }

  uint32_t parsed = 0;
  for (const char *cursor = begin; cursor < end; cursor++) {
    const int8_t digit = digit_value_(*cursor, 10);
    if (digit < 0) {
      return false;
    }
    parsed = (parsed * 10UL) + static_cast<uint8_t>(digit);
    if (parsed > 0xFFFFUL) {
      return false;
    }
  }
  *value = static_cast<uint16_t>(parsed);
  return true;
}

bool AS7261Component::parse_unsigned_digits_(const char *begin, const char *end, uint8_t base, uint16_t *value) {
  if (begin == nullptr || end == nullptr || value == nullptr || begin >= end) {
    return false;
  }

  uint16_t parsed = 0;
  for (const char *cursor = begin; cursor < end; cursor++) {
    const int8_t digit = digit_value_(*cursor, base);
    if (digit < 0) {
      return false;
    }
    parsed = static_cast<uint16_t>((parsed * base) + static_cast<uint8_t>(digit));
    if (parsed > 0xFF) {
      return false;
    }
  }
  *value = parsed;
  return true;
}

bool AS7261Component::parse_calibrated_xyz_(const char *text, CalibratedFrame *frame) {
  if (text == nullptr || frame == nullptr) {
    return false;
  }

  CalibratedFrame parsed{};
  const char *cursor = text;
  if (!parse_calibrated_xyz_field_(&cursor, &parsed.x, true) ||
      !parse_calibrated_xyz_field_(&cursor, &parsed.y, true) ||
      !parse_calibrated_xyz_field_(&cursor, &parsed.z, false)) {
    return false;
  }

  cursor = trim_left_(cursor);
  if (*cursor != '\0' && *cursor != '\n') {
    return false;
  }

  frame->x = parsed.x;
  frame->y = parsed.y;
  frame->z = parsed.z;
  return true;
}

bool AS7261Component::parse_calibrated_xyz_field_(const char **cursor, float *value, bool expect_separator) {
  if (cursor == nullptr || *cursor == nullptr || value == nullptr) {
    return false;
  }

  const char *const begin = trim_left_(*cursor);
  const char *end = begin;
  while (*end != '\0' && *end != '\n' && *end != ',') {
    end++;
  }
  const char *const value_end = trim_right_(begin, end);
  if (!parse_finite_float_(begin, value_end, value)) {
    return false;
  }

  *cursor = end;
  return expect_separator ? consume_calibrated_frame_separator_(cursor) : true;
}

bool AS7261Component::consume_calibrated_frame_separator_(const char **cursor) {
  if (cursor == nullptr || *cursor == nullptr || **cursor != ',') {
    return false;
  }
  *cursor += 1;
  return true;
}

bool AS7261Component::parse_calibrated_value_(const char *text, float *value) {
  if (text == nullptr || value == nullptr) {
    return false;
  }

  const char *begin = trim_left_(text);
  const char *end = begin;
  while (*end != '\0' && *end != '\n') {
    end++;
  }
  end = trim_right_(begin, end);
  return parse_finite_float_(begin, end, value);
}

bool AS7261Component::parse_finite_float_(const char *begin, const char *end, float *value) {
  if (begin == nullptr || end == nullptr || value == nullptr || begin >= end) {
    return false;
  }

  const char *cursor = begin;
  bool negative = false;
  if (*cursor == '+' || *cursor == '-') {
    negative = *cursor == '-';
    cursor++;
  }

  float parsed = 0.0f;
  bool saw_digit = false;
  while (cursor < end && *cursor >= '0' && *cursor <= '9') {
    saw_digit = true;
    parsed = (parsed * 10.0f) + static_cast<float>(*cursor - '0');
    if (!std::isfinite(parsed)) {
      return false;
    }
    cursor++;
  }

  if (cursor < end && *cursor == '.') {
    cursor++;
    float scale = 0.1f;
    while (cursor < end && *cursor >= '0' && *cursor <= '9') {
      saw_digit = true;
      parsed += static_cast<float>(*cursor - '0') * scale;
      if (!std::isfinite(parsed)) {
        return false;
      }
      scale *= 0.1f;
      cursor++;
    }
  }

  if (!saw_digit) {
    return false;
  }

  int16_t exponent = 0;
  bool exponent_negative = false;
  if (cursor < end && (*cursor == 'e' || *cursor == 'E')) {
    cursor++;
    if (cursor < end && (*cursor == '+' || *cursor == '-')) {
      exponent_negative = *cursor == '-';
      cursor++;
    }
    if (cursor >= end || *cursor < '0' || *cursor > '9') {
      return false;
    }
    while (cursor < end && *cursor >= '0' && *cursor <= '9') {
      if (exponent < 64) {
        exponent = static_cast<int16_t>((exponent * 10) + (*cursor - '0'));
        if (exponent > 64) {
          exponent = 64;
        }
      }
      cursor++;
    }
  }

  if (cursor != end) {
    return false;
  }

  for (int16_t i = 0; i < exponent; i++) {
    parsed *= exponent_negative ? 0.1f : 10.0f;
    if (!std::isfinite(parsed)) {
      return false;
    }
  }

  *value = negative ? -parsed : parsed;
  return std::isfinite(*value);
}

const char *AS7261Component::trim_left_(const char *text) {
  if (text == nullptr) {
    return nullptr;
  }
  while (*text != '\0' && is_space_(*text)) {
    text++;
  }
  return text;
}

const char *AS7261Component::trim_right_(const char *begin, const char *end) {
  if (begin == nullptr || end == nullptr) {
    return begin;
  }
  while (end > begin && is_space_(*(end - 1))) {
    end--;
  }
  return end;
}

bool AS7261Component::is_space_(char value) { return value == ' ' || value == '\t' || value == '\r' || value == '\n'; }

int8_t AS7261Component::digit_value_(char value, uint8_t base) {
  uint8_t digit = 0;
  if (value >= '0' && value <= '9') {
    digit = static_cast<uint8_t>(value - '0');
  } else if (value >= 'a' && value <= 'f') {
    digit = static_cast<uint8_t>(value - 'a' + 10);
  } else if (value >= 'A' && value <= 'F') {
    digit = static_cast<uint8_t>(value - 'A' + 10);
  } else {
    return -1;
  }
  return digit < base ? static_cast<int8_t>(digit) : -1;
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

const char *AS7261Component::diagnostic_state_to_string_(DiagnosticState state) {
  switch (state) {
    case DiagnosticState::IDLE:
      return "idle";
    case DiagnosticState::FIRMWARE_VERSION:
      return "firmware_version";
    case DiagnosticState::DEVICE_TEMPERATURE:
      return "device_temperature";
    default:
      return "unknown";
  }
}

const char *AS7261Component::sequence_status_to_string_(SequenceStatus status) {
  switch (status) {
    case SequenceStatus::IDLE:
      return "idle";
    case SequenceStatus::RUNNING:
      return "running";
    case SequenceStatus::COMPLETED:
      return "completed";
    case SequenceStatus::FAILED:
      return "failed";
    case SequenceStatus::TIMEOUT:
      return "timeout";
    case SequenceStatus::OVERFLOW:
      return "overflow";
    default:
      return "unknown";
  }
}

const char *AS7261Component::frame_state_to_string_(FrameState state) {
  switch (state) {
    case FrameState::IDLE:
      return "idle";
    case FrameState::TRIGGER_RUNNING:
      return "trigger_running";
    case FrameState::WAITING_INT:
      return "waiting_int";
    case FrameState::READY:
      return "ready";
    case FrameState::READOUT_RUNNING:
      return "readout_running";
    case FrameState::TIMEOUT:
      return "timeout";
    case FrameState::ERROR:
      return "error";
    default:
      return "unknown";
  }
}

const char *AS7261Component::raw_frame_status_to_string_(RawFrameStatus status) {
  switch (status) {
    case RawFrameStatus::INVALID:
      return "invalid";
    case RawFrameStatus::RUNNING:
      return "running";
    case RawFrameStatus::VALID:
      return "valid";
    case RawFrameStatus::FAILED:
      return "failed";
    case RawFrameStatus::TIMEOUT:
      return "timeout";
    case RawFrameStatus::OVERFLOW:
      return "overflow";
    case RawFrameStatus::MALFORMED:
      return "malformed";
    default:
      return "unknown";
  }
}

const char *AS7261Component::calibrated_frame_status_to_string_(CalibratedFrameStatus status) {
  switch (status) {
    case CalibratedFrameStatus::INVALID:
      return "invalid";
    case CalibratedFrameStatus::RUNNING:
      return "running";
    case CalibratedFrameStatus::VALID:
      return "valid";
    case CalibratedFrameStatus::FAILED:
      return "failed";
    case CalibratedFrameStatus::TIMEOUT:
      return "timeout";
    case CalibratedFrameStatus::OVERFLOW:
      return "overflow";
    case CalibratedFrameStatus::MALFORMED:
      return "malformed";
    default:
      return "unknown";
  }
}

const char *AS7261Component::calculated_duv_status_to_string_(CalculatedDuvStatus status) {
  switch (status) {
    case CalculatedDuvStatus::INVALID:
      return "invalid";
    case CalculatedDuvStatus::VALID:
      return "valid";
    case CalculatedDuvStatus::FAILED:
      return "failed";
    case CalculatedDuvStatus::TIMEOUT:
      return "timeout";
    case CalculatedDuvStatus::OVERFLOW:
      return "overflow";
    case CalculatedDuvStatus::MALFORMED:
      return "malformed";
    default:
      return "unknown";
  }
}

const char *AS7261Component::derived_color_status_to_string_(DerivedColorStatus status) {
  switch (status) {
    case DerivedColorStatus::INVALID:
      return "invalid";
    case DerivedColorStatus::VALID:
      return "valid";
    case DerivedColorStatus::FAILED:
      return "failed";
    case DerivedColorStatus::TIMEOUT:
      return "timeout";
    case DerivedColorStatus::OVERFLOW:
      return "overflow";
    case DerivedColorStatus::MALFORMED:
      return "malformed";
    default:
      return "unknown";
  }
}

uint8_t AS7261Component::gain_to_at_value_(AS7261Gain gain) {
  switch (gain) {
    case AS7261_GAIN_1X:
      return 0;
    case AS7261_GAIN_3_7X:
      return 1;
    case AS7261_GAIN_16X:
      return 2;
    case AS7261_GAIN_64X:
      return 3;
    default:
      return 2;
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
