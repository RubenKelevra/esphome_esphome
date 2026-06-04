#include "as7261.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome::as7261 {

static const char *const TAG = "as7261";

static void temp_log_atxyzc_hex_bytes_(const char *label, const uint8_t *bytes, size_t length, bool overflow) {
  static constexpr size_t BYTES_PER_LINE = 16;
  if (bytes == nullptr || length == 0) {
    ESP_LOGW(TAG, "TEMP AS7261 ATXYZC %s raw bytes%s: <empty>", label, overflow ? " (truncated)" : "");
    return;
  }

  for (size_t offset = 0; offset < length; offset += BYTES_PER_LINE) {
    const size_t chunk_length = std::min(BYTES_PER_LINE, length - offset);
    char hex[BYTES_PER_LINE * 3]{};
    size_t hex_offset = 0;
    for (size_t i = 0; i < chunk_length; i++) {
      const int written = std::snprintf(hex + hex_offset, sizeof(hex) - hex_offset, "%s%02X", i == 0 ? "" : " ",
                                        static_cast<unsigned>(bytes[offset + i]));
      if (written <= 0) {
        break;
      }
      hex_offset += static_cast<size_t>(written);
      if (hex_offset >= sizeof(hex)) {
        hex[sizeof(hex) - 1] = '\0';
        break;
      }
    }
    ESP_LOGW(TAG, "TEMP AS7261 ATXYZC %s raw bytes %u..%u of %u%s: %s", label, static_cast<unsigned>(offset),
             static_cast<unsigned>(offset + chunk_length), static_cast<unsigned>(length),
             overflow ? " (truncated)" : "", hex);
  }
}

void AS7261Component::setup() {
  if (this->int_pin_ != nullptr) {
    this->int_pin_->setup();
  }
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->release_reset_pulse_();
  }
  if (this->manual_exposure_) {
    this->clear_auto_exposure_policy_();
  } else {
    this->initialize_auto_exposure_policy_();
  }
}

void AS7261Component::set_oklab_reference_illuminance(float illuminance_lx) {
  if (std::isfinite(illuminance_lx) && illuminance_lx > 0.0f) {
    this->oklab_reference_illuminance_ = illuminance_lx;
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
  ESP_LOGCONFIG(TAG, "  OKLab reference illuminance: %.1f lx", this->oklab_reference_illuminance_);
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
  LOG_SENSOR("  ", "Completed measurement count", this->completed_measurement_count_sensor_);
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
  this->poll_single_bank_probe_();
}

void AS7261Component::update() {
  if (this->component_busy_()) {
    return;
  }
  if (!this->start_measurement_cycle_()) {
    ESP_LOGW(TAG, "Unable to start AS7261 polling measurement");
    this->publish_nan_default_measurement_outputs_();
  }
}

void AS7261Component::request_manual_measurement() {
  if (this->component_busy_()) {
    ESP_LOGW(TAG, "Dropping AS7261 manual measurement request because the component is busy");
    return;
  }
  this->active_measurement_manual_ = true;
  this->active_measurement_counted_ = false;
  if (!this->start_measurement_cycle_()) {
    ESP_LOGW(TAG, "Unable to start AS7261 manual measurement");
    this->publish_nan_default_measurement_outputs_();
  }
}

bool AS7261Component::start_measurement_cycle_() {
  if (this->precision_mode_ && !this->precision_collection_active_()) {
    this->reset_precision_collection_();
    this->precision_collection_status_ = PrecisionCollectionStatus::COLLECTING;
  }
  this->clear_exposure_assessment_();
  if (this->manual_exposure_) {
    this->clear_auto_exposure_policy_();
  } else {
    this->initialize_auto_exposure_policy_();
  }
  if (this->manual_exposure_state_ == ManualExposureCommandState::PENDING) {
    return this->start_manual_exposure_commands_();
  }
  if (!this->manual_exposure_) {
    return this->start_auto_exposure_convergence_();
  }

  return this->start_diagnostic_readout_();
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

  this->temp_atxyzc_raw_length_ = 0;
  this->temp_atxyzc_raw_overflow_ = false;
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
    uint8_t byte = 0;
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
    case SequenceOwner::AUTO_EXPOSURE:
      this->handle_finished_auto_exposure_candidate_commands_(status);
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
    case SequenceOwner::SINGLE_BANK_PROBE_CONFIGURE:
      this->handle_finished_single_bank_probe_configure_(status);
      break;
    case SequenceOwner::SINGLE_BANK_PROBE_STOP:
      this->handle_finished_single_bank_probe_stop_(status);
      break;
    case SequenceOwner::SINGLE_BANK_PROBE_RAW_READOUT:
      this->handle_finished_single_bank_probe_raw_readout_(status);
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
    if (!this->start_diagnostic_readout_()) {
      ESP_LOGW(TAG, "Unable to resume AS7261 measurement after manual exposure");
      this->publish_nan_default_measurement_outputs_();
    }
    return;
  }
  this->manual_exposure_state_ = ManualExposureCommandState::FAILED;
  this->publish_nan_default_measurement_outputs_();
  ESP_LOGW(TAG, "AS7261 manual exposure commands failed with %s", this->sequence_status_to_string_(status));
}

bool AS7261Component::start_auto_exposure_candidate_commands_() {
  if (this->manual_exposure_ || this->transport_busy_() || this->sequence_active_() ||
      this->single_bank_probe_active_() || this->frame_state_ != FrameState::IDLE) {
    return false;
  }

  this->initialize_auto_exposure_policy_();
  AutoExposurePolicy &policy = this->auto_exposure_policy_;
  if (!policy.candidate_initialized) {
    this->auto_exposure_state_ = AutoExposureCommandState::FAILED;
    return false;
  }

  policy.current_candidate = normalize_auto_exposure_candidate_(policy.current_candidate);
  if (this->auto_exposure_candidate_applied_()) {
    this->auto_exposure_state_ = AutoExposureCommandState::APPLIED;
    if (this->auto_exposure_convergence_state_ != AutoExposureConvergenceState::IDLE) {
      return this->start_auto_exposure_probe_attempt_();
    }
    return this->start_diagnostic_readout_();
  }

  const int gain_command_length =
      std::snprintf(this->auto_exposure_commands_[0], COMMAND_BUFFER_LENGTH, "ATGAIN=%u",
                    static_cast<unsigned>(gain_to_at_value_(policy.current_candidate.gain)));
  const int integration_time_command_length =
      std::snprintf(this->auto_exposure_commands_[1], COMMAND_BUFFER_LENGTH, "ATINTTIME=%u",
                    static_cast<unsigned>(policy.current_candidate.integration_time));
  if (gain_command_length <= 0 || gain_command_length >= static_cast<int>(COMMAND_BUFFER_LENGTH) ||
      integration_time_command_length <= 0 ||
      integration_time_command_length >= static_cast<int>(COMMAND_BUFFER_LENGTH)) {
    this->auto_exposure_state_ = AutoExposureCommandState::FAILED;
    policy.candidate_applied = false;
    return false;
  }

  const CommandSequenceStep steps[] = {
      {this->auto_exposure_commands_[0], DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {this->auto_exposure_commands_[1], DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->auto_exposure_state_ = AutoExposureCommandState::RUNNING;
  this->sequence_owner_ = SequenceOwner::AUTO_EXPOSURE;
  if (this->start_command_sequence_(steps, 2)) {
    return true;
  }

  this->auto_exposure_state_ = AutoExposureCommandState::FAILED;
  this->sequence_owner_ = SequenceOwner::NONE;
  policy.candidate_applied = false;
  return false;
}

void AS7261Component::handle_finished_auto_exposure_candidate_commands_(SequenceStatus status) {
  AutoExposurePolicy &policy = this->auto_exposure_policy_;
  if (status == SequenceStatus::COMPLETED) {
    policy.current_candidate = normalize_auto_exposure_candidate_(policy.current_candidate);
    policy.applied_candidate = policy.current_candidate;
    policy.candidate_applied = true;
    this->auto_exposure_state_ = AutoExposureCommandState::APPLIED;
    ESP_LOGD(TAG, "AS7261 auto exposure candidate applied: gain %u int %u",
             static_cast<unsigned>(gain_to_at_value_(policy.applied_candidate.gain)),
             static_cast<unsigned>(policy.applied_candidate.integration_time));
    if (this->auto_exposure_convergence_state_ != AutoExposureConvergenceState::IDLE) {
      this->start_auto_exposure_probe_attempt_();
      return;
    }
    if (!this->start_diagnostic_readout_()) {
      ESP_LOGW(TAG, "Unable to resume AS7261 measurement after auto exposure candidate application");
      this->publish_nan_default_measurement_outputs_();
    }
    return;
  }

  this->auto_exposure_state_ = AutoExposureCommandState::FAILED;
  policy.candidate_applied = false;
  if (this->auto_exposure_convergence_state_ != AutoExposureConvergenceState::IDLE) {
    this->fail_auto_exposure_convergence_();
  } else {
    this->publish_nan_default_measurement_outputs_();
  }
  ESP_LOGW(TAG, "AS7261 auto exposure candidate commands failed with %s", this->sequence_status_to_string_(status));
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
    this->publish_nan_default_measurement_outputs_();
    return;
  }
  if (this->manual_exposure_ && this->manual_exposure_state_ != ManualExposureCommandState::APPLIED) {
    ESP_LOGW(TAG, "Skipping AS7261 one-shot trigger because manual exposure is not applied");
    this->publish_nan_default_measurement_outputs_();
    return;
  }
  if (!this->device_temperature_valid_ || this->device_temperature_invalid_) {
    ESP_LOGW(TAG, "Skipping AS7261 one-shot trigger because device temperature is invalid");
    this->publish_nan_default_measurement_outputs_();
    return;
  }
  if (this->device_temperature_unsafe_) {
    ESP_LOGW(TAG, "Skipping AS7261 one-shot trigger while device temperature is unsafe");
    this->publish_nan_default_measurement_outputs_();
    return;
  }
  if (!this->start_one_shot_frame_trigger_()) {
    ESP_LOGW(TAG, "Unable to start AS7261 one-shot frame trigger");
    this->frame_state_ = FrameState::ERROR;
    this->publish_nan_default_measurement_outputs_();
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
      {"ATTCSMD=2", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
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
    this->publish_nan_default_measurement_outputs_();
    ESP_LOGW(TAG, "AS7261 one-shot trigger failed with %s", this->sequence_status_to_string_(status));
    return;
  }
  this->frame_readout_attempt_ = 0;
  this->schedule_frame_timed_readout_(this->calculate_frame_timed_readout_wait_ms_());
  ESP_LOGD(TAG, "AS7261 final Mode 2 conversion started; waiting %u ms before timed UART calibrated readout",
           static_cast<unsigned>(this->frame_timed_readout_wait_ms_));
}

void AS7261Component::poll_frame_trigger_() {
  if (this->frame_state_ == FrameState::READY) {
    if (!this->start_calibrated_frame_readout_()) {
      this->raw_frame_ = RawFrame{};
      this->raw_frame_status_ = RawFrameStatus::INVALID;
      this->clear_exposure_assessment_();
      this->calibrated_frame_ = CalibratedFrame{};
      this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
      this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
      this->clear_derived_color_(DerivedColorStatus::FAILED);
      this->publish_nan_default_measurement_outputs_();
      this->clear_terminal_frame_state_();
      ESP_LOGW(TAG, "Unable to start AS7261 calibrated frame readout");
    }
    return;
  }

  if (this->frame_state_ == FrameState::TIMEOUT) {
    this->raw_frame_ = RawFrame{};
    this->raw_frame_status_ = RawFrameStatus::TIMEOUT;
    this->clear_exposure_assessment_();
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::TIMEOUT;
    this->clear_calculated_duv_(CalculatedDuvStatus::TIMEOUT);
    this->clear_derived_color_(DerivedColorStatus::TIMEOUT);
    this->publish_nan_default_measurement_outputs_();
    this->clear_terminal_frame_state_();
    return;
  }

  if (this->frame_state_ == FrameState::ERROR) {
    this->raw_frame_ = RawFrame{};
    this->raw_frame_status_ = RawFrameStatus::FAILED;
    this->clear_exposure_assessment_();
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
    this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
    this->clear_derived_color_(DerivedColorStatus::FAILED);
    this->publish_nan_default_measurement_outputs_();
    this->clear_terminal_frame_state_();
    return;
  }

  if (this->frame_state_ != FrameState::WAITING_TIMED_READOUT) {
    return;
  }
  if (millis() - this->frame_wait_started_millis_ >= this->frame_timed_readout_wait_ms_) {
    this->frame_state_ = FrameState::READY;
    ESP_LOGD(TAG, "AS7261 timed UART readout window elapsed after %u ms; starting calibrated readout",
             static_cast<unsigned>(this->frame_timed_readout_wait_ms_));
  }
}

bool AS7261Component::start_single_bank_probe_() {
  if (this->manual_exposure_ || this->transport_busy_() || this->sequence_active_() ||
      this->frame_state_ != FrameState::IDLE || this->single_bank_probe_state_ != SingleBankProbeState::IDLE) {
    return false;
  }
  this->initialize_auto_exposure_policy_();
  const int mode_command_length = std::snprintf(this->single_bank_probe_mode_command_, COMMAND_BUFFER_LENGTH,
                                                "ATTCSMD=%u", static_cast<unsigned>(SINGLE_BANK_PROBE_SENSOR_MODE));
  const int interval_command_length =
      std::snprintf(this->single_bank_probe_interval_command_, COMMAND_BUFFER_LENGTH, "ATINTRVL=%u",
                    static_cast<unsigned>(this->calculate_single_bank_probe_interval_()));
  if (mode_command_length <= 0 || mode_command_length >= static_cast<int>(COMMAND_BUFFER_LENGTH) ||
      interval_command_length <= 0 || interval_command_length >= static_cast<int>(COMMAND_BUFFER_LENGTH)) {
    this->clear_single_bank_probe_result_(SingleBankProbeStatus::FAILED);
    return false;
  }

  const CommandSequenceStep steps[] = {
      {this->single_bank_probe_mode_command_, DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {this->single_bank_probe_interval_command_, DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {"ATBURST=1", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->single_bank_probe_raw_frame_ = RawFrame{};
  this->single_bank_probe_status_ = SingleBankProbeStatus::RUNNING;
  this->single_bank_probe_state_ = SingleBankProbeState::CONFIGURE_RUNNING;
  this->sequence_owner_ = SequenceOwner::SINGLE_BANK_PROBE_CONFIGURE;
  if (this->start_command_sequence_(steps, 3)) {
    return true;
  }

  this->sequence_owner_ = SequenceOwner::NONE;
  this->clear_single_bank_probe_result_(SingleBankProbeStatus::FAILED);
  return false;
}

void AS7261Component::handle_finished_single_bank_probe_configure_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    this->clear_single_bank_probe_result_(status == SequenceStatus::TIMEOUT    ? SingleBankProbeStatus::TIMEOUT
                                          : status == SequenceStatus::OVERFLOW ? SingleBankProbeStatus::OVERFLOW
                                                                               : SingleBankProbeStatus::FAILED);
    ESP_LOGW(TAG, "AS7261 single-bank probe configuration failed with %s", this->sequence_status_to_string_(status));
    if (this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING) {
      this->fail_auto_exposure_convergence_();
    }
    return;
  }

  this->single_bank_probe_wait_started_millis_ = millis();
  this->single_bank_probe_timed_readout_wait_ms_ = this->calculate_single_bank_probe_timed_readout_wait_ms_();
  this->single_bank_probe_state_ = SingleBankProbeState::WAITING_TIMED_READOUT;
  this->single_bank_probe_status_ = SingleBankProbeStatus::RUNNING;
  ESP_LOGD(TAG, "AS7261 single-bank probe burst started; waiting %u ms before timed UART stop/readout",
           static_cast<unsigned>(this->single_bank_probe_timed_readout_wait_ms_));
}

void AS7261Component::poll_single_bank_probe_() {
  if (this->single_bank_probe_state_ != SingleBankProbeState::WAITING_TIMED_READOUT) {
    return;
  }

  if (millis() - this->single_bank_probe_wait_started_millis_ < this->single_bank_probe_timed_readout_wait_ms_) {
    return;
  }

  if (!this->start_single_bank_probe_stop_()) {
    this->clear_single_bank_probe_result_(SingleBankProbeStatus::FAILED);
    ESP_LOGW(TAG, "Unable to stop AS7261 single-bank probe burst after timed wait");
    if (this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING) {
      this->fail_auto_exposure_convergence_();
    }
  }
}

bool AS7261Component::start_single_bank_probe_stop_() {
  if (this->transport_busy_() || this->sequence_active_() ||
      this->single_bank_probe_state_ != SingleBankProbeState::WAITING_TIMED_READOUT) {
    return false;
  }

  const CommandSequenceStep steps[] = {
      {"ATBURST=0", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  if (this->single_bank_probe_status_ != SingleBankProbeStatus::TIMEOUT) {
    this->single_bank_probe_status_ = SingleBankProbeStatus::RUNNING;
  }
  this->single_bank_probe_state_ = SingleBankProbeState::STOP_RUNNING;
  this->sequence_owner_ = SequenceOwner::SINGLE_BANK_PROBE_STOP;
  if (this->start_command_sequence_(steps, 1)) {
    return true;
  }

  this->sequence_owner_ = SequenceOwner::NONE;
  this->single_bank_probe_state_ = SingleBankProbeState::IDLE;
  return false;
}

void AS7261Component::handle_finished_single_bank_probe_stop_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    this->clear_single_bank_probe_result_(status == SequenceStatus::TIMEOUT    ? SingleBankProbeStatus::TIMEOUT
                                          : status == SequenceStatus::OVERFLOW ? SingleBankProbeStatus::OVERFLOW
                                                                               : SingleBankProbeStatus::FAILED);
    ESP_LOGW(TAG, "AS7261 single-bank probe burst stop failed with %s", this->sequence_status_to_string_(status));
    if (this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING) {
      this->fail_auto_exposure_convergence_();
    }
    return;
  }

  if (!this->start_single_bank_probe_raw_readout_()) {
    this->clear_single_bank_probe_result_(SingleBankProbeStatus::FAILED);
    ESP_LOGW(TAG, "Unable to start AS7261 single-bank probe raw readout after burst stop");
    if (this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING) {
      this->fail_auto_exposure_convergence_();
    }
  }
}

bool AS7261Component::start_single_bank_probe_raw_readout_() {
  if (this->transport_busy_() || this->sequence_active_() ||
      this->single_bank_probe_state_ != SingleBankProbeState::STOP_RUNNING) {
    return false;
  }

  const CommandSequenceStep steps[] = {
      {"ATDATA", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->single_bank_probe_raw_frame_ = RawFrame{};
  this->single_bank_probe_status_ = SingleBankProbeStatus::RUNNING;
  this->single_bank_probe_state_ = SingleBankProbeState::RAW_READOUT_RUNNING;
  this->sequence_owner_ = SequenceOwner::SINGLE_BANK_PROBE_RAW_READOUT;
  if (this->start_command_sequence_(steps, 1)) {
    return true;
  }

  this->sequence_owner_ = SequenceOwner::NONE;
  this->clear_single_bank_probe_result_(SingleBankProbeStatus::FAILED);
  return false;
}

void AS7261Component::handle_finished_single_bank_probe_raw_readout_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    this->clear_single_bank_probe_result_(status == SequenceStatus::TIMEOUT    ? SingleBankProbeStatus::TIMEOUT
                                          : status == SequenceStatus::OVERFLOW ? SingleBankProbeStatus::OVERFLOW
                                                                               : SingleBankProbeStatus::FAILED);
    ESP_LOGW(TAG, "AS7261 single-bank probe raw readout failed with %s", this->sequence_status_to_string_(status));
    if (this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING) {
      this->fail_auto_exposure_convergence_();
    }
    return;
  }

  RawFrame frame{};
  if (!parse_raw_frame_(this->first_response_value_(), &frame)) {
    this->clear_single_bank_probe_result_(SingleBankProbeStatus::MALFORMED);
    ESP_LOGW(TAG, "Unable to parse AS7261 single-bank probe raw response: %s", this->response_buffer_);
    if (this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING) {
      this->fail_auto_exposure_convergence_();
    }
    return;
  }
  const bool frame_empty = raw_frame_empty_(frame);
  const bool auto_exposure_probe_active =
      this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING;
  if (frame_empty && !auto_exposure_probe_active) {
    this->clear_single_bank_probe_result_(SingleBankProbeStatus::MALFORMED);
    ESP_LOGW(TAG, "Rejecting empty all-zero AS7261 single-bank probe raw frame");
    return;
  } else if (frame_empty) {
    ESP_LOGD(TAG, "Treating empty all-zero AS7261 single-bank probe raw frame as too-dark auto-exposure evidence");
  }
  this->single_bank_probe_raw_frame_ = frame;
  this->single_bank_probe_status_ = SingleBankProbeStatus::VALID;
  this->single_bank_probe_state_ = SingleBankProbeState::IDLE;
  ESP_LOGD(TAG, "AS7261 single-bank probe raw frame stored: X=%u Y=%u Z=%u NIR=%u Dark=%u Clear=%u",
           static_cast<unsigned>(this->single_bank_probe_raw_frame_.x),
           static_cast<unsigned>(this->single_bank_probe_raw_frame_.y),
           static_cast<unsigned>(this->single_bank_probe_raw_frame_.z),
           static_cast<unsigned>(this->single_bank_probe_raw_frame_.near_ir),
           static_cast<unsigned>(this->single_bank_probe_raw_frame_.dark),
           static_cast<unsigned>(this->single_bank_probe_raw_frame_.clear));
  if (this->auto_exposure_convergence_state_ == AutoExposureConvergenceState::PROBING) {
    this->handle_finished_auto_exposure_probe_();
  }
}

void AS7261Component::clear_single_bank_probe_result_(SingleBankProbeStatus status) {
  this->single_bank_probe_raw_frame_ = RawFrame{};
  this->single_bank_probe_status_ = status;
  this->single_bank_probe_state_ = SingleBankProbeState::IDLE;
}

bool AS7261Component::start_raw_frame_readout_() {
  if (this->transport_busy_() || this->sequence_active_() || this->frame_state_ != FrameState::READY) {
    return false;
  }

  const CommandSequenceStep steps[] = {
      {"ATDATA", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  this->raw_frame_ = RawFrame{};
  this->clear_exposure_assessment_();
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
  this->clear_exposure_assessment_();
  this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
  this->clear_derived_color_(DerivedColorStatus::FAILED);
  this->clear_terminal_frame_state_();
  return false;
}

void AS7261Component::handle_finished_raw_frame_readout_(SequenceStatus status) {
  if (status != SequenceStatus::COMPLETED) {
    this->fail_timed_frame_readout_("raw frame command failed",
                                    status == SequenceStatus::TIMEOUT    ? RawFrameStatus::TIMEOUT
                                    : status == SequenceStatus::OVERFLOW ? RawFrameStatus::OVERFLOW
                                                                         : RawFrameStatus::FAILED,
                                    status == SequenceStatus::TIMEOUT    ? CalibratedFrameStatus::TIMEOUT
                                    : status == SequenceStatus::OVERFLOW ? CalibratedFrameStatus::OVERFLOW
                                                                         : CalibratedFrameStatus::FAILED,
                                    status == SequenceStatus::TIMEOUT    ? CalculatedDuvStatus::TIMEOUT
                                    : status == SequenceStatus::OVERFLOW ? CalculatedDuvStatus::OVERFLOW
                                                                         : CalculatedDuvStatus::FAILED,
                                    status == SequenceStatus::TIMEOUT    ? DerivedColorStatus::TIMEOUT
                                    : status == SequenceStatus::OVERFLOW ? DerivedColorStatus::OVERFLOW
                                                                         : DerivedColorStatus::FAILED);
    ESP_LOGW(TAG, "AS7261 raw frame readout failed with %s", this->sequence_status_to_string_(status));
    return;
  }

  RawFrame frame{};
  if (!parse_raw_frame_(this->first_response_value_(), &frame)) {
    this->fail_timed_frame_readout_("malformed raw frame response", RawFrameStatus::MALFORMED,
                                    CalibratedFrameStatus::MALFORMED, CalculatedDuvStatus::MALFORMED,
                                    DerivedColorStatus::MALFORMED);
    ESP_LOGW(TAG, "Unable to parse AS7261 raw frame response: %s", this->response_buffer_);
    return;
  }
  if (raw_frame_empty_(frame)) {
    this->fail_timed_frame_readout_("empty all-zero raw frame", RawFrameStatus::MALFORMED,
                                    CalibratedFrameStatus::MALFORMED, CalculatedDuvStatus::MALFORMED,
                                    DerivedColorStatus::MALFORMED);
    ESP_LOGW(TAG, "Rejecting empty all-zero AS7261 raw frame");
    return;
  }

  this->raw_frame_ = frame;
  this->raw_frame_status_ = RawFrameStatus::VALID;
  this->clear_terminal_frame_state_();
  ESP_LOGD(TAG, "AS7261 raw frame stored: X=%u Y=%u Z=%u NIR=%u Dark=%u Clear=%u",
           static_cast<unsigned>(this->raw_frame_.x), static_cast<unsigned>(this->raw_frame_.y),
           static_cast<unsigned>(this->raw_frame_.z), static_cast<unsigned>(this->raw_frame_.near_ir),
           static_cast<unsigned>(this->raw_frame_.dark), static_cast<unsigned>(this->raw_frame_.clear));
  if (!this->assess_clear_channel_exposure_()) {
    ESP_LOGW(TAG, "Unable to assess AS7261 clear-channel exposure");
  }
  this->update_auto_exposure_policy_();

  if (!this->start_calibrated_frame_readout_()) {
    this->fail_timed_frame_readout_("unable to start calibrated frame readout", RawFrameStatus::VALID,
                                    CalibratedFrameStatus::FAILED, CalculatedDuvStatus::FAILED,
                                    DerivedColorStatus::FAILED);
    ESP_LOGW(TAG, "Unable to start AS7261 calibrated frame readout");
  }
}

void AS7261Component::clear_exposure_assessment_() { this->exposure_assessment_ = ExposureAssessment{}; }

bool AS7261Component::assess_clear_channel_exposure_() {
  return this->assess_clear_channel_exposure_(this->raw_frame_, this->raw_frame_status_ == RawFrameStatus::VALID);
}

bool AS7261Component::assess_clear_channel_exposure_(const RawFrame &frame, bool frame_valid) {
  if (!frame_valid) {
    this->clear_exposure_assessment_();
    return false;
  }

  ExposureAssessment assessment{};
  assessment.clear_percent = (CLEAR_PERCENT_SCALE * static_cast<float>(frame.clear)) / RAW_CLEAR_FULL_SCALE;
  if (!std::isfinite(assessment.clear_percent)) {
    this->clear_exposure_assessment_();
    return false;
  }

  if (assessment.clear_percent > CLEAR_OVEREXPOSED_PERCENT) {
    assessment.status = ExposureAssessmentStatus::CLEAR_OVEREXPOSED;
    assessment.guidance = ExposureAssessmentGuidance::RECOVER_OVEREXPOSED;
  } else if (assessment.clear_percent >= CLEAR_NEAR_SATURATION_PERCENT) {
    assessment.status = ExposureAssessmentStatus::CLEAR_NEAR_SATURATION;
    assessment.guidance = ExposureAssessmentGuidance::DECREASE;
  } else if (assessment.clear_percent >= CLEAR_TARGET_LOW_PERCENT) {
    assessment.status = ExposureAssessmentStatus::CLEAR_TARGET;
    assessment.guidance = ExposureAssessmentGuidance::ACCEPT;
  } else if (assessment.clear_percent >= CLEAR_TRUSTED_LOW_PERCENT) {
    assessment.status = ExposureAssessmentStatus::CLEAR_TOO_DARK;
    assessment.guidance = ExposureAssessmentGuidance::INCREASE;
  } else {
    assessment.status = ExposureAssessmentStatus::CLEAR_TOO_DARK_JUMP;
    assessment.guidance = ExposureAssessmentGuidance::JUMP_INCREASE;
  }

  this->exposure_assessment_ = assessment;
  ESP_LOGD(TAG, "AS7261 clear exposure assessed: %.2f%%, status %s, guidance %s",
           this->exposure_assessment_.clear_percent,
           exposure_assessment_status_to_string_(this->exposure_assessment_.status),
           exposure_assessment_guidance_to_string_(this->exposure_assessment_.guidance));
  this->learn_dark_channel_recovery_(frame, this->exposure_assessment_);
  return true;
}

void AS7261Component::learn_dark_channel_recovery_(const RawFrame &frame, const ExposureAssessment &assessment) {
  if (this->manual_exposure_ || assessment.status != ExposureAssessmentStatus::CLEAR_TARGET ||
      !std::isfinite(assessment.clear_percent) || assessment.clear_percent <= 0.0f || frame.clear == 0 ||
      frame.dark == 0) {
    return;
  }

  const float dark_percent = (CLEAR_PERCENT_SCALE * static_cast<float>(frame.dark)) / RAW_CLEAR_FULL_SCALE;
  if (!std::isfinite(dark_percent) || dark_percent <= 0.0f || dark_percent >= CLEAR_OVEREXPOSED_PERCENT) {
    return;
  }

  const float ratio = static_cast<float>(frame.dark) / static_cast<float>(frame.clear);
  if (!std::isfinite(ratio) || ratio < DARK_CHANNEL_RECOVERY_MIN_RATIO || ratio > DARK_CHANNEL_RECOVERY_MAX_RATIO) {
    return;
  }

  this->dark_channel_recovery_state_.dark_to_clear_ratio = ratio;
  this->dark_channel_recovery_state_.learned_at_millis = millis();
  this->dark_channel_recovery_state_.learned = true;
  ESP_LOGD(TAG, "AS7261 dark-channel recovery state learned from target exposure");
}

bool AS7261Component::dark_channel_recovery_state_valid_() const {
  const DarkChannelRecoveryState &state = this->dark_channel_recovery_state_;
  if (!state.learned || !std::isfinite(state.dark_to_clear_ratio) ||
      state.dark_to_clear_ratio < DARK_CHANNEL_RECOVERY_MIN_RATIO ||
      state.dark_to_clear_ratio > DARK_CHANNEL_RECOVERY_MAX_RATIO) {
    return false;
  }

  return millis() - state.learned_at_millis <= DARK_CHANNEL_RECOVERY_STATE_MAX_AGE_MS;
}

void AS7261Component::clear_auto_exposure_policy_() {
  this->auto_exposure_policy_ = AutoExposurePolicy{};
  this->auto_exposure_state_ = AutoExposureCommandState::NOT_CONFIGURED;
  this->auto_exposure_convergence_state_ = AutoExposureConvergenceState::IDLE;
  this->auto_exposure_convergence_attempts_ = 0;
}

bool AS7261Component::start_auto_exposure_convergence_() {
  if (this->manual_exposure_ || this->transport_busy_() || this->sequence_active_() ||
      this->single_bank_probe_active_() || this->frame_state_ != FrameState::IDLE) {
    return false;
  }

  this->initialize_auto_exposure_policy_();
  AutoExposurePolicy &policy = this->auto_exposure_policy_;
  if (!policy.candidate_initialized) {
    this->fail_auto_exposure_convergence_();
    return false;
  }

  this->clear_exposure_assessment_();
  this->auto_exposure_convergence_attempts_ = 0;
  this->auto_exposure_convergence_state_ = AutoExposureConvergenceState::APPLYING_CANDIDATE;
  if (this->start_auto_exposure_candidate_commands_()) {
    return true;
  }
  this->fail_auto_exposure_convergence_();
  return false;
}

bool AS7261Component::start_auto_exposure_probe_attempt_() {
  if (this->manual_exposure_ || this->auto_exposure_convergence_attempts_ >= AUTO_EXPOSURE_CONVERGENCE_ATTEMPT_LIMIT) {
    this->fail_auto_exposure_convergence_();
    return false;
  }

  this->auto_exposure_convergence_attempts_++;
  this->auto_exposure_convergence_state_ = AutoExposureConvergenceState::PROBING;
  if (this->start_single_bank_probe_()) {
    return true;
  }
  this->fail_auto_exposure_convergence_();
  return false;
}

void AS7261Component::handle_finished_auto_exposure_probe_() {
  if (this->manual_exposure_ || this->single_bank_probe_status_ != SingleBankProbeStatus::VALID ||
      !this->assess_clear_channel_exposure_(this->single_bank_probe_raw_frame_, true) ||
      !this->update_auto_exposure_policy_()) {
    this->fail_auto_exposure_convergence_();
    return;
  }

  AutoExposurePolicy &policy = this->auto_exposure_policy_;
  if (policy.accepted) {
    this->auto_exposure_convergence_state_ = AutoExposureConvergenceState::IDLE;
    this->auto_exposure_convergence_attempts_ = 0;
    if (!this->start_diagnostic_readout_()) {
      ESP_LOGW(TAG, "Unable to resume AS7261 measurement after auto exposure convergence");
      this->publish_nan_default_measurement_outputs_();
    }
    return;
  }

  const AutoExposureCandidate current_candidate = normalize_auto_exposure_candidate_(policy.current_candidate);
  const AutoExposureCandidate ordinary_next_candidate = normalize_auto_exposure_candidate_(policy.next_candidate);
  const bool ordinary_candidate_progress =
      !auto_exposure_candidates_equal_(ordinary_next_candidate, current_candidate) &&
      auto_exposure_candidate_lower_(ordinary_next_candidate, current_candidate);

  if (policy.dark_channel_recovery_required) {
    if (this->auto_exposure_convergence_attempts_ >= AUTO_EXPOSURE_CONVERGENCE_ATTEMPT_LIMIT) {
      this->fail_auto_exposure_convergence_();
      return;
    }

    ESP_LOGD(TAG, "AS7261 dark-channel recovery attempted: current gain %u int %u, ordinary fallback gain %u int %u%s",
             static_cast<unsigned>(gain_to_at_value_(current_candidate.gain)),
             static_cast<unsigned>(current_candidate.integration_time),
             static_cast<unsigned>(gain_to_at_value_(ordinary_next_candidate.gain)),
             static_cast<unsigned>(ordinary_next_candidate.integration_time),
             ordinary_candidate_progress ? "" : ", no ordinary progress");

    if (this->recover_auto_exposure_from_dark_channel_()) {
      const AutoExposureCandidate dark_recovery_candidate = normalize_auto_exposure_candidate_(policy.next_candidate);
      const bool dark_recovery_progress =
          !auto_exposure_candidates_equal_(dark_recovery_candidate, current_candidate) &&
          auto_exposure_candidate_lower_(dark_recovery_candidate, current_candidate);
      if (ordinary_candidate_progress &&
          !auto_exposure_candidate_lower_(dark_recovery_candidate, ordinary_next_candidate)) {
        policy.next_candidate = ordinary_next_candidate;
        policy.dark_channel_recovery_required = false;
        ESP_LOGD(TAG, "AS7261 dark-channel recovery candidate rejected; using ordinary lower candidate gain %u int %u",
                 static_cast<unsigned>(gain_to_at_value_(policy.next_candidate.gain)),
                 static_cast<unsigned>(policy.next_candidate.integration_time));
      } else if (!dark_recovery_progress) {
        if (!ordinary_candidate_progress) {
          this->fail_auto_exposure_convergence_();
          return;
        }
        policy.next_candidate = ordinary_next_candidate;
        policy.dark_channel_recovery_required = false;
        ESP_LOGD(TAG,
                 "AS7261 dark-channel recovery did not produce a lower candidate; using ordinary lower candidate gain "
                 "%u int %u",
                 static_cast<unsigned>(gain_to_at_value_(policy.next_candidate.gain)),
                 static_cast<unsigned>(policy.next_candidate.integration_time));
      }
    } else {
      if (!ordinary_candidate_progress) {
        this->fail_auto_exposure_convergence_();
        return;
      }
      policy.next_candidate = ordinary_next_candidate;
      policy.dark_channel_recovery_required = false;
      ESP_LOGD(TAG, "AS7261 dark-channel recovery failed; using ordinary lower candidate gain %u int %u",
               static_cast<unsigned>(gain_to_at_value_(policy.next_candidate.gain)),
               static_cast<unsigned>(policy.next_candidate.integration_time));
    }
  }

  if (auto_exposure_candidates_equal_(policy.current_candidate, policy.next_candidate) ||
      this->auto_exposure_convergence_attempts_ >= AUTO_EXPOSURE_CONVERGENCE_ATTEMPT_LIMIT) {
    this->fail_auto_exposure_convergence_();
    return;
  }

  ESP_LOGD(TAG, "AS7261 auto exposure candidate chosen for application: gain %u int %u",
           static_cast<unsigned>(gain_to_at_value_(policy.next_candidate.gain)),
           static_cast<unsigned>(policy.next_candidate.integration_time));

  policy.current_candidate = normalize_auto_exposure_candidate_(policy.next_candidate);
  policy.candidate_applied = false;
  this->auto_exposure_state_ = AutoExposureCommandState::PENDING;
  this->auto_exposure_convergence_state_ = AutoExposureConvergenceState::APPLYING_CANDIDATE;
  if (!this->start_auto_exposure_candidate_commands_()) {
    this->fail_auto_exposure_convergence_();
  }
}

void AS7261Component::fail_auto_exposure_convergence_() {
  this->auto_exposure_convergence_state_ = AutoExposureConvergenceState::FAILED;
  this->auto_exposure_state_ = AutoExposureCommandState::FAILED;
  this->auto_exposure_policy_.accepted = false;
  this->clear_exposure_assessment_();
  this->publish_nan_default_measurement_outputs_();
  this->auto_exposure_convergence_state_ = AutoExposureConvergenceState::IDLE;
  this->auto_exposure_convergence_attempts_ = 0;
}

void AS7261Component::initialize_auto_exposure_policy_() {
  if (this->manual_exposure_ || this->auto_exposure_policy_.candidate_initialized) {
    return;
  }

  const AutoExposureCandidate candidate{AUTO_EXPOSURE_FALLBACK_GAIN, AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME};
  this->auto_exposure_policy_.current_candidate = candidate;
  this->auto_exposure_policy_.next_candidate = candidate;
  this->auto_exposure_policy_.action = AutoExposurePolicyAction::PENDING_ASSESSMENT;
  this->auto_exposure_policy_.candidate_initialized = true;
  this->auto_exposure_policy_.candidate_applied = false;
  this->auto_exposure_state_ = AutoExposureCommandState::PENDING;
}

bool AS7261Component::update_auto_exposure_policy_() {
  if (this->manual_exposure_) {
    return true;
  }

  this->initialize_auto_exposure_policy_();
  AutoExposurePolicy &policy = this->auto_exposure_policy_;
  policy.current_candidate = normalize_auto_exposure_candidate_(policy.current_candidate);
  if (policy.candidate_applied &&
      !auto_exposure_candidates_equal_(policy.current_candidate, policy.applied_candidate)) {
    policy.candidate_applied = false;
    this->auto_exposure_state_ = AutoExposureCommandState::PENDING;
  }
  policy.next_candidate = policy.current_candidate;
  policy.accepted = false;
  policy.dark_channel_recovery_required = false;
  policy.clamped = false;

  const ExposureAssessment &assessment = this->exposure_assessment_;
  if (assessment.guidance == ExposureAssessmentGuidance::INVALID || !std::isfinite(assessment.clear_percent) ||
      assessment.clear_percent < 0.0f ||
      (assessment.clear_percent == 0.0f && assessment.guidance != ExposureAssessmentGuidance::JUMP_INCREASE)) {
    policy.action = AutoExposurePolicyAction::INVALID_ASSESSMENT;
    return false;
  }

  if (assessment.guidance == ExposureAssessmentGuidance::ACCEPT) {
    policy.action = AutoExposurePolicyAction::ACCEPT_CURRENT;
    policy.accepted = true;
  } else {
    float multiplier = 1.0f;
    switch (assessment.guidance) {
      case ExposureAssessmentGuidance::DECREASE:
        policy.action = AutoExposurePolicyAction::DECREASE;
        multiplier = AUTO_EXPOSURE_TARGET_CLEAR_PERCENT / assessment.clear_percent;
        break;
      case ExposureAssessmentGuidance::INCREASE:
        policy.action = AutoExposurePolicyAction::INCREASE;
        multiplier = AUTO_EXPOSURE_TARGET_CLEAR_PERCENT / assessment.clear_percent;
        break;
      case ExposureAssessmentGuidance::JUMP_INCREASE:
        policy.action = AutoExposurePolicyAction::JUMP_INCREASE;
        multiplier = AUTO_EXPOSURE_JUMP_INCREASE_MULTIPLIER;
        break;
      case ExposureAssessmentGuidance::RECOVER_OVEREXPOSED:
        policy.action = AutoExposurePolicyAction::RECOVER_OVEREXPOSED;
        policy.dark_channel_recovery_required = true;
        multiplier = AUTO_EXPOSURE_TARGET_CLEAR_PERCENT / assessment.clear_percent;
        break;
      default:
        policy.action = AutoExposurePolicyAction::INVALID_ASSESSMENT;
        return false;
    }

    const AutoExposureAdjustment adjustment =
        this->scale_auto_exposure_candidate_(policy.current_candidate, multiplier);
    policy.next_candidate = adjustment.candidate;
    policy.clamped = adjustment.clamped;
  }

  ESP_LOGD(TAG, "AS7261 auto exposure policy: action %s, current gain %u int %u, next gain %u int %u%s%s",
           auto_exposure_policy_action_to_string_(policy.action),
           static_cast<unsigned>(gain_to_at_value_(policy.current_candidate.gain)),
           static_cast<unsigned>(policy.current_candidate.integration_time),
           static_cast<unsigned>(gain_to_at_value_(policy.next_candidate.gain)),
           static_cast<unsigned>(policy.next_candidate.integration_time),
           policy.dark_channel_recovery_required ? ", dark recovery required" : "", policy.clamped ? ", clamped" : "");
  return true;
}

bool AS7261Component::recover_auto_exposure_from_dark_channel_() {
  if (this->manual_exposure_ || this->single_bank_probe_status_ != SingleBankProbeStatus::VALID ||
      !this->dark_channel_recovery_state_valid_()) {
    return false;
  }

  AutoExposurePolicy &policy = this->auto_exposure_policy_;
  if (!policy.candidate_initialized || !policy.dark_channel_recovery_required ||
      policy.action != AutoExposurePolicyAction::RECOVER_OVEREXPOSED) {
    return false;
  }

  const RawFrame &frame = this->single_bank_probe_raw_frame_;
  if (frame.dark == 0) {
    return false;
  }

  const float dark_percent = (CLEAR_PERCENT_SCALE * static_cast<float>(frame.dark)) / RAW_CLEAR_FULL_SCALE;
  if (!std::isfinite(dark_percent) || dark_percent <= 0.0f ||
      dark_percent >= DARK_CHANNEL_RECOVERY_CURRENT_DARK_MAX_PERCENT) {
    return false;
  }

  const float estimated_clear_percent = dark_percent / this->dark_channel_recovery_state_.dark_to_clear_ratio;
  if (!std::isfinite(estimated_clear_percent) || estimated_clear_percent <= AUTO_EXPOSURE_TARGET_CLEAR_PERCENT) {
    return false;
  }

  const float multiplier = AUTO_EXPOSURE_TARGET_CLEAR_PERCENT / estimated_clear_percent;
  if (!std::isfinite(multiplier) || multiplier <= 0.0f || multiplier >= 1.0f) {
    return false;
  }

  const AutoExposureCandidate baseline = normalize_auto_exposure_candidate_(policy.current_candidate);
  const AutoExposureAdjustment adjustment = this->scale_auto_exposure_candidate_(baseline, multiplier);
  if (!auto_exposure_candidate_lower_(adjustment.candidate, baseline)) {
    return false;
  }

  policy.next_candidate = normalize_auto_exposure_candidate_(adjustment.candidate);
  policy.dark_channel_recovery_required = false;
  policy.clamped = adjustment.clamped;
  ESP_LOGD(TAG, "AS7261 dark-channel recovery selected lower candidate: gain %u int %u -> gain %u int %u%s",
           static_cast<unsigned>(gain_to_at_value_(baseline.gain)), static_cast<unsigned>(baseline.integration_time),
           static_cast<unsigned>(gain_to_at_value_(policy.next_candidate.gain)),
           static_cast<unsigned>(policy.next_candidate.integration_time), policy.clamped ? ", clamped" : "");
  return true;
}

bool AS7261Component::auto_exposure_candidate_applied_() const {
  if (this->manual_exposure_ || !this->auto_exposure_policy_.candidate_initialized ||
      !this->auto_exposure_policy_.candidate_applied) {
    return false;
  }
  return auto_exposure_candidates_equal_(this->auto_exposure_policy_.current_candidate,
                                         this->auto_exposure_policy_.applied_candidate);
}

bool AS7261Component::auto_exposure_candidate_lower_(AutoExposureCandidate candidate, AutoExposureCandidate baseline) {
  candidate = normalize_auto_exposure_candidate_(candidate);
  baseline = normalize_auto_exposure_candidate_(baseline);
  const float candidate_exposure =
      auto_exposure_gain_multiplier_(candidate.gain) * static_cast<float>(candidate.integration_time);
  const float baseline_exposure =
      auto_exposure_gain_multiplier_(baseline.gain) * static_cast<float>(baseline.integration_time);
  return std::isfinite(candidate_exposure) && std::isfinite(baseline_exposure) &&
         candidate_exposure < baseline_exposure;
}

AS7261Component::AutoExposureAdjustment AS7261Component::scale_auto_exposure_candidate_(AutoExposureCandidate candidate,
                                                                                        float multiplier) const {
  AutoExposureAdjustment adjustment{normalize_auto_exposure_candidate_(candidate), false};
  if (!std::isfinite(multiplier) || multiplier <= 0.0f) {
    adjustment.clamped = true;
    return adjustment;
  }

  if (multiplier > 1.0f) {
    const float desired_integration_time = static_cast<float>(adjustment.candidate.integration_time) * multiplier;
    if (desired_integration_time <= static_cast<float>(AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME)) {
      adjustment.candidate.integration_time = clamp_auto_exposure_integration_time_(desired_integration_time);
      return adjustment;
    }

    const float integration_multiplier = static_cast<float>(AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME) /
                                         static_cast<float>(adjustment.candidate.integration_time);
    float remaining_multiplier = multiplier / integration_multiplier;
    adjustment.candidate.integration_time = AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME;

    for (uint8_t step = 0; step < AUTO_EXPOSURE_GAIN_STEP_COUNT && remaining_multiplier > 1.0f; step++) {
      bool stepped = false;
      const AS7261Gain next_gain = next_higher_auto_exposure_gain_(adjustment.candidate.gain, &stepped);
      if (!stepped) {
        adjustment.clamped = true;
        break;
      }
      const float gain_multiplier =
          auto_exposure_gain_multiplier_(next_gain) / auto_exposure_gain_multiplier_(adjustment.candidate.gain);
      adjustment.candidate.gain = next_gain;
      remaining_multiplier /= gain_multiplier;
    }
    if (remaining_multiplier > 1.0f) {
      adjustment.clamped = true;
    }
    return adjustment;
  }

  if (multiplier < 1.0f) {
    const float desired_integration_time = static_cast<float>(adjustment.candidate.integration_time) * multiplier;
    if (desired_integration_time >= static_cast<float>(AUTO_EXPOSURE_MIN_INTEGRATION_TIME)) {
      adjustment.candidate.integration_time = clamp_auto_exposure_integration_time_(desired_integration_time);
      return adjustment;
    }

    const float integration_multiplier = static_cast<float>(AUTO_EXPOSURE_MIN_INTEGRATION_TIME) /
                                         static_cast<float>(adjustment.candidate.integration_time);
    float remaining_multiplier = multiplier / integration_multiplier;
    adjustment.candidate.integration_time = AUTO_EXPOSURE_MIN_INTEGRATION_TIME;

    for (uint8_t step = 0; step < AUTO_EXPOSURE_GAIN_STEP_COUNT && remaining_multiplier < 1.0f; step++) {
      bool stepped = false;
      const AS7261Gain next_gain = next_lower_auto_exposure_gain_(adjustment.candidate.gain, &stepped);
      if (!stepped) {
        adjustment.clamped = true;
        break;
      }
      const float gain_multiplier =
          auto_exposure_gain_multiplier_(next_gain) / auto_exposure_gain_multiplier_(adjustment.candidate.gain);
      adjustment.candidate.gain = next_gain;
      remaining_multiplier /= gain_multiplier;
    }
    if (remaining_multiplier < 1.0f) {
      adjustment.clamped = true;
    }
  }

  return adjustment;
}

AS7261Component::AutoExposureCandidate AS7261Component::normalize_auto_exposure_candidate_(
    AutoExposureCandidate candidate) {
  switch (candidate.gain) {
    case AS7261_GAIN_1X:
    case AS7261_GAIN_3_7X:
    case AS7261_GAIN_16X:
    case AS7261_GAIN_64X:
      break;
    default:
      candidate.gain = AUTO_EXPOSURE_FALLBACK_GAIN;
      break;
  }
  candidate.integration_time = clamp_auto_exposure_integration_time_(static_cast<float>(candidate.integration_time));
  return candidate;
}

bool AS7261Component::auto_exposure_candidates_equal_(AutoExposureCandidate lhs, AutoExposureCandidate rhs) {
  lhs = normalize_auto_exposure_candidate_(lhs);
  rhs = normalize_auto_exposure_candidate_(rhs);
  return lhs.gain == rhs.gain && lhs.integration_time == rhs.integration_time;
}

uint8_t AS7261Component::clamp_auto_exposure_integration_time_(float integration_time) {
  if (!std::isfinite(integration_time) || integration_time <= static_cast<float>(AUTO_EXPOSURE_MIN_INTEGRATION_TIME)) {
    return AUTO_EXPOSURE_MIN_INTEGRATION_TIME;
  }
  if (integration_time >= static_cast<float>(AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME)) {
    return AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME;
  }
  return static_cast<uint8_t>(std::round(integration_time));
}

bool AS7261Component::start_calibrated_frame_readout_() {
  if (this->transport_busy_() || this->sequence_active_() || this->frame_state_ != FrameState::READY) {
    return false;
  }

  CommandSequenceStep steps[COMMAND_SEQUENCE_LENGTH] = {
      {"ATXYZC", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {"ATLUXC", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
      {"ATCCTC", DiagnosticState::IDLE, SequenceFailurePolicy::STOP},
  };
  size_t step_count = 3;
#ifdef USE_SENSOR
  if (this->duv_cie1976_sensor_ != nullptr) {
    steps[step_count] = CommandSequenceStep{"ATDUVC", DiagnosticState::IDLE, SequenceFailurePolicy::CONTINUE};
    step_count++;
  }
#endif
  steps[step_count] = CommandSequenceStep{"ATDATA", DiagnosticState::IDLE, SequenceFailurePolicy::STOP};
  step_count++;
  this->raw_frame_ = RawFrame{};
  this->raw_frame_status_ = RawFrameStatus::INVALID;
  this->clear_exposure_assessment_();
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::RUNNING;
  this->clear_calculated_duv_(CalculatedDuvStatus::INVALID);
  this->clear_derived_color_(DerivedColorStatus::INVALID);
  this->clear_vendor_duv_cie1976_();
  this->frame_state_ = FrameState::READOUT_RUNNING;
  this->sequence_owner_ = SequenceOwner::CALIBRATED_FRAME_READOUT;
  if (this->start_command_sequence_(steps, step_count)) {
    return true;
  }

  this->sequence_owner_ = SequenceOwner::NONE;
  this->raw_frame_ = RawFrame{};
  this->raw_frame_status_ = RawFrameStatus::FAILED;
  this->clear_exposure_assessment_();
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
  this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
  this->clear_derived_color_(DerivedColorStatus::FAILED);
  this->clear_vendor_duv_cie1976_();
  this->clear_terminal_frame_state_();
  return false;
}

bool AS7261Component::handle_finished_calibrated_frame_command_(size_t step_index, TransportResult result) {
  const char *const command = this->command_sequence_[step_index].command;
  const bool vendor_duv_step = command != nullptr && std::strcmp(command, "ATDUVC") == 0;
  const bool raw_frame_step = command != nullptr && std::strcmp(command, "ATDATA") == 0;
  const bool burst_stop_step = command != nullptr && std::strcmp(command, "ATBURST=0") == 0;
  if (result != TransportResult::OK) {
    if (vendor_duv_step) {
      this->clear_vendor_duv_cie1976_();
    }
    return true;
  }

  if (burst_stop_step) {
    return true;
  }

  const char *const value = this->first_response_value_();
  if (raw_frame_step) {
    RawFrame frame{};
    if (!parse_raw_frame_(value, &frame) || raw_frame_empty_(frame)) {
      this->raw_frame_ = RawFrame{};
      this->raw_frame_status_ = RawFrameStatus::MALFORMED;
      this->clear_exposure_assessment_();
      ESP_LOGW(TAG, "Unable to parse AS7261 raw frame response after calibrated readout: %s",
               value == nullptr ? "<empty>" : value);
      return false;
    }
    this->raw_frame_ = frame;
    this->raw_frame_status_ = RawFrameStatus::VALID;
    ESP_LOGD(TAG, "AS7261 raw frame stored: X=%u Y=%u Z=%u NIR=%u Dark=%u Clear=%u",
             static_cast<unsigned>(this->raw_frame_.x), static_cast<unsigned>(this->raw_frame_.y),
             static_cast<unsigned>(this->raw_frame_.z), static_cast<unsigned>(this->raw_frame_.near_ir),
             static_cast<unsigned>(this->raw_frame_.dark), static_cast<unsigned>(this->raw_frame_.clear));
    if (!this->assess_clear_channel_exposure_()) {
      ESP_LOGW(TAG, "Unable to assess AS7261 clear-channel exposure");
    }
    this->update_auto_exposure_policy_();
    return true;
  }

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
    if (vendor_duv_step) {
      this->clear_vendor_duv_cie1976_();
      ESP_LOGW(TAG, "Unable to parse AS7261 vendor CIE 1976 DUV response: %s", value == nullptr ? "<empty>" : value);
      return true;
    }
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
  if (vendor_duv_step) {
    this->vendor_duv_cie1976_ = parsed;
    this->vendor_duv_cie1976_valid_ = true;
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
    const CalibratedFrameStatus calibrated_status =
        this->calibrated_frame_status_ == CalibratedFrameStatus::MALFORMED ? CalibratedFrameStatus::MALFORMED
        : status == SequenceStatus::TIMEOUT                                ? CalibratedFrameStatus::TIMEOUT
        : status == SequenceStatus::OVERFLOW                               ? CalibratedFrameStatus::OVERFLOW
                                                                           : CalibratedFrameStatus::FAILED;
    const CalculatedDuvStatus duv_status = calibrated_status == CalibratedFrameStatus::MALFORMED
                                               ? CalculatedDuvStatus::MALFORMED
                                           : status == SequenceStatus::TIMEOUT  ? CalculatedDuvStatus::TIMEOUT
                                           : status == SequenceStatus::OVERFLOW ? CalculatedDuvStatus::OVERFLOW
                                                                                : CalculatedDuvStatus::FAILED;
    const DerivedColorStatus derived_status = calibrated_status == CalibratedFrameStatus::MALFORMED
                                                  ? DerivedColorStatus::MALFORMED
                                              : status == SequenceStatus::TIMEOUT  ? DerivedColorStatus::TIMEOUT
                                              : status == SequenceStatus::OVERFLOW ? DerivedColorStatus::OVERFLOW
                                                                                   : DerivedColorStatus::FAILED;
    const RawFrameStatus raw_status = this->raw_frame_status_ == RawFrameStatus::VALID       ? RawFrameStatus::VALID
                                      : this->raw_frame_status_ == RawFrameStatus::MALFORMED ? RawFrameStatus::MALFORMED
                                      : this->raw_frame_status_ == RawFrameStatus::TIMEOUT   ? RawFrameStatus::TIMEOUT
                                                                                             : RawFrameStatus::FAILED;
    this->fail_timed_frame_readout_("calibrated frame readout failed", raw_status, calibrated_status, duv_status,
                                    derived_status);
    ESP_LOGW(TAG, "AS7261 calibrated frame readout failed with %s",
             this->calibrated_frame_status_to_string_(this->calibrated_frame_status_));
    return;
  }

  if (!std::isfinite(this->calibrated_frame_.x) || !std::isfinite(this->calibrated_frame_.y) ||
      !std::isfinite(this->calibrated_frame_.z) || !std::isfinite(this->calibrated_frame_.lux) ||
      !std::isfinite(this->calibrated_frame_.cct)) {
    this->fail_timed_frame_readout_("malformed calibrated frame", RawFrameStatus::VALID,
                                    CalibratedFrameStatus::MALFORMED, CalculatedDuvStatus::MALFORMED,
                                    DerivedColorStatus::MALFORMED);
    ESP_LOGW(TAG, "Rejecting malformed AS7261 calibrated frame");
    return;
  }

  this->calibrated_frame_status_ = CalibratedFrameStatus::VALID;
  ESP_LOGD(TAG, "AS7261 calibrated frame stored: X=%f Y=%f Z=%f Lux=%f CCT=%f", this->calibrated_frame_.x,
           this->calibrated_frame_.y, this->calibrated_frame_.z, this->calibrated_frame_.lux,
           this->calibrated_frame_.cct);
  this->clear_terminal_frame_state_();
  if (this->precision_collection_active_()) {
    if (!this->handle_precision_calibrated_frame_()) {
      ESP_LOGW(TAG, "AS7261 precision-mode aggregation failed");
      this->publish_nan_default_measurement_outputs_();
    }
    return;
  }
  if (!this->derive_calculated_duv_()) {
    ESP_LOGW(TAG, "AS7261 calculated Duv derivation failed with %s",
             this->calculated_duv_status_to_string_(this->calculated_duv_status_));
  }
  if (!this->derive_oklab_oklch_()) {
    ESP_LOGW(TAG, "AS7261 OKLab/OKLCH derivation failed with %s",
             this->derived_color_status_to_string_(this->derived_color_status_));
  }
  this->publish_default_measurement_outputs_();
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

bool AS7261Component::default_measurement_outputs_publishable_() const {
  return this->raw_frame_status_ == RawFrameStatus::VALID &&
         this->calibrated_frame_status_ == CalibratedFrameStatus::VALID &&
         this->calculated_duv_status_ == CalculatedDuvStatus::VALID &&
         this->derived_color_status_ == DerivedColorStatus::VALID && this->device_temperature_valid_ &&
         !this->device_temperature_invalid_ && !this->device_temperature_unsafe_ &&
         std::isfinite(this->calibrated_frame_.cct) && std::isfinite(this->calibrated_frame_.lux) &&
         std::isfinite(this->calculated_duv_frame_.duv) && std::isfinite(this->derived_color_frame_.oklab.l) &&
         std::isfinite(this->derived_color_frame_.oklab.a) && std::isfinite(this->derived_color_frame_.oklab.b) &&
         std::isfinite(this->derived_color_frame_.oklch.l) && std::isfinite(this->derived_color_frame_.oklch.c) &&
         std::isfinite(this->derived_color_frame_.oklch.h);
}

void AS7261Component::publish_default_measurement_outputs_() {
#ifdef USE_SENSOR
  if (!this->default_measurement_outputs_publishable_()) {
    this->publish_nan_default_measurement_outputs_();
    return;
  }
  if (this->cct_sensor_ != nullptr) {
    this->cct_sensor_->publish_state(std::isfinite(this->calibrated_frame_.cct) ? this->calibrated_frame_.cct : NAN);
  }
  if (this->calculated_duv_sensor_ != nullptr) {
    this->calculated_duv_sensor_->publish_state(
        std::isfinite(this->calculated_duv_frame_.duv) ? this->calculated_duv_frame_.duv : NAN);
  }
  if (this->lux_sensor_ != nullptr) {
    this->lux_sensor_->publish_state(std::isfinite(this->calibrated_frame_.lux) ? this->calibrated_frame_.lux : NAN);
  }
  if (this->oklab_l_sensor_ != nullptr) {
    this->oklab_l_sensor_->publish_state(
        std::isfinite(this->derived_color_frame_.oklab.l) ? this->derived_color_frame_.oklab.l : NAN);
  }
  if (this->oklab_a_sensor_ != nullptr) {
    this->oklab_a_sensor_->publish_state(
        std::isfinite(this->derived_color_frame_.oklab.a) ? this->derived_color_frame_.oklab.a : NAN);
  }
  if (this->oklab_b_sensor_ != nullptr) {
    this->oklab_b_sensor_->publish_state(
        std::isfinite(this->derived_color_frame_.oklab.b) ? this->derived_color_frame_.oklab.b : NAN);
  }
  if (this->oklch_l_sensor_ != nullptr) {
    this->oklch_l_sensor_->publish_state(
        std::isfinite(this->derived_color_frame_.oklch.l) ? this->derived_color_frame_.oklch.l : NAN);
  }
  if (this->oklch_c_sensor_ != nullptr) {
    this->oklch_c_sensor_->publish_state(
        std::isfinite(this->derived_color_frame_.oklch.c) ? this->derived_color_frame_.oklch.c : NAN);
  }
  if (this->oklch_h_sensor_ != nullptr) {
    this->oklch_h_sensor_->publish_state(
        std::isfinite(this->derived_color_frame_.oklch.h) ? this->derived_color_frame_.oklch.h : NAN);
  }
#endif
  this->publish_optional_measurement_diagnostics_();
  this->publish_manual_measurement_completion_();
}

void AS7261Component::publish_optional_measurement_diagnostics_() {
#ifdef USE_SENSOR
  if (this->duv_cie1976_sensor_ != nullptr) {
    this->duv_cie1976_sensor_->publish_state(
        this->vendor_duv_cie1976_valid_ && std::isfinite(this->vendor_duv_cie1976_) ? this->vendor_duv_cie1976_ : NAN);
  }
  if (this->near_ir_percent_sensor_ != nullptr) {
    float near_ir_percent = NAN;
    if (this->raw_frame_status_ == RawFrameStatus::VALID && this->raw_frame_.clear > 0) {
      near_ir_percent = (CLEAR_PERCENT_SCALE * static_cast<float>(this->raw_frame_.near_ir)) /
                        static_cast<float>(this->raw_frame_.clear);
      if (!std::isfinite(near_ir_percent)) {
        near_ir_percent = NAN;
      }
    }
    this->near_ir_percent_sensor_->publish_state(near_ir_percent);
  }
  if (this->raw_clear_sensor_ != nullptr) {
    this->raw_clear_sensor_->publish_state(
        this->raw_frame_status_ == RawFrameStatus::VALID ? static_cast<float>(this->raw_frame_.clear) : NAN);
  }
  if (this->raw_dark_sensor_ != nullptr) {
    this->raw_dark_sensor_->publish_state(
        this->raw_frame_status_ == RawFrameStatus::VALID ? static_cast<float>(this->raw_frame_.dark) : NAN);
  }
  if (this->raw_near_ir_sensor_ != nullptr) {
    this->raw_near_ir_sensor_->publish_state(
        this->raw_frame_status_ == RawFrameStatus::VALID ? static_cast<float>(this->raw_frame_.near_ir) : NAN);
  }

  Cie1960UcsPoint cie1960{};
  const bool chromaticity_valid = this->calibrated_frame_status_ == CalibratedFrameStatus::VALID &&
                                  cie1960_uv_from_xyz_(this->calibrated_frame_, &cie1960);
  const float u_cie1960 = chromaticity_valid ? cie1960.u : NAN;
  const float v_cie1960 = chromaticity_valid ? cie1960.v : NAN;
  const float u_cie1976 = u_cie1960;
  const float v_cie1976 = chromaticity_valid ? 1.5f * cie1960.v : NAN;
  if (this->u_cie1960_sensor_ != nullptr) {
    this->u_cie1960_sensor_->publish_state(std::isfinite(u_cie1960) ? u_cie1960 : NAN);
  }
  if (this->v_cie1960_sensor_ != nullptr) {
    this->v_cie1960_sensor_->publish_state(std::isfinite(v_cie1960) ? v_cie1960 : NAN);
  }
  if (this->u_cie1976_sensor_ != nullptr) {
    this->u_cie1976_sensor_->publish_state(std::isfinite(u_cie1976) ? u_cie1976 : NAN);
  }
  if (this->v_cie1976_sensor_ != nullptr) {
    this->v_cie1976_sensor_->publish_state(std::isfinite(v_cie1976) ? v_cie1976 : NAN);
  }
  if (this->x_sensor_ != nullptr) {
    this->x_sensor_->publish_state(this->calibrated_frame_status_ == CalibratedFrameStatus::VALID &&
                                           std::isfinite(this->calibrated_frame_.x)
                                       ? this->calibrated_frame_.x
                                       : NAN);
  }
  if (this->y_sensor_ != nullptr) {
    this->y_sensor_->publish_state(this->calibrated_frame_status_ == CalibratedFrameStatus::VALID &&
                                           std::isfinite(this->calibrated_frame_.y)
                                       ? this->calibrated_frame_.y
                                       : NAN);
  }
  if (this->z_sensor_ != nullptr) {
    this->z_sensor_->publish_state(this->calibrated_frame_status_ == CalibratedFrameStatus::VALID &&
                                           std::isfinite(this->calibrated_frame_.z)
                                       ? this->calibrated_frame_.z
                                       : NAN);
  }
#endif
}

void AS7261Component::publish_nan_optional_measurement_diagnostics_() {
  this->clear_vendor_duv_cie1976_();
#ifdef USE_SENSOR
  if (this->duv_cie1976_sensor_ != nullptr) {
    this->duv_cie1976_sensor_->publish_state(NAN);
  }
  if (this->near_ir_percent_sensor_ != nullptr) {
    this->near_ir_percent_sensor_->publish_state(NAN);
  }
  if (this->raw_clear_sensor_ != nullptr) {
    this->raw_clear_sensor_->publish_state(NAN);
  }
  if (this->raw_dark_sensor_ != nullptr) {
    this->raw_dark_sensor_->publish_state(NAN);
  }
  if (this->raw_near_ir_sensor_ != nullptr) {
    this->raw_near_ir_sensor_->publish_state(NAN);
  }
  if (this->u_cie1960_sensor_ != nullptr) {
    this->u_cie1960_sensor_->publish_state(NAN);
  }
  if (this->v_cie1960_sensor_ != nullptr) {
    this->v_cie1960_sensor_->publish_state(NAN);
  }
  if (this->u_cie1976_sensor_ != nullptr) {
    this->u_cie1976_sensor_->publish_state(NAN);
  }
  if (this->v_cie1976_sensor_ != nullptr) {
    this->v_cie1976_sensor_->publish_state(NAN);
  }
  if (this->x_sensor_ != nullptr) {
    this->x_sensor_->publish_state(NAN);
  }
  if (this->y_sensor_ != nullptr) {
    this->y_sensor_->publish_state(NAN);
  }
  if (this->z_sensor_ != nullptr) {
    this->z_sensor_->publish_state(NAN);
  }
#endif
}

void AS7261Component::clear_vendor_duv_cie1976_() {
  this->vendor_duv_cie1976_ = NAN;
  this->vendor_duv_cie1976_valid_ = false;
}

void AS7261Component::publish_nan_default_measurement_outputs_() {
  if (this->precision_collection_status_ != PrecisionCollectionStatus::IDLE) {
    this->reset_precision_collection_();
  }
#ifdef USE_SENSOR
  if (this->cct_sensor_ != nullptr) {
    this->cct_sensor_->publish_state(NAN);
  }
  if (this->calculated_duv_sensor_ != nullptr) {
    this->calculated_duv_sensor_->publish_state(NAN);
  }
  if (this->lux_sensor_ != nullptr) {
    this->lux_sensor_->publish_state(NAN);
  }
  if (this->oklab_l_sensor_ != nullptr) {
    this->oklab_l_sensor_->publish_state(NAN);
  }
  if (this->oklab_a_sensor_ != nullptr) {
    this->oklab_a_sensor_->publish_state(NAN);
  }
  if (this->oklab_b_sensor_ != nullptr) {
    this->oklab_b_sensor_->publish_state(NAN);
  }
  if (this->oklch_l_sensor_ != nullptr) {
    this->oklch_l_sensor_->publish_state(NAN);
  }
  if (this->oklch_c_sensor_ != nullptr) {
    this->oklch_c_sensor_->publish_state(NAN);
  }
  if (this->oklch_h_sensor_ != nullptr) {
    this->oklch_h_sensor_->publish_state(NAN);
  }
#endif
  this->publish_nan_optional_measurement_diagnostics_();
  this->publish_manual_measurement_completion_();
}

void AS7261Component::publish_manual_measurement_completion_() {
  if (!this->active_measurement_manual_ || this->active_measurement_counted_) {
    return;
  }
  this->active_measurement_counted_ = true;
  this->active_measurement_manual_ = false;
  this->completed_measurement_count_++;
#ifdef USE_SENSOR
  if (this->completed_measurement_count_sensor_ != nullptr) {
    this->completed_measurement_count_sensor_->publish_state(static_cast<float>(this->completed_measurement_count_));
  }
#endif
}

void AS7261Component::reset_precision_collection_() {
  for (size_t i = 0; i < PRECISION_FRAME_COUNT; i++) {
    this->precision_frames_[i] = CalibratedFrame{};
  }
  this->precision_frame_count_ = 0;
  this->precision_collection_status_ = PrecisionCollectionStatus::IDLE;
}

bool AS7261Component::handle_precision_calibrated_frame_() {
  if (!calibrated_frame_valid_for_precision_(this->calibrated_frame_) ||
      this->precision_frame_count_ >= PRECISION_FRAME_COUNT) {
    this->precision_collection_status_ = PrecisionCollectionStatus::FAILED;
    this->calibrated_frame_ = CalibratedFrame{};
    this->calibrated_frame_status_ = CalibratedFrameStatus::FAILED;
    this->clear_calculated_duv_(CalculatedDuvStatus::FAILED);
    this->clear_derived_color_(DerivedColorStatus::FAILED);
    return false;
  }

  this->precision_frames_[this->precision_frame_count_] = this->calibrated_frame_;
  this->precision_frame_count_++;
  if (this->precision_frame_count_ < PRECISION_FRAME_COUNT) {
    return this->start_next_precision_frame_();
  }

  return this->finish_precision_collection_();
}

bool AS7261Component::finish_precision_collection_() {
  if (this->precision_frame_count_ != PRECISION_FRAME_COUNT) {
    this->precision_collection_status_ = PrecisionCollectionStatus::FAILED;
    return false;
  }

  const float median_lux =
      median3_(this->precision_frames_[0].lux, this->precision_frames_[1].lux, this->precision_frames_[2].lux);
  const float median_cct =
      median3_(this->precision_frames_[0].cct, this->precision_frames_[1].cct, this->precision_frames_[2].cct);
  const float median_x =
      median3_(this->precision_frames_[0].x, this->precision_frames_[1].x, this->precision_frames_[2].x);
  const float median_y =
      median3_(this->precision_frames_[0].y, this->precision_frames_[1].y, this->precision_frames_[2].y);
  const float median_z =
      median3_(this->precision_frames_[0].z, this->precision_frames_[1].z, this->precision_frames_[2].z);
  if (!std::isfinite(median_lux) || !std::isfinite(median_cct) || !std::isfinite(median_x) ||
      !std::isfinite(median_y) || !std::isfinite(median_z)) {
    this->precision_collection_status_ = PrecisionCollectionStatus::FAILED;
    return false;
  }

  size_t discard_index = 0;
  float largest_error = -1.0f;
  for (size_t i = 0; i < PRECISION_FRAME_COUNT; i++) {
    const CalibratedFrame &frame = this->precision_frames_[i];
    const float error = precision_field_error_(frame.lux, median_lux) + precision_field_error_(frame.cct, median_cct) +
                        precision_field_error_(frame.x, median_x) + precision_field_error_(frame.y, median_y) +
                        precision_field_error_(frame.z, median_z);
    if (!std::isfinite(error)) {
      this->precision_collection_status_ = PrecisionCollectionStatus::FAILED;
      return false;
    }
    if (error > largest_error) {
      largest_error = error;
      discard_index = i;
    }
  }

  CalibratedFrame averaged{};
  size_t kept_count = 0;
  for (size_t i = 0; i < PRECISION_FRAME_COUNT; i++) {
    if (i == discard_index) {
      continue;
    }
    const CalibratedFrame &frame = this->precision_frames_[i];
    averaged.x += frame.x;
    averaged.y += frame.y;
    averaged.z += frame.z;
    averaged.lux += frame.lux;
    averaged.cct += frame.cct;
    kept_count++;
  }
  if (kept_count != 2) {
    this->precision_collection_status_ = PrecisionCollectionStatus::FAILED;
    return false;
  }

  averaged.x *= 0.5f;
  averaged.y *= 0.5f;
  averaged.z *= 0.5f;
  averaged.lux *= 0.5f;
  averaged.cct *= 0.5f;
  if (!calibrated_frame_valid_for_precision_(averaged)) {
    this->precision_collection_status_ = PrecisionCollectionStatus::FAILED;
    return false;
  }

  this->calibrated_frame_ = averaged;
  this->calibrated_frame_status_ = CalibratedFrameStatus::VALID;
  this->precision_collection_status_ = PrecisionCollectionStatus::COMPLETE;
  if (!this->derive_calculated_duv_()) {
    ESP_LOGW(TAG, "AS7261 precision-mode calculated Duv derivation failed with %s",
             this->calculated_duv_status_to_string_(this->calculated_duv_status_));
  }
  if (!this->derive_oklab_oklch_()) {
    ESP_LOGW(TAG, "AS7261 precision-mode OKLab/OKLCH derivation failed with %s",
             this->derived_color_status_to_string_(this->derived_color_status_));
  }
  this->publish_default_measurement_outputs_();
  this->reset_precision_collection_();
  return true;
}

bool AS7261Component::start_next_precision_frame_() {
  this->raw_frame_ = RawFrame{};
  this->raw_frame_status_ = RawFrameStatus::INVALID;
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::INVALID;
  this->clear_calculated_duv_(CalculatedDuvStatus::INVALID);
  this->clear_derived_color_(DerivedColorStatus::INVALID);
  return this->start_measurement_cycle_();
}

bool AS7261Component::calibrated_frame_valid_for_precision_(const CalibratedFrame &frame) {
  return std::isfinite(frame.lux) && std::isfinite(frame.cct) && std::isfinite(frame.x) && std::isfinite(frame.y) &&
         std::isfinite(frame.z) && frame.lux >= 0.0f && frame.cct >= 0.0f && frame.x >= 0.0f && frame.y >= 0.0f &&
         frame.z >= 0.0f;
}

float AS7261Component::median3_(float a, float b, float c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) {
    return b;
  }
  if ((b <= a && a <= c) || (c <= a && a <= b)) {
    return a;
  }
  return c;
}

float AS7261Component::precision_field_error_(float value, float median) {
  if (!std::isfinite(value) || !std::isfinite(median)) {
    return 1.0f;
  }
  const float denominator = std::fmax(std::fmax(std::fabs(value), std::fabs(median)), PRECISION_NORMALIZATION_FLOOR);
  if (!std::isfinite(denominator) || denominator <= 0.0f) {
    return 1.0f;
  }
  const float error = std::fabs(value - median) / denominator;
  if (!std::isfinite(error)) {
    return 1.0f;
  }
  return error > 1.0f ? 1.0f : error;
}

bool AS7261Component::derive_oklab_oklch_() {
  if (this->calibrated_frame_status_ != CalibratedFrameStatus::VALID ||
      !calibrated_xyz_valid_for_oklab_(this->calibrated_frame_)) {
    this->clear_derived_color_(DerivedColorStatus::MALFORMED);
    return false;
  }

  const float reference_illuminance = this->oklab_reference_illuminance_;
  const float normalized_x = this->calibrated_frame_.x / reference_illuminance;
  const float normalized_y = this->calibrated_frame_.y / reference_illuminance;
  const float normalized_z = this->calibrated_frame_.z / reference_illuminance;

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

bool AS7261Component::calibrated_xyz_valid_for_oklab_(const CalibratedFrame &frame) const {
  return std::isfinite(frame.x) && std::isfinite(frame.y) && std::isfinite(frame.z) && frame.x >= 0.0f &&
         frame.y >= 0.0f && frame.z >= 0.0f && std::isfinite(this->oklab_reference_illuminance_) &&
         this->oklab_reference_illuminance_ > 0.0f;
}

void AS7261Component::clear_terminal_frame_state_() {
  if (this->frame_state_ == FrameState::READY || this->frame_state_ == FrameState::READOUT_RUNNING ||
      this->frame_state_ == FrameState::TIMEOUT || this->frame_state_ == FrameState::ERROR) {
    this->frame_state_ = FrameState::IDLE;
  }
}

void AS7261Component::schedule_frame_timed_readout_(uint32_t wait_ms) {
  this->frame_wait_started_millis_ = millis();
  this->frame_timed_readout_wait_ms_ = wait_ms;
  this->frame_state_ = FrameState::WAITING_TIMED_READOUT;
}

bool AS7261Component::retry_timed_frame_readout_(const char *reason) {
  if (this->frame_readout_attempt_ + 1U >= FRAME_TIMED_READOUT_ATTEMPT_LIMIT) {
    return false;
  }
  this->frame_readout_attempt_++;
  this->raw_frame_ = RawFrame{};
  this->raw_frame_status_ = RawFrameStatus::INVALID;
  this->clear_exposure_assessment_();
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = CalibratedFrameStatus::INVALID;
  this->clear_calculated_duv_(CalculatedDuvStatus::INVALID);
  this->clear_derived_color_(DerivedColorStatus::INVALID);
  this->clear_vendor_duv_cie1976_();
  this->schedule_frame_timed_readout_(FRAME_TIMED_READOUT_RETRY_MARGIN_MS);
  ESP_LOGW(TAG, "AS7261 timed UART readout produced %s; retrying once after %u ms", reason,
           static_cast<unsigned>(FRAME_TIMED_READOUT_RETRY_MARGIN_MS));
  return true;
}

void AS7261Component::fail_timed_frame_readout_(const char *reason, RawFrameStatus raw_status,
                                                CalibratedFrameStatus calibrated_status, CalculatedDuvStatus duv_status,
                                                DerivedColorStatus derived_status) {
  if (raw_status != RawFrameStatus::VALID) {
    this->raw_frame_ = RawFrame{};
  }
  this->raw_frame_status_ = raw_status;
  this->clear_exposure_assessment_();
  this->calibrated_frame_ = CalibratedFrame{};
  this->calibrated_frame_status_ = calibrated_status;
  this->clear_calculated_duv_(duv_status);
  this->clear_derived_color_(derived_status);
  this->clear_vendor_duv_cie1976_();
  if (this->retry_timed_frame_readout_(reason)) {
    return;
  }
  this->start_reset_pulse_("timed UART frame readout failed after retry");
  this->publish_nan_default_measurement_outputs_();
  this->clear_terminal_frame_state_();
  this->frame_readout_attempt_ = 0;
  ESP_LOGW(TAG, "AS7261 timed UART readout failed after retry: %s", reason);
}

uint32_t AS7261Component::calculate_frame_timed_readout_wait_ms_() const {
  const uint32_t conversion_time_us =
      2UL * static_cast<uint32_t>(this->measurement_integration_time_()) * AS7261_INTEGRATION_TIME_STEP_US;
  const uint32_t conversion_time_ms = (conversion_time_us + 999U) / 1000U;
  return conversion_time_ms + FRAME_TIMED_READOUT_MARGIN_MS;
}

uint8_t AS7261Component::measurement_integration_time_() const {
  if (this->manual_exposure_ && this->integration_time_ != 0) {
    return this->integration_time_;
  }
  if (this->auto_exposure_policy_.candidate_applied) {
    const AutoExposureCandidate candidate =
        normalize_auto_exposure_candidate_(this->auto_exposure_policy_.applied_candidate);
    return candidate.integration_time;
  }
  if (this->auto_exposure_policy_.candidate_initialized) {
    const AutoExposureCandidate candidate =
        normalize_auto_exposure_candidate_(this->auto_exposure_policy_.current_candidate);
    return candidate.integration_time;
  }
  return AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME;
}

bool AS7261Component::raw_frame_empty_(const RawFrame &frame) {
  return frame.x == 0 && frame.y == 0 && frame.z == 0 && frame.near_ir == 0 && frame.dark == 0 && frame.clear == 0;
}

uint8_t AS7261Component::single_bank_probe_integration_time_() const {
  if (this->manual_exposure_ && this->integration_time_ != 0) {
    return this->integration_time_;
  }
  if (this->auto_exposure_policy_.candidate_initialized) {
    const AutoExposureCandidate candidate =
        normalize_auto_exposure_candidate_(this->auto_exposure_policy_.current_candidate);
    return candidate.integration_time;
  }
  return AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME;
}

uint8_t AS7261Component::calculate_single_bank_probe_interval_() const {
  const uint32_t integration_time_us =
      static_cast<uint32_t>(this->single_bank_probe_integration_time_()) * AS7261_INTEGRATION_TIME_STEP_US;
  if (integration_time_us == 0) {
    return 1;
  }
  uint32_t interval = (SINGLE_BANK_PROBE_REPEAT_INTERVAL_MIN_US + integration_time_us - 1U) / integration_time_us;
  interval = std::max<uint32_t>(interval, 1U);
  interval = std::min<uint32_t>(interval, 255U);
  return static_cast<uint8_t>(interval);
}

uint32_t AS7261Component::calculate_single_bank_probe_timed_readout_wait_ms_() const {
  const uint32_t integration_time_us =
      static_cast<uint32_t>(this->single_bank_probe_integration_time_()) * AS7261_INTEGRATION_TIME_STEP_US;
  const uint32_t conversion_time_us = integration_time_us > SINGLE_BANK_PROBE_REPEAT_INTERVAL_MIN_US
                                          ? integration_time_us
                                          : SINGLE_BANK_PROBE_REPEAT_INTERVAL_MIN_US;
  const uint32_t conversion_time_ms = (conversion_time_us + 999U) / 1000U;
  return conversion_time_ms + FRAME_TIMED_READOUT_MARGIN_MS;
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
  if (!AS7261Component::parse_device_temperature_(value, &temperature_c, &invalid)) {
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
  if (std::strcmp(this->command_buffer_, "ATXYZC") == 0) {
    if (this->temp_atxyzc_raw_length_ < sizeof(this->temp_atxyzc_raw_bytes_)) {
      this->temp_atxyzc_raw_bytes_[this->temp_atxyzc_raw_length_++] = byte;
    } else {
      this->temp_atxyzc_raw_overflow_ = true;
    }
  }
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

  const char *const line_begin = AS7261Component::trim_left_(this->line_buffer_);
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
  const char *const line_begin = AS7261Component::trim_left_(this->line_buffer_);
  const char *const line_end = AS7261Component::trim_right_(line_begin, line_begin + std::strlen(line_begin));
  if (line_end < line_begin + terminal_length ||
      std::strncmp(line_end - terminal_length, terminal, terminal_length) != 0) {
    return false;
  }
  if (line_end != line_begin + terminal_length && !AS7261Component::is_space_(*(line_end - terminal_length - 1))) {
    return false;
  }

  const char *const value_end = AS7261Component::trim_right_(line_begin, line_end - terminal_length);
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

  if (std::strcmp(this->command_buffer_, "ATXYZC") == 0) {
    temp_log_atxyzc_hex_bytes_("UART", this->temp_atxyzc_raw_bytes_, this->temp_atxyzc_raw_length_,
                               this->temp_atxyzc_raw_overflow_);
    temp_log_atxyzc_hex_bytes_("assembled response_buffer", reinterpret_cast<const uint8_t *>(this->response_buffer_),
                               this->response_length_, false);
  }

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
  const char *const begin = AS7261Component::trim_left_(this->response_buffer_);
  if (begin == nullptr) {
    this->line_buffer_[0] = '\0';
    return this->line_buffer_;
  }

  const char *line_end = begin;
  while (*line_end != '\0' && *line_end != '\n') {
    line_end++;
  }
  const char *const end = AS7261Component::trim_right_(begin, line_end);
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
    parsed = (parsed * 10U) + static_cast<uint32_t>(digit);
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
        exponent = std::min<int16_t>(exponent, 64);
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
    case FrameState::WAITING_TIMED_READOUT:
      return "waiting_timed_readout";
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

const char *AS7261Component::single_bank_probe_state_to_string_(SingleBankProbeState state) {
  switch (state) {
    case SingleBankProbeState::IDLE:
      return "idle";
    case SingleBankProbeState::CONFIGURE_RUNNING:
      return "configure_running";
    case SingleBankProbeState::WAITING_TIMED_READOUT:
      return "waiting_timed_readout";
    case SingleBankProbeState::STOP_RUNNING:
      return "stop_running";
    case SingleBankProbeState::RAW_READOUT_RUNNING:
      return "raw_readout_running";
    default:
      return "unknown";
  }
}

const char *AS7261Component::single_bank_probe_status_to_string_(SingleBankProbeStatus status) {
  switch (status) {
    case SingleBankProbeStatus::INVALID:
      return "invalid";
    case SingleBankProbeStatus::RUNNING:
      return "running";
    case SingleBankProbeStatus::VALID:
      return "valid";
    case SingleBankProbeStatus::FAILED:
      return "failed";
    case SingleBankProbeStatus::TIMEOUT:
      return "timeout";
    case SingleBankProbeStatus::OVERFLOW:
      return "overflow";
    case SingleBankProbeStatus::MALFORMED:
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

const char *AS7261Component::exposure_assessment_status_to_string_(ExposureAssessmentStatus status) {
  switch (status) {
    case ExposureAssessmentStatus::INVALID:
      return "invalid";
    case ExposureAssessmentStatus::CLEAR_OVEREXPOSED:
      return "clear_overexposed";
    case ExposureAssessmentStatus::CLEAR_NEAR_SATURATION:
      return "clear_near_saturation";
    case ExposureAssessmentStatus::CLEAR_TARGET:
      return "clear_target";
    case ExposureAssessmentStatus::CLEAR_TOO_DARK:
      return "clear_too_dark";
    case ExposureAssessmentStatus::CLEAR_TOO_DARK_JUMP:
      return "clear_too_dark_jump";
    default:
      return "unknown";
  }
}

const char *AS7261Component::exposure_assessment_guidance_to_string_(ExposureAssessmentGuidance guidance) {
  switch (guidance) {
    case ExposureAssessmentGuidance::INVALID:
      return "invalid";
    case ExposureAssessmentGuidance::ACCEPT:
      return "accept";
    case ExposureAssessmentGuidance::DECREASE:
      return "decrease";
    case ExposureAssessmentGuidance::INCREASE:
      return "increase";
    case ExposureAssessmentGuidance::JUMP_INCREASE:
      return "jump_increase";
    case ExposureAssessmentGuidance::RECOVER_OVEREXPOSED:
      return "recover_overexposed";
    default:
      return "unknown";
  }
}

const char *AS7261Component::auto_exposure_policy_action_to_string_(AutoExposurePolicyAction action) {
  switch (action) {
    case AutoExposurePolicyAction::INERT:
      return "inert";
    case AutoExposurePolicyAction::PENDING_ASSESSMENT:
      return "pending_assessment";
    case AutoExposurePolicyAction::ACCEPT_CURRENT:
      return "accept_current";
    case AutoExposurePolicyAction::DECREASE:
      return "decrease";
    case AutoExposurePolicyAction::INCREASE:
      return "increase";
    case AutoExposurePolicyAction::JUMP_INCREASE:
      return "jump_increase";
    case AutoExposurePolicyAction::RECOVER_OVEREXPOSED:
      return "recover_overexposed";
    case AutoExposurePolicyAction::INVALID_ASSESSMENT:
      return "invalid_assessment";
    default:
      return "unknown";
  }
}

float AS7261Component::auto_exposure_gain_multiplier_(AS7261Gain gain) {
  switch (gain) {
    case AS7261_GAIN_1X:
      return 1.0f;
    case AS7261_GAIN_3_7X:
      return 3.7f;
    case AS7261_GAIN_16X:
      return 16.0f;
    case AS7261_GAIN_64X:
      return 64.0f;
    default:
      return 16.0f;
  }
}

AS7261Gain AS7261Component::next_higher_auto_exposure_gain_(AS7261Gain gain, bool *stepped) {
  if (stepped != nullptr) {
    *stepped = true;
  }
  switch (gain) {
    case AS7261_GAIN_1X:
      return AS7261_GAIN_3_7X;
    case AS7261_GAIN_3_7X:
      return AS7261_GAIN_16X;
    case AS7261_GAIN_16X:
      return AS7261_GAIN_64X;
    case AS7261_GAIN_64X:
    default:
      if (stepped != nullptr) {
        *stepped = false;
      }
      return AS7261_GAIN_64X;
  }
}

AS7261Gain AS7261Component::next_lower_auto_exposure_gain_(AS7261Gain gain, bool *stepped) {
  if (stepped != nullptr) {
    *stepped = true;
  }
  switch (gain) {
    case AS7261_GAIN_64X:
      return AS7261_GAIN_16X;
    case AS7261_GAIN_16X:
      return AS7261_GAIN_3_7X;
    case AS7261_GAIN_3_7X:
      return AS7261_GAIN_1X;
    case AS7261_GAIN_1X:
    default:
      if (stepped != nullptr) {
        *stepped = false;
      }
      return AS7261_GAIN_1X;
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

#ifdef USE_BUTTON
void AS7261MeasureButton::press_action() { this->parent_->request_manual_measurement(); }
#endif

}  // namespace esphome::as7261
