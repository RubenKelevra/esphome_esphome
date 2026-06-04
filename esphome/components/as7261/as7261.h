#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::as7261 {

enum AS7261Gain : uint8_t {
  AS7261_GAIN_1X,
  AS7261_GAIN_3_7X,
  AS7261_GAIN_16X,
  AS7261_GAIN_64X,
};

class AS7261Component : public PollingComponent, public uart::UARTDevice {
#ifdef USE_SENSOR
  SUB_SENSOR(cct)
  SUB_SENSOR(calculated_duv)
  SUB_SENSOR(lux)
  SUB_SENSOR(oklab_l)
  SUB_SENSOR(oklab_a)
  SUB_SENSOR(oklab_b)
  SUB_SENSOR(oklch_l)
  SUB_SENSOR(oklch_c)
  SUB_SENSOR(oklch_h)
  SUB_SENSOR(completed_measurement_count)
  SUB_SENSOR(device_temperature)
  SUB_SENSOR(duv_cie1976)
  SUB_SENSOR(near_ir_percent)
  SUB_SENSOR(raw_clear)
  SUB_SENSOR(raw_dark)
  SUB_SENSOR(raw_near_ir)
  SUB_SENSOR(u_cie1960)
  SUB_SENSOR(v_cie1960)
  SUB_SENSOR(u_cie1976)
  SUB_SENSOR(v_cie1976)
  SUB_SENSOR(x)
  SUB_SENSOR(y)
  SUB_SENSOR(z)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(firmware_version)
#endif

  enum class ManualExposureCommandState : uint8_t {
    NOT_CONFIGURED,
    PENDING,
    RUNNING,
    APPLIED,
    FAILED,
  };

  enum class AutoExposureCommandState : uint8_t {
    NOT_CONFIGURED,
    PENDING,
    RUNNING,
    APPLIED,
    FAILED,
  };

 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;
  void request_manual_measurement();

  void set_int_pin(InternalGPIOPin *int_pin) { this->int_pin_ = int_pin; }
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  void set_precision_mode(bool precision_mode) { this->precision_mode_ = precision_mode; }
  void set_oklab_reference_illuminance(float illuminance_lx);
  void set_manual_exposure(AS7261Gain gain, uint8_t integration_time) {
    this->manual_exposure_ = true;
    this->manual_exposure_state_ = ManualExposureCommandState::PENDING;
    this->gain_ = gain;
    this->integration_time_ = integration_time;
    this->clear_auto_exposure_policy_();
  }

 protected:
  enum class TransportState : uint8_t {
    IDLE,
    WAITING_RESPONSE,
    RESET_ASSERTED,
  };

  enum class TransportResult : uint8_t {
    NONE,
    OK,
    ERROR,
    TIMEOUT,
    OVERFLOW,
  };

  enum class DiagnosticState : uint8_t {
    IDLE,
    FIRMWARE_VERSION,
    DEVICE_TEMPERATURE,
  };

  enum class SequenceStatus : uint8_t {
    IDLE,
    RUNNING,
    COMPLETED,
    FAILED,
    TIMEOUT,
    OVERFLOW,
  };

  enum class SequenceFailurePolicy : uint8_t {
    STOP,
    CONTINUE,
  };

  enum class SequenceOwner : uint8_t {
    NONE,
    DIAGNOSTIC_READOUT,
    MANUAL_EXPOSURE,
    AUTO_EXPOSURE,
    FRAME_TRIGGER,
    RAW_FRAME_READOUT,
    CALIBRATED_FRAME_READOUT,
    SINGLE_BANK_PROBE_CONFIGURE,
    SINGLE_BANK_PROBE_STOP,
    SINGLE_BANK_PROBE_RAW_READOUT,
  };

  enum class FrameState : uint8_t {
    IDLE,
    TRIGGER_RUNNING,
    WAITING_TIMED_READOUT,
    READY,
    READOUT_RUNNING,
    TIMEOUT,
    ERROR,
  };

  enum class RawFrameStatus : uint8_t {
    INVALID,
    RUNNING,
    VALID,
    FAILED,
    TIMEOUT,
    OVERFLOW,
    MALFORMED,
  };

  enum class SingleBankProbeState : uint8_t {
    IDLE,
    CONFIGURE_RUNNING,
    WAITING_TIMED_READOUT,
    STOP_RUNNING,
    RAW_READOUT_RUNNING,
  };

  enum class SingleBankProbeStatus : uint8_t {
    INVALID,
    RUNNING,
    VALID,
    FAILED,
    TIMEOUT,
    OVERFLOW,
    MALFORMED,
  };

  enum class AutoExposureConvergenceState : uint8_t {
    IDLE,
    APPLYING_CANDIDATE,
    PROBING,
    FAILED,
  };

  enum class CalibratedFrameStatus : uint8_t {
    INVALID,
    RUNNING,
    VALID,
    FAILED,
    TIMEOUT,
    OVERFLOW,
    MALFORMED,
  };

  enum class DerivedColorStatus : uint8_t {
    INVALID,
    VALID,
    FAILED,
    TIMEOUT,
    OVERFLOW,
    MALFORMED,
  };

  enum class CalculatedDuvStatus : uint8_t {
    INVALID,
    VALID,
    FAILED,
    TIMEOUT,
    OVERFLOW,
    MALFORMED,
  };

  enum class ExposureAssessmentStatus : uint8_t {
    INVALID,
    CLEAR_OVEREXPOSED,
    CLEAR_NEAR_SATURATION,
    CLEAR_TARGET,
    CLEAR_TOO_DARK,
    CLEAR_TOO_DARK_JUMP,
  };

  enum class ExposureAssessmentGuidance : uint8_t {
    INVALID,
    ACCEPT,
    DECREASE,
    INCREASE,
    JUMP_INCREASE,
    RECOVER_OVEREXPOSED,
  };

  enum class AutoExposurePolicyAction : uint8_t {
    INERT,
    PENDING_ASSESSMENT,
    ACCEPT_CURRENT,
    DECREASE,
    INCREASE,
    JUMP_INCREASE,
    RECOVER_OVEREXPOSED,
    INVALID_ASSESSMENT,
  };

  struct RawFrame {
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t near_ir;
    uint16_t dark;
    uint16_t clear;
  };

  struct CalibratedFrame {
    float x;
    float y;
    float z;
    float lux;
    float cct;
  };

  enum class PrecisionCollectionStatus : uint8_t {
    IDLE,
    COLLECTING,
    COMPLETE,
    FAILED,
  };

  struct Cie1960UcsPoint {
    float u;
    float v;
  };

  struct CalculatedDuvFrame {
    float duv;
    Cie1960UcsPoint measured;
    Cie1960UcsPoint planckian;
  };

  struct ExposureAssessment {
    float clear_percent;
    ExposureAssessmentStatus status;
    ExposureAssessmentGuidance guidance;
  };

  struct AutoExposureCandidate {
    AS7261Gain gain;
    uint8_t integration_time;
  };

  struct AutoExposureAdjustment {
    AutoExposureCandidate candidate;
    bool clamped;
  };

  struct DarkChannelRecoveryState {
    float dark_to_clear_ratio;
    uint32_t learned_at_millis;
    bool learned;
  };

  struct AutoExposurePolicy {
    AutoExposureCandidate current_candidate;
    AutoExposureCandidate next_candidate;
    AutoExposureCandidate applied_candidate;
    AutoExposurePolicyAction action;
    bool candidate_initialized;
    bool candidate_applied;
    bool accepted;
    bool dark_channel_recovery_required;
    bool clamped;
  };

  struct OklabColor {
    float l;
    float a;
    float b;
  };

  struct OklchColor {
    float l;
    float c;
    float h;
  };

  struct DerivedColorFrame {
    OklabColor oklab;
    OklchColor oklch;
  };

  struct CommandSequenceStep {
    const char *command;
    DiagnosticState diagnostic_state;
    SequenceFailurePolicy failure_policy;
  };

  static constexpr size_t COMMAND_BUFFER_LENGTH = 48;
  static constexpr size_t LINE_BUFFER_LENGTH = 96;
  static constexpr size_t RESPONSE_BUFFER_LENGTH = 192;
  static constexpr size_t ATXYZC_RAW_DEBUG_BUFFER_LENGTH = 256;
  static constexpr size_t COMMAND_SEQUENCE_LENGTH = 6;
  static constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;
  static constexpr uint32_t RESET_PULSE_MS = 2;
  static constexpr float DEVICE_TEMPERATURE_UNSAFE_C = 76.5f;
  static constexpr uint32_t AS7261_INTEGRATION_TIME_STEP_US = 2800;
  static constexpr uint8_t AUTO_EXPOSURE_FALLBACK_INTEGRATION_TIME = 255;
  static constexpr uint32_t FRAME_TIMED_READOUT_MARGIN_MS = 250;
  static constexpr uint32_t FRAME_TIMED_READOUT_RETRY_MARGIN_MS = 1000;
  static constexpr uint8_t FRAME_TIMED_READOUT_ATTEMPT_LIMIT = 2;
  static constexpr uint8_t SINGLE_BANK_PROBE_SENSOR_MODE = 0;
  static constexpr uint32_t SINGLE_BANK_PROBE_REPEAT_INTERVAL_MIN_US = 50000;
  static constexpr float DEFAULT_OKLAB_REFERENCE_ILLUMINANCE_LX = 1000.0f;
  static constexpr float OKLCH_ZERO_CHROMA_HUE_DEGREES = 0.0f;
  static constexpr float DEGREES_PER_RADIAN = 57.29577951308232f;
  // Piecewise CCT-to-CIE 1931 xy Planckian approximation range. Reject, do not extrapolate.
  static constexpr float PLANCKIAN_LOCUS_MIN_CCT_K = 1667.0f;
  static constexpr float PLANCKIAN_LOCUS_LOW_CCT_K = 2222.0f;
  static constexpr float PLANCKIAN_LOCUS_MID_CCT_K = 4000.0f;
  static constexpr float PLANCKIAN_LOCUS_MAX_CCT_K = 25000.0f;
  static constexpr float CIE_1960_UCS_DENOMINATOR_EPSILON = 1.0e-6f;
  static constexpr float RAW_CLEAR_FULL_SCALE = 65535.0f;
  static constexpr float CLEAR_PERCENT_SCALE = 100.0f;
  static constexpr float CLEAR_OVEREXPOSED_PERCENT = 98.0f;
  static constexpr float CLEAR_NEAR_SATURATION_PERCENT = 88.0f;
  static constexpr float CLEAR_TARGET_LOW_PERCENT = 20.0f;
  static constexpr float CLEAR_TRUSTED_LOW_PERCENT = 5.0f;
  static constexpr uint8_t AUTO_EXPOSURE_MIN_INTEGRATION_TIME = 1;
  static constexpr AS7261Gain AUTO_EXPOSURE_FALLBACK_GAIN = AS7261_GAIN_16X;
  static constexpr float AUTO_EXPOSURE_TARGET_CLEAR_PERCENT =
      (CLEAR_TARGET_LOW_PERCENT + CLEAR_NEAR_SATURATION_PERCENT) / 2.0f;
  static constexpr float AUTO_EXPOSURE_JUMP_INCREASE_MULTIPLIER = 8.0f;
  static constexpr uint8_t AUTO_EXPOSURE_GAIN_STEP_COUNT = 3;
  // Fixed private bound: initial fallback probe plus four correction probes. This keeps auto mode finite
  // without exposing retry tuning through the public YAML contract.
  static constexpr uint8_t AUTO_EXPOSURE_CONVERGENCE_ATTEMPT_LIMIT = 5;
  static constexpr uint32_t DARK_CHANNEL_RECOVERY_STATE_MAX_AGE_MS = 300000;
  static constexpr float DARK_CHANNEL_RECOVERY_MIN_RATIO = 1.0e-6f;
  static constexpr float DARK_CHANNEL_RECOVERY_MAX_RATIO = 1.0f;
  static constexpr float DARK_CHANNEL_RECOVERY_CURRENT_DARK_MAX_PERCENT = CLEAR_OVEREXPOSED_PERCENT;
  static constexpr size_t PRECISION_FRAME_COUNT = 3;
  static constexpr float PRECISION_NORMALIZATION_FLOOR = 1.0f;

  const char *gain_to_string_() const;
  bool transport_busy_() const { return this->transport_state_ != TransportState::IDLE; }
  bool sequence_active_() const { return this->sequence_status_ == SequenceStatus::RUNNING; }
  bool diagnostic_active_() const {
    return this->sequence_active_() || this->diagnostic_state_ != DiagnosticState::IDLE;
  }
  bool frame_status_active_() const {
    return this->frame_state_ != FrameState::IDLE && this->frame_state_ != FrameState::TRIGGER_RUNNING;
  }
  bool single_bank_probe_active_() const { return this->single_bank_probe_state_ != SingleBankProbeState::IDLE; }
  bool component_busy_() const {
    return this->transport_busy_() || this->sequence_active_() || this->frame_status_active_() ||
           this->single_bank_probe_active_();
  }
  bool begin_at_command_(const char *command, uint32_t timeout_ms = COMMAND_TIMEOUT_MS);
  void poll_transport_();
  void poll_command_sequence_();
  bool start_command_sequence_(const CommandSequenceStep *steps, size_t step_count);
  bool start_current_sequence_step_();
  void handle_finished_sequence_step_();
  void finish_command_sequence_(SequenceStatus status);
  bool start_diagnostic_readout_();
  bool start_manual_exposure_commands_();
  bool start_auto_exposure_candidate_commands_();
  bool start_one_shot_frame_trigger_();
  bool start_single_bank_probe_();
  bool start_raw_frame_readout_();
  bool start_calibrated_frame_readout_();
  bool start_measurement_cycle_();
  void poll_frame_trigger_();
  void poll_single_bank_probe_();
  void handle_finished_diagnostic_readout_(SequenceStatus status);
  void handle_finished_frame_trigger_(SequenceStatus status);
  void clear_terminal_frame_state_();
  void handle_finished_raw_frame_readout_(SequenceStatus status);
  void handle_finished_single_bank_probe_configure_(SequenceStatus status);
  void handle_finished_single_bank_probe_stop_(SequenceStatus status);
  void handle_finished_single_bank_probe_raw_readout_(SequenceStatus status);
  bool start_single_bank_probe_stop_();
  bool start_single_bank_probe_raw_readout_();
  void clear_single_bank_probe_result_(SingleBankProbeStatus status);
  void handle_finished_calibrated_frame_readout_(SequenceStatus status);
  void clear_calculated_duv_(CalculatedDuvStatus status);
  void clear_derived_color_(DerivedColorStatus status);
  void clear_exposure_assessment_();
  bool assess_clear_channel_exposure_();
  bool assess_clear_channel_exposure_(const RawFrame &frame, bool frame_valid);
  void learn_dark_channel_recovery_(const RawFrame &frame, const ExposureAssessment &assessment);
  bool dark_channel_recovery_state_valid_() const;
  void clear_auto_exposure_policy_();
  bool start_auto_exposure_convergence_();
  bool start_auto_exposure_probe_attempt_();
  void handle_finished_auto_exposure_probe_();
  void fail_auto_exposure_convergence_();
  void initialize_auto_exposure_policy_();
  bool update_auto_exposure_policy_();
  bool recover_auto_exposure_from_dark_channel_();
  bool auto_exposure_candidate_applied_() const;
  static bool auto_exposure_candidate_lower_(AutoExposureCandidate candidate, AutoExposureCandidate baseline);
  AutoExposureAdjustment scale_auto_exposure_candidate_(AutoExposureCandidate candidate, float multiplier) const;
  static AutoExposureCandidate normalize_auto_exposure_candidate_(AutoExposureCandidate candidate);
  static bool auto_exposure_candidates_equal_(AutoExposureCandidate lhs, AutoExposureCandidate rhs);
  static uint8_t clamp_auto_exposure_integration_time_(float integration_time);
  bool default_measurement_outputs_publishable_() const;
  void publish_default_measurement_outputs_();
  void publish_optional_measurement_diagnostics_();
  void publish_nan_default_measurement_outputs_();
  void publish_nan_optional_measurement_diagnostics_();
  void publish_manual_measurement_completion_();
  void clear_vendor_duv_cie1976_();
  bool precision_collection_active_() const {
    return this->precision_collection_status_ == PrecisionCollectionStatus::COLLECTING;
  }
  void reset_precision_collection_();
  bool handle_precision_calibrated_frame_();
  bool finish_precision_collection_();
  bool start_next_precision_frame_();
  static bool calibrated_frame_valid_for_precision_(const CalibratedFrame &frame);
  static float median3_(float a, float b, float c);
  static float precision_field_error_(float value, float median);
  static bool derive_calibrated_frame_from_raw_(const RawFrame &raw_frame, CalibratedFrame *calibrated_frame);
  bool derive_calculated_duv_();
  bool derive_oklab_oklch_();
  static bool calibrated_frame_valid_for_duv_(const CalibratedFrame &frame);
  bool calibrated_xyz_valid_for_oklab_(const CalibratedFrame &frame) const;
  static bool cie1960_uv_from_xyz_(const CalibratedFrame &frame, Cie1960UcsPoint *point);
  static bool cie1960_uv_from_xy_(float x, float y, Cie1960UcsPoint *point);
  static bool planckian_locus_uv_from_cct_(float cct, Cie1960UcsPoint *point);
  bool handle_finished_calibrated_frame_command_(size_t step_index, TransportResult result);
  void schedule_frame_timed_readout_(uint32_t wait_ms);
  bool retry_timed_frame_readout_(const char *reason);
  void fail_timed_frame_readout_(const char *reason, RawFrameStatus raw_status, CalibratedFrameStatus calibrated_status,
                                 CalculatedDuvStatus duv_status, DerivedColorStatus derived_status);
  uint32_t calculate_frame_timed_readout_wait_ms_() const;
  uint8_t measurement_integration_time_() const;
  static bool raw_frame_empty_(const RawFrame &frame);
  uint8_t single_bank_probe_integration_time_() const;
  uint8_t calculate_single_bank_probe_interval_() const;
  uint32_t calculate_single_bank_probe_timed_readout_wait_ms_() const;
  void handle_finished_manual_exposure_commands_(SequenceStatus status);
  void handle_finished_auto_exposure_candidate_commands_(SequenceStatus status);
  void handle_finished_diagnostic_command_(DiagnosticState state, TransportResult result);
  void handle_firmware_version_response_();
  void handle_device_temperature_response_();
  void handle_uart_byte_(uint8_t byte);
  void finish_response_line_();
  bool finish_response_line_with_terminal_(const char *terminal, TransportResult result);
  bool append_response_line_(const char *line);
  void complete_transport_(TransportResult result);
  void clear_transport_buffers_();
  void drain_uart_();
  const char *first_response_value_();
  void start_reset_pulse_(const char *reason);
  void release_reset_pulse_();
  static bool parse_device_temperature_(const char *text, int16_t *temperature_c, bool *invalid);
  static bool parse_unsigned_byte_(const char *text, uint8_t *value);
  static bool parse_raw_frame_(const char *text, RawFrame *frame);
  static bool parse_raw_frame_field_(const char **cursor, uint16_t *value, bool expect_separator);
  static bool consume_raw_frame_separator_(const char **cursor);
  static bool parse_unsigned_u16_(const char *begin, const char *end, uint16_t *value);
  static bool parse_unsigned_digits_(const char *begin, const char *end, uint8_t base, uint16_t *value);
  static bool parse_calibrated_xyz_(const char *text, CalibratedFrame *frame);
  static bool parse_calibrated_xyz_field_(const char **cursor, float *value, bool expect_separator);
  static bool consume_calibrated_frame_separator_(const char **cursor);
  static bool parse_calibrated_value_(const char *text, float *value);
  static bool parse_finite_float_(const char *begin, const char *end, float *value);
  static const char *trim_left_(const char *text);
  static const char *trim_right_(const char *begin, const char *end);
  static bool is_space_(char value);
  static int8_t digit_value_(char value, uint8_t base);
  static const char *transport_result_to_string_(TransportResult result);
  static const char *diagnostic_state_to_string_(DiagnosticState state);
  static const char *sequence_status_to_string_(SequenceStatus status);
  static const char *frame_state_to_string_(FrameState state);
  static const char *raw_frame_status_to_string_(RawFrameStatus status);
  static const char *single_bank_probe_state_to_string_(SingleBankProbeState state);
  static const char *single_bank_probe_status_to_string_(SingleBankProbeStatus status);
  static const char *calibrated_frame_status_to_string_(CalibratedFrameStatus status);
  static const char *calculated_duv_status_to_string_(CalculatedDuvStatus status);
  static const char *derived_color_status_to_string_(DerivedColorStatus status);
  static const char *exposure_assessment_status_to_string_(ExposureAssessmentStatus status);
  static const char *exposure_assessment_guidance_to_string_(ExposureAssessmentGuidance guidance);
  static const char *auto_exposure_policy_action_to_string_(AutoExposurePolicyAction action);
  static float auto_exposure_gain_multiplier_(AS7261Gain gain);
  static AS7261Gain next_higher_auto_exposure_gain_(AS7261Gain gain, bool *stepped);
  static AS7261Gain next_lower_auto_exposure_gain_(AS7261Gain gain, bool *stepped);
  static uint8_t gain_to_at_value_(AS7261Gain gain);

  InternalGPIOPin *int_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  TransportState transport_state_{TransportState::IDLE};
  TransportResult last_transport_result_{TransportResult::NONE};
  DiagnosticState diagnostic_state_{DiagnosticState::IDLE};
  SequenceStatus sequence_status_{SequenceStatus::IDLE};
  ManualExposureCommandState manual_exposure_state_{ManualExposureCommandState::NOT_CONFIGURED};
  AutoExposureCommandState auto_exposure_state_{AutoExposureCommandState::NOT_CONFIGURED};
  AutoExposureConvergenceState auto_exposure_convergence_state_{AutoExposureConvergenceState::IDLE};
  uint8_t auto_exposure_convergence_attempts_{0};
  SequenceOwner sequence_owner_{SequenceOwner::NONE};
  FrameState frame_state_{FrameState::IDLE};
  SingleBankProbeState single_bank_probe_state_{SingleBankProbeState::IDLE};
  CommandSequenceStep command_sequence_[COMMAND_SEQUENCE_LENGTH]{};
  size_t sequence_step_count_{0};
  size_t sequence_step_index_{0};
  uint32_t transport_started_millis_{0};
  uint32_t transport_timeout_ms_{COMMAND_TIMEOUT_MS};
  uint32_t reset_started_millis_{0};
  char command_buffer_[COMMAND_BUFFER_LENGTH]{};
  char line_buffer_[LINE_BUFFER_LENGTH]{};
  size_t line_length_{0};
  char response_buffer_[RESPONSE_BUFFER_LENGTH]{};
  char manual_exposure_commands_[2][COMMAND_BUFFER_LENGTH]{};
  char auto_exposure_commands_[2][COMMAND_BUFFER_LENGTH]{};
  char single_bank_probe_mode_command_[COMMAND_BUFFER_LENGTH]{};
  char single_bank_probe_interval_command_[COMMAND_BUFFER_LENGTH]{};
  size_t response_length_{0};
  uint32_t frame_wait_started_millis_{0};
  uint32_t frame_timed_readout_wait_ms_{0};
  uint8_t frame_readout_attempt_{0};
  uint32_t single_bank_probe_wait_started_millis_{0};
  uint32_t single_bank_probe_timed_readout_wait_ms_{0};
  uint8_t response_line_count_{0};
  uint8_t temp_atxyzc_raw_bytes_[ATXYZC_RAW_DEBUG_BUFFER_LENGTH]{};
  size_t temp_atxyzc_raw_length_{0};
  bool temp_atxyzc_raw_overflow_{false};
  RawFrame raw_frame_{};
  RawFrameStatus raw_frame_status_{RawFrameStatus::INVALID};
  RawFrame single_bank_probe_raw_frame_{};
  SingleBankProbeStatus single_bank_probe_status_{SingleBankProbeStatus::INVALID};
  CalibratedFrame calibrated_frame_{};
  CalibratedFrameStatus calibrated_frame_status_{CalibratedFrameStatus::INVALID};
  CalculatedDuvFrame calculated_duv_frame_{};
  CalculatedDuvStatus calculated_duv_status_{CalculatedDuvStatus::INVALID};
  DerivedColorFrame derived_color_frame_{};
  DerivedColorStatus derived_color_status_{DerivedColorStatus::INVALID};
  float vendor_duv_cie1976_{0.0f};
  bool vendor_duv_cie1976_valid_{false};
  CalibratedFrame precision_frames_[PRECISION_FRAME_COUNT]{};
  size_t precision_frame_count_{0};
  PrecisionCollectionStatus precision_collection_status_{PrecisionCollectionStatus::IDLE};
  ExposureAssessment exposure_assessment_{};
  AutoExposurePolicy auto_exposure_policy_{};
  DarkChannelRecoveryState dark_channel_recovery_state_{};
  int16_t last_device_temperature_c_{0};
  bool device_temperature_valid_{false};
  bool device_temperature_invalid_{false};
  bool device_temperature_unsafe_{false};
  bool active_measurement_manual_{false};
  bool active_measurement_counted_{false};
  uint32_t completed_measurement_count_{0};
  bool precision_mode_{false};
  float oklab_reference_illuminance_{DEFAULT_OKLAB_REFERENCE_ILLUMINANCE_LX};
  bool manual_exposure_{false};
  AS7261Gain gain_{AS7261_GAIN_16X};
  uint8_t integration_time_{0};
};

#ifdef USE_BUTTON
class AS7261MeasureButton : public button::Button, public Parented<AS7261Component> {
 public:
  AS7261MeasureButton() = default;

 protected:
  void press_action() override;
};
#endif

}  // namespace esphome::as7261
