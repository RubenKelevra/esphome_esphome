#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::as7261 {

enum AS7261Gain {
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
  SUB_SENSOR(device_temperature)
  SUB_SENSOR(duv_cie1976)
  SUB_SENSOR(near_ir_percent)
  SUB_SENSOR(raw_clear)
  SUB_SENSOR(raw_dark)
  SUB_SENSOR(raw_near_ir)
  SUB_SENSOR(x)
  SUB_SENSOR(y)
  SUB_SENSOR(z)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(firmware_version)
#endif

 public:
  void setup() override;
  void dump_config() override;
  void loop() override;
  void update() override;

  void set_int_pin(InternalGPIOPin *int_pin) { this->int_pin_ = int_pin; }
  void set_reset_pin(GPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
  void set_precision_mode(bool precision_mode) { this->precision_mode_ = precision_mode; }
  void set_manual_exposure(AS7261Gain gain, uint8_t integration_time) {
    this->manual_exposure_ = true;
    this->gain_ = gain;
    this->integration_time_ = integration_time;
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

  struct CommandSequenceStep {
    const char *command;
    DiagnosticState diagnostic_state;
    SequenceFailurePolicy failure_policy;
  };

  static constexpr size_t COMMAND_BUFFER_LENGTH = 48;
  static constexpr size_t LINE_BUFFER_LENGTH = 96;
  static constexpr size_t RESPONSE_BUFFER_LENGTH = 192;
  static constexpr size_t COMMAND_SEQUENCE_LENGTH = 4;
  static constexpr uint32_t COMMAND_TIMEOUT_MS = 1000;
  static constexpr uint32_t RESET_PULSE_MS = 2;
  static constexpr float DEVICE_TEMPERATURE_UNSAFE_C = 76.5f;

  const char *gain_to_string_() const;
  bool transport_busy_() const { return this->transport_state_ != TransportState::IDLE; }
  bool sequence_active_() const { return this->sequence_status_ == SequenceStatus::RUNNING; }
  bool diagnostic_active_() const {
    return this->sequence_active_() || this->diagnostic_state_ != DiagnosticState::IDLE;
  }
  bool component_busy_() const { return this->transport_busy_() || this->sequence_active_(); }
  bool begin_at_command_(const char *command, uint32_t timeout_ms = COMMAND_TIMEOUT_MS);
  void poll_transport_();
  void poll_command_sequence_();
  bool start_command_sequence_(const CommandSequenceStep *steps, size_t step_count);
  bool start_current_sequence_step_();
  void handle_finished_sequence_step_();
  void finish_command_sequence_(SequenceStatus status);
  bool start_diagnostic_readout_();
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
  static bool parse_unsigned_digits_(const char *begin, const char *end, uint8_t base, uint16_t *value);
  static const char *trim_left_(const char *text);
  static const char *trim_right_(const char *begin, const char *end);
  static bool is_space_(char value);
  static int8_t digit_value_(char value, uint8_t base);
  static const char *transport_result_to_string_(TransportResult result);
  static const char *diagnostic_state_to_string_(DiagnosticState state);
  static const char *sequence_status_to_string_(SequenceStatus status);

  InternalGPIOPin *int_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  TransportState transport_state_{TransportState::IDLE};
  TransportResult last_transport_result_{TransportResult::NONE};
  DiagnosticState diagnostic_state_{DiagnosticState::IDLE};
  SequenceStatus sequence_status_{SequenceStatus::IDLE};
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
  size_t response_length_{0};
  uint8_t response_line_count_{0};
  int16_t last_device_temperature_c_{0};
  bool device_temperature_valid_{false};
  bool device_temperature_invalid_{false};
  bool device_temperature_unsafe_{false};
  bool precision_mode_{false};
  bool manual_exposure_{false};
  AS7261Gain gain_{AS7261_GAIN_16X};
  uint8_t integration_time_{0};
};

}  // namespace esphome::as7261
