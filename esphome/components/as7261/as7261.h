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
  const char *gain_to_string_() const;

  InternalGPIOPin *int_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  bool precision_mode_{false};
  bool manual_exposure_{false};
  AS7261Gain gain_{AS7261_GAIN_16X};
  uint8_t integration_time_{0};
};

}  // namespace esphome::as7261
