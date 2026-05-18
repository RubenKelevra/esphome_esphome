#include "as7261.h"

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
    this->reset_pin_->digital_write(false);
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

void AS7261Component::update() { ESP_LOGD(TAG, "AS7261 runtime capture is intentionally stubbed in this scaffold"); }

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
