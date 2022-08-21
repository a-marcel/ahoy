#include "esphome/core/log.h"
#include "hoymiles_sensor.h"

namespace esphome {
namespace hoymiles {

    static const char *TAG = "hoymiles.sensor";

    void HoymilesSensor::setup() {
        ESP_LOGI(TAG, "Sensor %s", this->inverter_id_);
    }

    void HoymilesSensor::update() {
        if (this->parent_ == NULL) {
            return;
        }

        Inverter<> *iv = this->parent_->get_inverter(this->inverter_id_);

        char topic[30], val[10];

        if (NULL != iv && iv->isAvailable(this->parent_->getTimestamp())) {
        // if (NULL != iv && iv->isAvailable(this->parent_->getTimestamp())) {
            for(uint8_t i = 0; i < iv->listLen; i++) {

                if (0.0f != iv->getValue(i)) {     

                    snprintf(topic, 30, "%s/ch%d/%s", iv->name, iv->assign[i].ch, String(iv->getFieldName(i)));
                    snprintf(val, 10, "%.3f %s", iv->getValue(i), String(iv->getUnit(i)));
                    ESP_LOGI(TAG, "%s %s: %s", String(iv->name), String(iv->assign[i].fieldId), String(val));


                    if (iv->assign[i].ch == 0) {
                        if (iv->assign[i].fieldId == FLD_T && this->general_.temperature_sensor_ != nullptr) {
                            this->general_.temperature_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_UAC && this->grid_.voltage_sensor_ != nullptr) {
                            this->grid_.voltage_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_IAC && this->grid_.current_sensor_ != nullptr) {
                            this->grid_.current_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_PAC && this->grid_.power_sensor_ != nullptr) {
                            this->grid_.power_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_PDC && this->general_.power_sensor_ != nullptr) {
                            this->general_.power_sensor_->publish_state(iv->getValue(i));
                        }                        
                        else if (iv->assign[i].fieldId == FLD_F && this->grid_.frequency_sensor_ != nullptr) {
                            this->grid_.frequency_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_YD && this->general_.yield_day_sensor_ != nullptr) {
                            this->general_.yield_day_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_YT && this->general_.yield_total_sensor_ != nullptr) {
                            this->general_.yield_total_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_EFF && this->general_.efficiency_sensor_ != nullptr) {
                            this->general_.efficiency_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_PCT && this->grid_.power_factor_sensor_ != nullptr) {
                            this->grid_.power_factor_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_PRA && this->grid_.reactive_power_sensor_ != nullptr) {
                            this->grid_.reactive_power_sensor_->publish_state(iv->getValue(i));
                        }


                    } else if (iv->assign[i].ch > 0) {

                        int dc_channel = (iv->assign[i].ch-1);


                        if (iv->assign[i].fieldId == FLD_PDC && this->dc_channels_[dc_channel].power_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].power_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_IDC && this->dc_channels_[dc_channel].current_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].current_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_UDC && this->dc_channels_[dc_channel].voltage_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].voltage_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_IRR && this->dc_channels_[dc_channel].irradiation_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].irradiation_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_YD && this->dc_channels_[dc_channel].yield_day_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].yield_day_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (iv->assign[i].fieldId == FLD_YT && this->dc_channels_[dc_channel].yield_total_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].yield_total_sensor_->publish_state(iv->getValue(i));
                        }
                    }
                }
            }
        } else {
            ESP_LOGI(TAG, "Data not available for %lld", this->parent_->getTimestamp());
        }

        // Debug sensors
        if (this->debug_.debug_rx_failed_sensor_ != nullptr) {
            this->debug_.debug_rx_failed_sensor_->publish_state(this->parent_->getDebugRxFailed());
        }
        if (this->debug_.debug_rx_success_sensor_ != nullptr) {
            this->debug_.debug_rx_success_sensor_->publish_state(this->parent_->getDebugRxSuccess());
        }
        if (this->debug_.debug_rx_frames_count_sensor_ != nullptr) {
            this->debug_.debug_rx_frames_count_sensor_->publish_state(this->parent_->getDebugRxFailed());
        }
        if (this->debug_.debug_send_count_sensor_ != nullptr) {
            this->debug_.debug_send_count_sensor_->publish_state(this->parent_->getDebugSendCount());
        }
    
    }

    void HoymilesSensor::loop() {

    }

    void HoymilesSensor::dump_config() {
        // ESP_LOGCONFIG(TAG, "Empty Hoymiles sensor");
    }

}  // namespace hoymiles.sensor
}  // namespace esphome