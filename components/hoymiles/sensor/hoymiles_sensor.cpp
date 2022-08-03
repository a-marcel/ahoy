#include "esphome/core/log.h"
#include "hoymiles_sensor.h"

namespace esphome {
namespace hoymiles {

    static const char *TAG = "hoymiles.sensor";

    void HoymilesSensor::setup() {
        ESP_LOGI(TAG, "Sensor %s, %s", this->inviter_id_, this->parent_);
    }

    void HoymilesSensor::update() {
        if (this->parent_ == NULL) {
            return;
        }

        Inverter<> *iv = this->parent_->get_inverter(this->inviter_id_);

        char topic[30], val[10];

        if (iv->isAvailable(this->parent_->getTimestamp())) {
            for(uint8_t i = 0; i < iv->listLen; i++) {

                if (0.0f != iv->getValue(i)) {     

                    snprintf(topic, 30, "%s/ch%d/%s", iv->name, iv->assign[i].ch, iv->getFieldName(i));
                                        snprintf(val, 10, "%.3f %s", iv->getValue(i), iv->getUnit(i));
                                        DPRINTLN(DBG_INFO, String(topic) + ": " + String(val));


                    if (iv->assign[i].ch == 0) {
                        if (String(fields[iv->assign[i].fieldId]).compareTo(String("Temp")) == 0 && this->general_.temperature_sensor_ != nullptr) {
                            this->general_.temperature_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("U_AC")) == 0 && this->grid_.voltage_sensor_ != nullptr) {
                            this->grid_.voltage_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("I_AC")) == 0 && this->grid_.current_sensor_ != nullptr) {
                            this->grid_.current_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("P_AC")) == 0 && this->grid_.power_sensor_ != nullptr) {
                            this->grid_.power_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("P_DC")) == 0 && this->general_.power_sensor_ != nullptr) {
                            this->general_.power_sensor_->publish_state(iv->getValue(i));
                        }                        
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("Freq")) == 0 && this->grid_.frequency_sensor_ != nullptr) {
                            this->grid_.frequency_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("YieldDay")) == 0 && this->general_.yield_day_sensor_ != nullptr) {
                            this->general_.yield_day_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("YieldTotal")) == 0 && this->general_.yield_total_sensor_ != nullptr) {
                            this->general_.yield_total_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("Effiency")) == 0 && this->general_.efficiency_sensor_ != nullptr) {
                            this->general_.efficiency_sensor_->publish_state(iv->getValue(i));
                        }


                    } else if (iv->assign[i].ch > 0) {

                        int dc_channel = (iv->assign[i].ch-1);


                        if (String(fields[iv->assign[i].fieldId]).compareTo(String("P_DC")) == 0 && this->dc_channels_[dc_channel].power_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].power_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("I_DC")) == 0 && this->dc_channels_[dc_channel].current_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].current_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("U_DC")) == 0 && this->dc_channels_[dc_channel].voltage_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].voltage_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("Irradiation")) == 0 && this->dc_channels_[dc_channel].irradiation_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].irradiation_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("YieldDay")) == 0 && this->dc_channels_[dc_channel].yield_day_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].yield_day_sensor_->publish_state(iv->getValue(i));
                        }
                        else if (String(fields[iv->assign[i].fieldId]).compareTo(String("YieldTotal")) == 0 && this->dc_channels_[dc_channel].yield_total_sensor_ != nullptr) {
                            this->dc_channels_[dc_channel].yield_total_sensor_->publish_state(iv->getValue(i));
                        }
                    }
                }
            }
        }
    
    }

    void HoymilesSensor::loop() {

    }

    void HoymilesSensor::dump_config() {
        // ESP_LOGCONFIG(TAG, "Empty Hoymiles sensor");
    }

}  // namespace hoymiles.sensor
}  // namespace esphome