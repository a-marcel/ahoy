#include "esphome/core/log.h"
#include "hoymiles_text_sensor.h"
#include <time.h>

namespace esphome {
namespace hoymiles {

static const char *TAG = "hoymiles.text_sensor";

void HoymilesTextSensor::setup() {
  
}

void HoymilesTextSensor::update() {
    // this->general_.last_updated_sensor_->publish_state(time::ESPTime::from_epoch_local(this->parent_->getTimestamp()));
    if (this->parent_ == NULL) {
        return;
    }

    Inverter<> *iv = this->parent_->get_inverter(this->inverter_id_);

    if (NULL != iv &&iv->getLastTs() > 0) {
        struct tm * timeinfo;
        time_t t = iv->getLastTs();

        timeinfo = localtime (&t);

        std::string time_str = asctime(timeinfo);
        time_str.erase(time_str.end() - 1);

        this->publish_state(time_str);
    }

    // iv->alarmMesIndex;

}


void HoymilesTextSensor::dump_config() { 
    ESP_LOGCONFIG(TAG, "Empty text sensor");
}

}  // namespace hoymiles
}  // namespace esphome