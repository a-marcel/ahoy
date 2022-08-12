#pragma once

#include "esphome/core/component.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "../hoymiles.h"

namespace esphome {
namespace hoymiles {

enum HoymilesTextSensorTypes {
  LAST_UPDATED = 1,
};

class HoymilesTextSensor : public text_sensor::TextSensor, public PollingComponent, public hoymiles::HoymilesDevice {
    public:
        void setup() override;
        void dump_config() override;
        void update() override;
        void set_inverter_id(char *inverterId) { this->inverter_id_ = inverterId; }
        void set_type(HoymilesTextSensorTypes type) { this->_type = type; }
    protected:
        char *inverter_id_;
        HoymilesTextSensorTypes _type;        
};

}  // namespace hoymilesText
}  // namespace esphome