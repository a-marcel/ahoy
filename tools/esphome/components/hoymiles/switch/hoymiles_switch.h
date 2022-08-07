#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../hoymiles.h"

namespace esphome {
namespace hoymiles {

class HoymilesSwitch : public switch_::Switch, public Component, public hoymiles::HoymilesDevice {
    public:
        void setup() override;
        void write_state(bool state) override;
        void dump_config() override;

        void set_inverter_id(char *inverterId) { this->inverter_id_ = inverterId; }
    protected:
        char *inverter_id_;
};

} //namespace hoymiles
} //namespace esphome