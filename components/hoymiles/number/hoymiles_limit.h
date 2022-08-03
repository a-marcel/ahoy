#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "../hoymiles.h"

namespace esphome {
namespace hoymiles {

class HoymilesLimit : public number::Number, public Component, public hoymiles::HoymilesDevice {
    public:
//      void set_source(number::Number *source) { source_ = source; }
        void setup() override;
        void dump_config() override;
        float get_setup_priority() const override { return setup_priority::DATA; }

        void set_inverter_id(char *inverterId) { this->inviter_id_ = inverterId; }
        void set_max_power(float max_power) { this->max_power = max_power; }

    protected:
        void control(float value) override;

        number::Number *source_;
        char *inviter_id_;
        float max_power;
};

}  // namespace copy
}  // namespace hoymiles