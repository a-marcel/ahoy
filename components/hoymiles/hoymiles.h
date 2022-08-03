#pragma once

#include <vector>
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "hoymiles_component.h"

namespace esphome {
namespace hoymiles {

class HoymilesDevice {
    public:
        HoymilesDevice() = default;
        HoymilesDevice(HoymilesComponent *parent) : parent_(parent) {}

        void set_hoymiles_parent(HoymilesComponent *parent) { this->parent_ = parent; }
    protected:
        HoymilesComponent *parent_{nullptr};
};

}  // namespace uart
}  // namespace esphome