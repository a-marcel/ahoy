#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace hoymiles {
    class HoymilesLimiter : public number::Number, public Component {
        public:
    };

}
}