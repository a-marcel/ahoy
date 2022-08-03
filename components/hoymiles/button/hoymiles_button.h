#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "../hoymiles.h"

namespace esphome {
namespace hoymiles {



enum HoymilesButtonTypes {
  RESTART = 1,
  CLEAN_STATE = 2
};

class HoymilesButton : public button::Button, public Component, public hoymiles::HoymilesDevice {
  public:
    void dump_config() override;
    void set_inverter_id(char *inverterId) { this->inviter_id_ = inverterId; }
    void set_type(HoymilesButtonTypes type) { this->_type = type; }

  protected:
    char *inviter_id_;
    HoymilesButtonTypes _type;

    void press_action() override;
};

}  // namespace hoymiles
}  // namespace esphome