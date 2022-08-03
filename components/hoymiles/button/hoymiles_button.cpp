#include "hoymiles_button.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hoymiles {

    static const char *const TAG = "hoymiles.button";

    void HoymilesButton::dump_config() { 
        LOG_BUTTON("", "Hoymiles Button", this); 
    }

    void HoymilesButton::press_action() {
        ESP_LOGI(TAG, "Restart ...");

        Inverter<> *iv = this->parent_->get_inverter(this->inviter_id_);

        if (this->_type == HoymilesButtonTypes::RESTART) {
            this->parent_->sendRestartPacket(iv->radioId.u64);
        }
        else if (this->_type == HoymilesButtonTypes::CLEAN_STATE) {
            this->parent_->sendCleanStatePacket(iv->radioId.u64);
        }

    }

}  // namespace hoymiles
}  // namespace esphome