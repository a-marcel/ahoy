#include "esphome/core/log.h"
#include "hoymiles_switch.h"

namespace esphome {
namespace hoymiles {

    static const char *TAG = "hoymiles.switch";

    void HoymilesSwitch::setup() {
        ESP_LOGI(TAG, "Switch %s", this->inverter_id_);

    }    


    void HoymilesSwitch::write_state(bool state) {
        if (this->parent_ == NULL) {
            return;
        }

        Inverter<> *iv = this->parent_->get_inverter(this->inverter_id_);

        this->parent_->sendTurnOnOffPacket(iv->radioId.u64, state);

        publish_state(state);
    }

    void HoymilesSwitch::dump_config() {}

    

}  // namespace hoymiles.switch
}  // namespace esphome