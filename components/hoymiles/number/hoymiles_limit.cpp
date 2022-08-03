#include "hoymiles_limit.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hoymiles {

static const char *const TAG = "hoymiles.limit";

    void HoymilesLimit::setup() {
       
    }

    void HoymilesLimit::dump_config() { LOG_NUMBER("", "Hoymiles Limit", this); }

    void HoymilesLimit::control(float value) {
        ESP_LOGI(TAG, "Limit %f", value);

        Inverter<> *iv = this->parent_->get_inverter(this->inviter_id_);

        this->parent_->sendLimitPacket(iv->radioId.u64, value);

        this->publish_state(value);
    }

}  // namespace copy
}  // namespace hoymiles