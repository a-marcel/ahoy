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

        Inverter<> *iv = this->parent_->get_inverter(this->inverter_id_);

        uint16_t *limit;
        limit = (uint16_t*)(&value);


        this->parent_->sendLimitPacket(iv->radioId.u64, limit);

        this->publish_state(value);
    }

}  // namespace copy
}  // namespace hoymiles