#include "esphome/core/log.h"
#include "hoymiles_switch.h"

namespace esphome {
namespace hoymiles {

    static const char *TAG = "hoymiles.switch";

    void dumpBuf(const char *info, uint8_t buf[], uint8_t len) {
            //DPRINTLN(DBG_VERBOSE, F("hmRadio.h:dumpBuf"));
            if(NULL != info)
                DBGPRINT(String(info));
            for(uint8_t i = 0; i < len; i++) {
                DHEX(buf[i]);
                DBGPRINT(" ");
            }
            DBGPRINTLN("");
        }    

    void HoymilesSwitch::setup() {
        ESP_LOGI(TAG, "Switch %s", this->parent_);
    }    


    void HoymilesSwitch::write_state(bool state) {
        if (this->parent_ == NULL) {
            return;
        }

        Inverter<> *iv = this->parent_->get_inverter(this->inviter_id_);

        this->parent_->sendTurnOnOffPacket(iv->radioId.u64, state);

        publish_state(state);
    }

    void HoymilesSwitch::dump_config() {}

    

}  // namespace hoymiles.switch
}  // namespace esphome