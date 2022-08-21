#include "Arduino.h"
#include "hoymiles_component.h"
#include "esphome/core/log.h"
#include "esphome/components/time/real_time_clock.h"

namespace esphome {
namespace hoymiles {
    static const char *const TAG = "hoymiles.esp32";

    enum control_commands { 
        TURN_ON = 0x00, 
        TURN_OFF = 0x01, 
        RESTART = 0x02,
        POWER_LIMIT = 0x0b,
        CLEAN_STATE_ALARM = 0x14
    }; 

    uint32_t mRxTicker;
    // time_t mTimestamp;
    uint32_t mTicker;
    uint16_t mSendTicker;
    uint16_t mSendInterval;
    uint8_t mLastPacketId;
    uint32_t mUptimeTicker;
    uint16_t mUptimeInterval;
    uint32_t mUptimeSecs;
    uint8_t mSendLastIvId;
    bool mMqttNewDataAvail;

    uint32_t mUpdateTicker;
    uint16_t mUpdateInterval;

    // esphome custom interval
    uint8_t mNtpUpdateCounter;
    uint16_t mNtpUpdateInterval;

    HmSystemType *mSys;
    // config_t *mConfig = new config_t();

    invPayload_t mPayload[MAX_NUM_INVERTERS];

    static ICACHE_RAM_ATTR void handleIntr(void);
    static bool checkTicker(uint32_t *ticker, uint32_t interval);
    // static void processPayload(bool retransmit);
    static bool buildPayload(uint8_t id);

    static const char* C_dumpBuf(uint8_t buf[], uint8_t len);

    void HoymilesComponent::setup() {
        ESP_LOGD(TAG, "Setting up Hoymiles Component");
        uint16_t modPwr[4];

        memset(mPayload, 0, (MAX_NUM_INVERTERS * sizeof(invPayload_t)));

        mSys = new HmSystemType();
        memset(&mConfig, 0, sizeof(config_t));

        // nrf24
        mConfig.sendInterval      = SEND_INTERVAL;
        mConfig.maxRetransPerPyld = DEF_MAX_RETRANS_PER_PYLD;
        mConfig.pinCs             = DEF_RF24_CS_PIN;
        mConfig.pinCe             = DEF_RF24_CE_PIN;
        mConfig.pinIrq            = DEF_RF24_IRQ_PIN;
        mConfig.amplifierPower    = DEF_AMPLIFIERPOWER & 0x03;


        mConfig.pinCs = this->cs_pin_->get_pin();
        mConfig.pinCe = this->ce_pin_->get_pin();
        mConfig.pinIrq = this->irq_pin_->get_pin();
        mConfig.amplifierPower = this->amplifier_power_;
        

        mConfig.serialDebug = true;
        
        for(int i = 0; i < MAX_NUM_INVERTERS; i++) {
            if (this->inverters_[i].identification_ != NULL) {

                mSys->addInverter(this->inverters_[i].identification_, this->inverters_[i].serial_number_, modPwr);

                char buffer [50];
                sprintf(buffer, "0x%llx", this->inverters_[i].serial_number_);
                ESP_LOGI(TAG, "Adding Inverter: %s ( %s )", this->inverters_[i].identification_, buffer);
            }
        }
        mSys->setup(&mConfig);

        attachInterrupt(digitalPinToInterrupt(this->irq_pin_->get_pin()), handleIntr, FALLING);

        mRxSuccess    = 0;

        mRxTicker = 0;
        // mTimestamp = 0;

        mNtpUpdateCounter = 0;
        mNtpUpdateInterval = 120; // seconds

        ESP_LOGI(TAG, "Setting inital timestamp %lu", this->getTimestamp());

        mSendTicker     = 0xffff;
        mSendInterval   = this->send_interval_;

        // mUpdateTicker   = 0xffff;
        // mUpdateInterval = update_interval_min_ / 1000;

        mLastPacketId = 0x00;

        mUptimeSecs     = 0;
        mUptimeTicker   = 0xffffffff;
        mUptimeInterval = 1000;

        mSendLastIvId = 0;

        mMqttNewDataAvail = false;
        mRxFailed     = 0;
        mRxSuccess    = 0;
        mFrameCnt     = 0;

        // ESP_LOGI(TAG, "Update Interval %s", String(this->update_interval_min_));
    }

    void HoymilesComponent::loop() {
        if (checkTicker(&mUptimeTicker, mUptimeInterval)) {
            // mTimestamp++;
            mUptimeSecs++;

            time_t mTimestamp = this->getTimestamp();


            // mTimestamp++;

            mNtpUpdateCounter++;

            if (mNtpUpdateCounter > mNtpUpdateInterval || mTimestamp == 0 || mTimestamp < 100000) {
                mTimestamp  = this->getRtcTimestamp();
                ESP_LOGI(TAG, "Sync timestamp with NTP: %lu", mTimestamp);

                mNtpUpdateCounter = 0;
            } else {
                mTimestamp++;
            }
            this->setTimestamp(mTimestamp);
        }

        mSys->Radio.loop();
        if (checkTicker(&mRxTicker, 5)) {
            bool rxRdy = mSys->Radio.switchRxCh();

            if(!mSys->BufCtrl.empty()) {
                uint8_t len;
                packet_t *p = mSys->BufCtrl.getBack();
                //mSys->Radio.dumpBuf("RAW ", p->packet, MAX_RF_PAYLOAD_SIZE);

                if(mSys->Radio.checkPaketCrc(p->packet, &len, p->rxCh)) {
                    // if(mSerialDebug) {
                        // ESP_LOGD(TAG, "Received %i bytes channel %i: ", len, p->rxCh);
                        ESP_LOGD(TAG, "RX %iB Ch%i | %s", len, p->rxCh, C_dumpBuf(p->packet, len));

                        // mSys->Radio.dumpBuf(NULL, p->packet, len);
                        
                    // }                
                    // process buffer only on first occurrence
                    mFrameCnt++;

                    if((0 != len)) {
                        // uint8_t *packetId = &p->packet[9];
                        //DPRINTLN("CMD " + String(*cmd, HEX));
                        // mSys->Radio.dumpBuf("Payload ", p->packet, len);

                        Inverter<> *iv = mSys->findInverter(&p->packet[1]);
                        if(NULL != iv&& p->packet[0] == (TX_REQ_INFO + 0x80)) { // response from get all information command
                            DPRINTLN(DBG_DEBUG, F("Response from info request received"));
                            uint8_t *pid = &p->packet[9];
                            if (*pid == 0x00) {
                                ESP_LOGD(TAG, "fragment number zero received and ignored");
                            } else {
                                if((*pid & 0x7F) < 5) {
                                    memcpy(mPayload[iv->id].data[(*pid & 0x7F) - 1], &p->packet[10], len-11);
                                    mPayload[iv->id].len[(*pid & 0x7F) - 1] = len-11;
                                }

                                if((*pid & 0x80) == 0x80) { // Last packet
                                    if((*pid & 0x7f) > mPayload[iv->id].maxPackId)
                                        mPayload[iv->id].maxPackId = (*pid & 0x7f);

                                        if(*pid > 0x81)
                                            mLastPacketId = *pid;
                                }
                            }

                            switch (mSys->InfoCmd){
                                case InverterDevInform_Simple:
                                {
                                    DPRINT(DBG_INFO, "Response from inform simple\n");
                                    mSys->InfoCmd = RealTimeRunData_Debug; // Set back to default
                                    break;
                                }
                                case InverterDevInform_All:
                                {
                                    DPRINT(DBG_INFO, "Response from inform all\n");
                                    mSys->InfoCmd = RealTimeRunData_Debug; // Set back to default
                                    break;
                                }
                                case GetLossRate:
                                {
                                    DPRINT(DBG_INFO, "Response from get loss rate\n");
                                    mSys->InfoCmd = RealTimeRunData_Debug; // Set back to default
                                    break;
                                }
                                case AlarmData:
                                {
                                    DPRINT(DBG_INFO, "Response from AlarmData\n");
                                    mSys->InfoCmd = RealTimeRunData_Debug; // Set back to default
                                    break;
                                }
                                case AlarmUpdate:
                                {
                                    DPRINT(DBG_INFO, "Response from AlarmUpdate\n");
                                    mSys->InfoCmd = RealTimeRunData_Debug; // Set back to default
                                    break;
                                }
                                case RealTimeRunData_Debug:
                                {
                                    break;
                                }
                            }
                        }
                        if(NULL != iv && p->packet[0] == (TX_REQ_DEVCONTROL + 0x80)) { // response from dev control command
                            ESP_LOGD(TAG, "Response from devcontrol request received");
                            iv->devControlRequest = false; 
                            switch (p->packet[12]){
                                case ActivePowerContr:
                                    if (iv->devControlCmd >= ActivePowerContr && iv->devControlCmd <= PFSet){ // ok inverter accepted the set point copy it to dtu eeprom
                                        if (iv->powerLimit[1]>0){ // User want to have it persistent
                                            // mEep->write(ADDR_INV_PWR_LIM + iv->id * 2,iv->powerLimit[0]);
                                            // updateCrc();
                                            // mEep->commit();
                                            ESP_LOGI(TAG, "Inverter has accepted power limit set point, written to dtu eeprom");
                                        } else {
                                            ESP_LOGI(TAG, "Inverter has accepted power limit set point");
                                        }
                                        iv->devControlCmd = Init;
                                    }
                                    break;
                                default:
                                    if (iv->devControlCmd == ActivePowerContr){
                                        //case inverter did not accept the sent limit; set back to last stored limit
                                        // mEep->read(ADDR_INV_PWR_LIM + iv->id * 2, (uint16_t *)&(iv->powerLimit[0]));
                                        // DPRINTLN(DBG_INFO, F("Inverter has not accepted power limit set point"));    
                                    }
                                    iv->devControlCmd = Init;
                                    break;
                            }
                        }                     
                    }
                }
                mSys->BufCtrl.popBack();
            }
            yield();

            if (rxRdy) {
                processPayload(true, mSys->InfoCmd);
            }
        }

        if(checkTicker(&mTicker, 1000)) {
            if(++mSendTicker >= mSendInterval) {
                mSendTicker = 0;

                if(0 != this->getTimestamp()) {
                    if(!mSys->BufCtrl.empty()) {
                        // if(mSerialDebug)
                            ESP_LOGV(TAG, "recbuf not empty! # %i", mSys->BufCtrl.getFill());
                    }

                    int8_t maxLoop = MAX_NUM_INVERTERS;
                    Inverter<> *iv = mSys->getInverterByPos(mSendLastIvId);
                    do {
                        if(NULL != iv)
                            mPayload[iv->id].requested = false;
                        mSendLastIvId = ((MAX_NUM_INVERTERS-1) == mSendLastIvId) ? 0 : mSendLastIvId + 1;
                        iv = mSys->getInverterByPos(mSendLastIvId);

                    } while((NULL == iv) && ((maxLoop--) > 0));

                    if(NULL != iv) {
                        if(!mPayload[iv->id].complete)
                            processPayload(false, mSys->InfoCmd);

                        if(!mPayload[iv->id].complete) {
                            mRxFailed++;
                            // if(mSerialDebug) {
                                ESP_LOGV(TAG, "Inverter #%s no Payload received! (retransmits: %i)", String(iv->id),  mPayload[iv->id].retransmits);
                                // DPRINTLN(String(F("no Payload received! (retransmits: ")) + String(mPayload[iv->id].retransmits) + ")");
                            // }
                        }

                        // reset payload data
                        resetPayload(iv);

                        yield();
                        ESP_LOGV(TAG, "Setting timestamp: %i", this->getTimestamp());

                        // if(mSerialDebug) {
                            char buffer [50];
                            sprintf(buffer, "val = 0x%llx\n", iv->serial.u64);
                            ESP_LOGV(TAG, "Requesting Inverter SN %s", buffer);
                        // }

                        if(iv->devControlRequest && iv->powerLimit[0] > 0) { // prevent to "switch off") {
                            // if(mSerialDebug)
                                ESP_LOGV(TAG, "Devcontrol request %s power limit %s", String(iv->devControlCmd), String(iv->powerLimit));
                                // DPRINTLN(DBG_INFO, F("Devcontrol request ") + String(iv->devControlCmd) + F(" power limit ") + String(iv->powerLimit));

                            // ToDo: Only set Request to false if succesful executed
                            mSys->Radio.sendControlPacket(iv->radioId.u64,iv->devControlCmd ,iv->powerLimit);
                        } else {
                            mSys->Radio.sendTimePacket(iv->radioId.u64, mSys->InfoCmd, mPayload[iv->id].ts,iv->alarmMesIndex);

                            mRxTicker = 0;
                        }
                    }
                }
                else {
                    ESP_LOGE(TAG, "time not set, can't request inverter!");
                }
                yield();
            }

            // if(++mUpdateTicker >= mUpdateInterval) {
            //     mUpdateTicker = 0;
            //     HoymilesSensor::update();
            // }
        }        
    }

    void HoymilesComponent::sendTurnOnOffPacket(uint64_t invId, bool state) {
        ESP_LOGI(TAG, "Sending OnOff: %d", state);

        if (state) {
            mSys->Radio.sendControlPacket(invId, control_commands::TURN_ON, NULL);
        } else {
            mSys->Radio.sendControlPacket(invId, control_commands::TURN_OFF, NULL);
        }
    }

    void HoymilesComponent::sendRestartPacket(uint64_t invId) {
        ESP_LOGI(TAG, "Sending Restart");

        mSys->Radio.sendControlPacket(invId, control_commands::RESTART, NULL);
    }

    void HoymilesComponent::sendCleanStatePacket(uint64_t invId) {
        ESP_LOGI(TAG, "Sending Command to clean alarm and state");

        mSys->Radio.sendControlPacket(invId, control_commands::CLEAN_STATE_ALARM, NULL);
    }


    void HoymilesComponent::sendLimitPacket(uint64_t invId, uint16_t *limit) {
        ESP_LOGI(TAG, "Sending Limitation: %i", limit);

        mSys->Radio.sendControlPacket(invId, control_commands::POWER_LIMIT, limit);

        // mSys->Radio.sendPowerLimitPacket(invId, limit, true);
    }



    void HoymilesComponent::dump_config() {
    }

    Inverter<> * HoymilesComponent::get_inverter(char *identification) {
        if (mSys != NULL) {
            for (uint8_t id = 0; id < mSys->getNumInverters(); id++) {
                Inverter<> *iv = mSys->getInverterByPos(id);

                if (strcmp(iv->name, identification) == 0) {
                    // ESP_LOGI(TAG, "Get inverter %s", iv->name);
                    return iv;
                }
            }
        }
        ESP_LOGW(TAG, "Inverter with identification %s not found", identification);
        return NULL;
    }

    bool buildPayload(uint8_t id) {
        //DPRINTLN("Payload");
        uint16_t crc = 0xffff, crcRcv = 0x0000;
        if(mPayload[id].maxPackId > MAX_PAYLOAD_ENTRIES)
            mPayload[id].maxPackId = MAX_PAYLOAD_ENTRIES;

        for(uint8_t i = 0; i < mPayload[id].maxPackId; i ++) {
            if(mPayload[id].len[i] > 0) {
                if(i == (mPayload[id].maxPackId-1)) {
                    crc = Hoymiles::crc16(mPayload[id].data[i], mPayload[id].len[i] - 2, crc);
                    crcRcv = (mPayload[id].data[i][mPayload[id].len[i] - 2] << 8)
                        | (mPayload[id].data[i][mPayload[id].len[i] - 1]);
                }
                else
                    crc = Hoymiles::crc16(mPayload[id].data[i], mPayload[id].len[i], crc);
            }
        }
        if (crc == crcRcv)
            return true;
        return false;
    }

    void HoymilesComponent::processPayload(bool retransmit) {
        processPayload(retransmit, RealTimeRunData_Debug);
    }

    void HoymilesComponent::processPayload(bool retransmit, uint8_t cmd) {
        // ESP_LOGD(TAG, "app::processPayload %i",mSys->getNumInverters() );
        for(uint8_t id = 0; id < mSys->getNumInverters(); id++) {
            Inverter<> *iv = mSys->getInverterByPos(id);
            if(NULL != iv) {
                if(!mPayload[iv->id].complete) {
                    if(!buildPayload(iv->id)) {
                        if(mPayload[iv->id].requested) {
                            if(retransmit) {
                                if(mPayload[iv->id].retransmits < mConfig.maxRetransPerPyld) {
                                    mPayload[iv->id].retransmits++;
                                    if(mPayload[iv->id].maxPackId != 0) {
                                        for(uint8_t i = 0; i < (mPayload[iv->id].maxPackId-1); i ++) {
                                            if(mPayload[iv->id].len[i] == 0) {
                                                ESP_LOGE(TAG, "while retrieving data: Frame %s missing: Request Retransmit", String(i+1));

                                                mSys->Radio.sendCmdPacket(iv->radioId.u64, TX_REQ_INFO, (SINGLE_FRAME+i), true);
                                                break; // only retransmit one frame per loop
                                            }
                                            yield();
                                        }
                                    }
                                    else {
                                        ESP_LOGE(TAG, "while retrieving data: last frame missing: Request Retransmit");
                                        if(0x00 != mLastPacketId)
                                            mSys->Radio.sendCmdPacket(iv->radioId.u64, TX_REQ_INFO, mLastPacketId, true);
                                        else
                                            mSys->Radio.sendTimePacket(iv->radioId.u64, mSys->InfoCmd, mPayload[iv->id].ts,iv->alarmMesIndex);

                                    }
                                    mSys->Radio.switchRxCh(100);
                                }
                            }
                        }
                    }
                    else {
                        mPayload[iv->id].complete = true;
                        
                        ESP_LOGD(TAG, "Last recieved TimeStamp: %i", mPayload[iv->id].ts);

                        iv->ts = mPayload[iv->id].ts;
                        uint8_t payload[128] = {0};
                        uint8_t offs = 0;
                        for(uint8_t i = 0; i < (mPayload[iv->id].maxPackId); i ++) {
                            memcpy(&payload[offs], mPayload[iv->id].data[i], (mPayload[iv->id].len[i]));
                            offs += (mPayload[iv->id].len[i]);
                            yield();
                        }
                        offs-=2;
                        
                        ESP_LOGI(TAG, "Payload (%s): %s", String(offs), C_dumpBuf(payload, offs));
                        // mSys->Radio.dumpBuf(NULL, payload, offs);

                        mRxSuccess++;
                        mSys->InfoCmd = RealTimeRunData_Debug; // On success set back to default

                        iv->getAssignment(cmd); // choose the parser

                        for(uint8_t i = 0; i < iv->listLen; i++) {
                            iv->addValue(i, payload, cmd);
                            yield();
                        }
                        iv->doCalculations(cmd);
                    }
                }
                yield();
            }
        }
    }

    void HoymilesComponent::resetPayload(Inverter<>* iv)
    {
        // reset payload data
        memset(mPayload[iv->id].len, 0, MAX_PAYLOAD_ENTRIES);
        mPayload[iv->id].retransmits = 0;
        mPayload[iv->id].maxPackId = 0;
        mPayload[iv->id].complete = false;
        mPayload[iv->id].requested = true;
        mPayload[iv->id].ts = this->getTimestamp();
    }

    uint32_t HoymilesComponent::getDebugSendCount() {
        return mSys->Radio.mSendCnt;
    }


    static const char* C_dumpBuf(uint8_t buf[], uint8_t len) {

        char buffer [len * 3];

        std::string result = "";


        for(uint8_t i = 0; i < len; i++) {
            result.append(format_hex_pretty(buf[i]));
            result.append(" ");
        }

        return result.c_str();
    }


    ICACHE_RAM_ATTR void handleIntr(void) {
        mSys->Radio.handleIntr();
    }

    bool checkTicker(uint32_t *ticker, uint32_t interval) {
        uint32_t mil = millis();
        if(mil >= *ticker) {
            *ticker = mil + interval;
            return true;
        }
        else if(mil < (*ticker - interval)) {
            *ticker = mil + interval;
            return true;
        }

        return false;
    }



}  // namespace uart
}  // namespace esphome
