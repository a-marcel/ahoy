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
        CLEAN_STATE_ALARM = 0x14
    }; 

    uint32_t mRxTicker;
    time_t mTimestamp;
    uint32_t mTicker;
    uint16_t mSendTicker;
    uint16_t mSendInterval;
    uint8_t mLastPacketId;
    uint32_t mUptimeTicker;
    uint16_t mUptimeInterval;
    uint32_t mUptimeSecs;
    uint8_t mSendLastIvId;
    bool mMqttNewDataAvail;
    uint32_t mRxFailed;
    uint32_t mRxSuccess;
    uint8_t mMaxRetransPerPyld;
    uint32_t mFrameCnt;

    uint32_t mUpdateTicker;
    uint16_t mUpdateInterval;

    HmSystemType *mSys;

    invPayload_t mPayload[MAX_NUM_INVERTERS];

    static ICACHE_RAM_ATTR void handleIntr(void);
    static bool checkTicker(uint32_t *ticker, uint32_t interval);
    static void processPayload(bool retransmit);
    static bool buildPayload(uint8_t id);

    static char* C_dumpBuf(uint8_t buf[], uint8_t len);

    // static String C_dumpBuf(uint8_t buf[], uint8_t len);
    static String C_DHEX(uint8_t b);
    static String C_DHEX(uint16_t b);
    static String C_DHEX(uint32_t b);

    void HoymilesComponent::setup() {
        ESP_LOGD(TAG, "Setting up Hoymiles Component");
        uint16_t modPwr[4];

        mSys = new HmSystemType();

        mSys->Radio.AmplifierPower = this->amplifier_power_;

        mSys->Radio.pinCs  = this->cs_pin_->get_pin();
        mSys->Radio.pinCe  = this->ce_pin_->get_pin();
        mSys->Radio.pinIrq = this->irq_pin_->get_pin();

        
        for(int i = 0; i < MAX_NUM_INVERTERS; i++) {
            if (this->inverters_[i].identification_ != NULL) {

                mSys->addInverter(this->inverters_[i].identification_, this->inverters_[i].serial_number_, modPwr);

                char buffer [50];
                sprintf(buffer, "0x%llx", this->inverters_[i].serial_number_);
                ESP_LOGI(TAG, "Adding Inverter: %s ( %s )", this->inverters_[i].identification_, buffer);
            }
        }
        mSys->setup();

        memset(mPayload, 0, (MAX_NUM_INVERTERS * sizeof(invPayload_t)));

        attachInterrupt(digitalPinToInterrupt(mSys->Radio.pinIrq), handleIntr, FALLING);

        mRxSuccess    = 0;

        mRxTicker = 0;
        mTimestamp = time_clock->now().timestamp;

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
        if(checkTicker(&mUptimeTicker, mUptimeInterval)) {
            // mTimestamp++;
            mUptimeSecs++;
            if(0 != mTimestamp)
                mTimestamp++;
            else {
                // if(!mApActive) {
                    mTimestamp  = time_clock->now().timestamp;

                    // DPRINTLN("[NTP]: " + getDateTimeStr(mTimestamp));
                // }
            }        
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
                        ESP_LOGD(TAG, "Received %i bytes channel %i: ", len, p->rxCh);
                        // ESP_LOGD(TAG, "Received %i bytes channel %i: %s", len, p->rxCh, C_dumpBuf(p->packet, len));
                        mSys->Radio.dumpBuf(NULL, p->packet, len);
                    // }                
                    // process buffer only on first occurrence
                    mFrameCnt++;

                    if((0 != len)) {
                        // uint8_t *packetId = &p->packet[9];
                        //DPRINTLN("CMD " + String(*cmd, HEX));
                        // mSys->Radio.dumpBuf("Payload ", p->packet, len);

                        Inverter<> *iv = mSys->findInverter(&p->packet[1]);
                        if(NULL != iv) {
                            uint8_t *pid = &p->packet[9];
                            if((*pid & 0x7F) < 5) {
                                memcpy(mPayload[iv->id].data[(*pid & 0x7F) - 1], &p->packet[10], len-11);
                                mPayload[iv->id].len[(*pid & 0x7F) - 1] = len-11;
                            }

                            if((*pid & 0x80) == 0x80) {
                                if((*pid & 0x7f) > mPayload[iv->id].maxPackId)
                                    mPayload[iv->id].maxPackId = (*pid & 0x7f);

                                    if(*pid > 0x81)
                                        mLastPacketId = *pid;
                            }
                        }
                    }
                }
                mSys->BufCtrl.popBack();
            }

            if (rxRdy) {
                processPayload(true);
            }
        }

        if(checkTicker(&mTicker, 1000)) {
            if(++mSendTicker >= mSendInterval) {
                mSendTicker = 0;

                if(0 != mTimestamp) {
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
                            processPayload(false);

                        if(!mPayload[iv->id].complete) {
                            mRxFailed++;
                            // if(mSerialDebug) {
                                ESP_LOGV(TAG, "Inverter #%s no Payload received! (retransmits: %i)", String(iv->id),  mPayload[iv->id].retransmits);
                                // DPRINTLN(String(F("no Payload received! (retransmits: ")) + String(mPayload[iv->id].retransmits) + ")");
                            // }
                        }

                        // reset payload data
                        memset(mPayload[iv->id].len, 0, MAX_PAYLOAD_ENTRIES);
                        mPayload[iv->id].retransmits = 0;
                        mPayload[iv->id].maxPackId = 0;
                        mPayload[iv->id].complete  = false;
                        mPayload[iv->id].requested = true;
                        mPayload[iv->id].ts = mTimestamp;

                        yield();

                        // if(mSerialDebug) {
                            char buffer [50];
                            sprintf(buffer, "val = 0x%llx\n", iv->serial.u64);
                            ESP_LOGV(TAG, "Requesting Inverter SN %s", buffer);
                        // }

                        mSys->Radio.sendTimePacket(iv->radioId.u64, mPayload[iv->id].ts);
                        mRxTicker = 0;
                    }
                }
                else {
                    ESP_LOGE(TAG, "time not set, can't request inverter!");
                }
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
            mSys->Radio.sendControlPacket(invId, control_commands::TURN_ON);
        } else {
            mSys->Radio.sendControlPacket(invId, control_commands::TURN_OFF);
        }
    }

    void HoymilesComponent::sendRestartPacket(uint64_t invId) {
        ESP_LOGI(TAG, "Sending Restart");

        mSys->Radio.sendControlPacket(invId, control_commands::RESTART);
    }

    void HoymilesComponent::sendCleanStatePacket(uint64_t invId) {
        ESP_LOGI(TAG, "Sending Command to clean alarm and state");

        mSys->Radio.sendControlPacket(invId, control_commands::CLEAN_STATE_ALARM);
    }


    void HoymilesComponent::sendLimitPacket(uint64_t invId, uint32_t limit) {
        ESP_LOGI(TAG, "Sending Limitation: %i", limit);

        mSys->Radio.sendPowerLimitPacket(invId, limit, true);
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


    void processPayload(bool retransmit) {
        ESP_LOGV(TAG, "app::processPayload");
        for(uint8_t id = 0; id < mSys->getNumInverters(); id++) {
            Inverter<> *iv = mSys->getInverterByPos(id);
            if(NULL != iv) {
                if(!mPayload[iv->id].complete) {
                    if(!buildPayload(iv->id)) {
                        if(mPayload[iv->id].requested) {
                            if(retransmit) {
                                if(mPayload[iv->id].retransmits < mMaxRetransPerPyld) {
                                    mPayload[iv->id].retransmits++;
                                    if(mPayload[iv->id].maxPackId != 0) {
                                        for(uint8_t i = 0; i < (mPayload[iv->id].maxPackId-1); i ++) {
                                            if(mPayload[iv->id].len[i] == 0) {
                                                ESP_LOGE(TAG, "while retrieving data: Frame %s missing: Request Retransmit", String(i+1));
                                                mSys->Radio.sendCmdPacket(iv->radioId.u64, 0x15, (0x81+i), true);
                                                break; // only retransmit one frame per loop
                                            }
                                            yield();
                                        }
                                    }
                                    else {
                                        ESP_LOGE(TAG, "while retrieving data: last frame missing: Request Retransmit");
                                        if(0x00 != mLastPacketId)
                                            mSys->Radio.sendCmdPacket(iv->radioId.u64, 0x15, mLastPacketId, true);
                                        else
                                            mSys->Radio.sendTimePacket(iv->radioId.u64, mPayload[iv->id].ts);
                                    }
                                    mSys->Radio.switchRxCh(100);
                                }
                            }
                        }
                    }
                    else {
                        mPayload[iv->id].complete = true;
                        iv->ts = mPayload[iv->id].ts;
                        uint8_t payload[128] = {0};
                        uint8_t offs = 0;
                        for(uint8_t i = 0; i < (mPayload[iv->id].maxPackId); i ++) {
                            memcpy(&payload[offs], mPayload[iv->id].data[i], (mPayload[iv->id].len[i]));
                            offs += (mPayload[iv->id].len[i]);
                            yield();
                        }
                        offs-=2;
                        
                        ESP_LOGI(TAG, "Payload (%s): ", String(offs));
                        mSys->Radio.dumpBuf(NULL, payload, offs);

                        mRxSuccess++;

                        for(uint8_t i = 0; i < iv->listLen; i++) {
                            iv->addValue(i, payload);
                            yield();
                        }
                        iv->doCalculations();
                    }
                }
                yield();
            }
        }
    }

    static char* C_dumpBuf(uint8_t buf[], uint8_t len) {

        char buffer [len * 2];

        for(uint8_t i = 0; i < len; i++) {
            // sprintf(&buffer[2*i], "%02x", buf[i]);

            // Serial.print(C_DHEX(buf[i]));
            // result = result + String(C_DHEX(buf[i])) + " ";
            // result.concat(String(C_DHEX(buf[i])));
            // result += " " + String(C_DHEX(buf[i]));
        }
        // ESP_LOGI(TAG, "test %s", buffer);

        return buffer;
    }

    static String C_DHEX(uint8_t b) {
        if( b<0x10 ) return "0";
        return String(b,HEX);
    }
    static String C_DHEX(uint16_t b) {
        if( b<0x10 ) return "000";
        else if( b<0x100 ) return "00";
        else if( b<0x1000 ) return "0";
        return String(b,HEX);
    }
    static String C_DHEX(uint32_t b) {
        if( b<0x10 ) return F("0000000");
        else if( b<0x100 ) return "000000";
        else if( b<0x1000 ) return "00000";
        else if( b<0x10000 ) return "0000";
        else if( b<0x100000 ) return "000";
        else if( b<0x1000000 ) return "00";
        else if( b<0x10000000 ) return "0";
        return String(b,HEX);
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
