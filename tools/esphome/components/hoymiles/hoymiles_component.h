#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/time/real_time_clock.h"


#include "Arduino.h"
#include "debug.h"
#include "defines.h"
#include "CircularBuffer.h"
#include "hmInverter.h"
#include "hmRadio.h"
#include "hmSystem.h"

namespace esphome {
namespace hoymiles {


typedef CircularBuffer<packet_t, PACKET_BUFFER_SIZE> BufferType;
typedef HmRadio<DEF_RF24_CE_PIN, DEF_RF24_CS_PIN, BufferType> RadioType;
typedef Inverter<float> InverterType;
typedef HmSystem<RadioType, BufferType, MAX_NUM_INVERTERS, InverterType> HmSystemType;


typedef struct {
    uint8_t invId;
    uint32_t ts;
    uint8_t data[MAX_PAYLOAD_ENTRIES][MAX_RF_PAYLOAD_SIZE];
    uint8_t len[MAX_PAYLOAD_ENTRIES];
    bool complete;
    uint8_t maxPackId;
    uint8_t retransmits;
    bool requested;
} invPayload_t;

class HoymilesComponent : public Component {
    public:
        void setup() override;
        void dump_config() override;
        float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }
        // void set_component_source(const char * 	source) {}
        void loop() override;
        void sendLimitPacket(uint64_t invId, uint16_t *limit);
        void sendTurnOnOffPacket(uint64_t invId, bool state);
        void sendRestartPacket(uint64_t invId);
        void sendCleanStatePacket(uint64_t invId);

        void set_ce_pin(InternalGPIOPin *pin) { this->ce_pin_ = pin; }
        void set_cs_pin(InternalGPIOPin *pin) { this->cs_pin_ = pin; }
        void set_irq_pin(InternalGPIOPin *pin) { this->irq_pin_ = pin; }

        void set_time(esphome::time::RealTimeClock *clock) { time_clock = clock; }
        void set_send_interval(uint16_t send_interval) { send_interval_ = send_interval; }
        void set_amplifier_power(uint8_t amplifier_power) { amplifier_power_ = amplifier_power; }

        void add_inverter(int count, char *identification, char* serial_number) {
            this->inverters_[count].identification_ = identification;

            unsigned long long   x = 0x0000; 
            sscanf( serial_number , "%llx" , &x); 
        
            this->inverters_[count].serial_number_ = (uint64_t)x;
        }

        Inverter<> * get_inverter(char *identification);

        time_t getRtcTimestamp() {
            return this->time_clock->now().timestamp;
        }

        time_t getTimestamp() { return this->timestamp_; }
        void setTimestamp(time_t timestamp ) { this->timestamp_ = timestamp; }

        uint32_t getDebugRxFailed() { return this->mRxFailed; }
        uint32_t getDebugRxSuccess() { return this->mRxSuccess; }
        uint32_t getDebugFrameCount() { return this->mFrameCnt; }
        uint32_t getDebugSendCount();


    protected:
        time_t timestamp_ = 0;
        uint8_t mMaxRetransPerPyld = 5;
        uint32_t mRxFailed = 0;
        uint32_t mRxSuccess = 0;
        uint32_t mFrameCnt = 0;

        InternalGPIOPin *ce_pin_;
        InternalGPIOPin *cs_pin_;
        InternalGPIOPin *irq_pin_;

        struct Hoymiles_Inverter {
            char *identification_{nullptr};
            uint64_t serial_number_;
        } inverters_[MAX_NUM_INVERTERS];

        esphome::time::RealTimeClock *time_clock{nullptr};
        uint16_t send_interval_;
        uint8_t amplifier_power_;

        config_t mConfig;



    private:
        void processPayload(bool retransmit);
        void resetPayload(Inverter<>* iv);

};
}  // namespace uart
}  // namespace esphome
