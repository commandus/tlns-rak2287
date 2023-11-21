#ifndef SCHEDULECONCENTRATOR_H
#define SCHEDULECONCENTRATOR_H

#include "packet-forwarder/loragw_hal.h"

#include "scheduler.h"

class ScheduleConcentrator : public ScheduleItem {
    private:
        struct lgw_pkt_tx_s item;
    public:
        void getItem(void *value) override;
        void setItem(const void* value) override;
        uint32_t getCountUs() const override;
        void setCountUs(uint32_t value) override;
        uint32_t getTimeOnAir() const override;
        void setTxMode(uint8_t value) override;
        ScheduleConcentrator();
        ScheduleConcentrator(const struct lgw_pkt_tx_s& value);
};

#endif
