#ifndef SCHEDULECONCENTRATOR_H
#define SCHEDULECONCENTRATOR_H

#include "gateway-settings.h"

#include "scheduler.h"

class ScheduleItemConcentrator : public ScheduleItem {
    private:
        struct lgw_pkt_tx_s item;
    public:
        /**
        * Copy struct lgw_pkt_tx_s to the value
        * @param value returned value
        */
        void *get() override;
        /**
         * Set struct lgw_pkt_tx_s
         * @param value struct lgw_pkt_tx_s
         */
        void setItem(const void* value) override;
        /**
         * Get internal time counter value in microseconds
         * @return internal time counter in microseconds
         */
        uint32_t getCountUs() const override;
        /**
         * Set internal time counter value in microseconds
         * @param value internal time counter value in microseconds
         */
        void setCountUs(uint32_t value) override;
        uint32_t getTimeOnAir() const override;
        void setTxMode(SCHEDULER_TX_MODE value) override;
        ScheduleItemConcentrator();
        ScheduleItemConcentrator(const struct lgw_pkt_tx_s& value);
};

#endif
