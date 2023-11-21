#ifndef SCHEDULER_H
#define SCHEDULER_H

// C++ version (c) 2023 Andrei Ivanov MIT License
/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator : Just In Time TX scheduling queue

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

#include <vector>
#include <cstdint>

#include "packet-forwarder/loragw_hal.h"

// values available for the 'tx_mode' parameter
#define IMMEDIATE       0
#define TIMESTAMPED     1
#define ON_GPS          2

enum scheduler_error_e {
    SCHEDULER_ERROR_OK,
    SCHEDULER_ERROR_TOO_LATE,
    SCHEDULER_ERROR_TOO_EARLY,
    SCHEDULER_ERROR_FULL,
    SCHEDULER_ERROR_EMPTY,
    SCHEDULER_ERROR_COLLISION_PACKET,
    SCHEDULER_ERROR_COLLISION_BEACON,
    SCHEDULER_ERROR_TX_FREQ,
    SCHEDULER_ERROR_TX_POWER,
    SCHEDULER_ERROR_GPS_UNLOCKED,
    SCHEDULER_ERROR_INVALID
};

enum scheduler_pkt_type_e {
    SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A,
    SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_B,
    SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_C,
    SCHEDULER_PKT_TYPE_BEACON
};

class ScheduleItem {
public:
    enum scheduler_pkt_type_e pktType;   // Packet type: Downlink, Beacon...
    uint32_t preDelay;             // Amount of time before packet timestamp to be reserved
    uint32_t postDelay;            // Amount of time after packet timestamp to be reserved (time on air)
    virtual void getItem(void *value) = 0;
    virtual void setItem(const void* value) = 0;
    virtual uint32_t getCountUs() const = 0;
    virtual void setCountUs(uint32_t value) = 0;
    virtual uint32_t getTimeOnAir() const = 0;
    virtual void setTxMode(uint8_t value) = 0;        // select on what event/time the TX is triggered
    /*
    ScheduleItem() = default;
    ScheduleItem(const ScheduleItem &value) = delete;
    ScheduleItem(ScheduleItem &&value) = default;
    ScheduleItem& operator=(const ScheduleItem&);
     */
};

class Scheduler {
private:
    std::vector<ScheduleItem> queue;
    std::size_t count;                // Total number of packets in the queue (downlinks, beacons...)
    void reset();
    void sortTime();
public:
    Scheduler();
    Scheduler(std::size_t size);
    std::size_t size();
    virtual std::size_t itemSize() = 0;
    void setSize(std::size_t size);
    bool isFull();
    bool isEmpty();
    // Return index of node containing a packet inline with given time
    enum scheduler_error_e peek(std::size_t &retIndex, uint32_t time_us);
    enum scheduler_error_e enqueue(uint32_t time_us, ScheduleItem &item, enum scheduler_pkt_type_e pktType);
    enum scheduler_error_e dequeue(ScheduleItem &retItem, enum scheduler_pkt_type_e &pktType, int index);
};

#endif
