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

License: Revised BSD License, see LICENSE.SEMTECH.txt file include in the project
*/

#include <vector>
#include <cstdint>
#include <functional>

// values available for the 'tx_mode' parameter
enum SCHEDULER_TX_MODE {
    IMMEDIATE = 0,
    TIMESTAMPED = 1,
    ON_GPS = 2
};

enum SCHEDULER_ERROR {
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

enum SCHEDULER_PACKET_TYPE {
    SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A,
    SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_B,
    SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_C,
    SCHEDULER_PKT_TYPE_BEACON
};

/**
 * Abstract class
 */
class ScheduleItem {
public:
    enum SCHEDULER_PACKET_TYPE pktType;             ///< Packet type: Downlink class A, B. C, Beacon...
    uint32_t preDelay;                              ///< Amount of time before packet timestamp to be reserved
    uint32_t postDelay;                             ///< Amount of time after packet timestamp to be reserved (time on air)
    virtual void* get() = 0;                        ///< return packet to be send
    virtual void setItem(const void* value) = 0;    ///< assign packet to be send
    virtual uint32_t getCountUs() const = 0;        ///< return internal time counter in microseconds
    virtual void setCountUs(uint32_t value) = 0;    ///< set internal time counter in microseconds
    virtual uint32_t getTimeOnAir() const = 0;
    /**
     * Set when to send
     * @param value 0- IMMEDIATE, 1- TIMESTAMPED, 2- ON_GPS
     */
    virtual void setTxMode(SCHEDULER_TX_MODE value) = 0;        // select on what event/time the TX is triggered
    ScheduleItem();
    virtual ~ScheduleItem() = default;
};

class Scheduler {
private:
    std::function<ScheduleItem*()> onCreate;
    std::vector<ScheduleItem*> queue;
    std::size_t count;                // Total number of packets in the queue (downlinks, beacons...)
    void sortTime();
public:
    explicit Scheduler(std::function<ScheduleItem*()> create);
    explicit Scheduler(std::function<ScheduleItem*()> create, std::size_t size);
    virtual ~Scheduler();
    std::size_t size();
    void setSize(std::size_t size);
    bool isFull();
    bool isEmpty() const;
    // Return index of node containing a packet inline with given time
    enum SCHEDULER_ERROR peek(std::size_t &retIndex, uint32_t time_us);
    enum SCHEDULER_ERROR enqueue(uint32_t time_us, ScheduleItem &item);
    enum SCHEDULER_ERROR dequeue(ScheduleItem &retItem, int index);
};

#endif
