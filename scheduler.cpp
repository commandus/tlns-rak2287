#include "scheduler.h"
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

#include <cstdlib>     // qsort_r
#include <utility>

#define TX_START_DELAY                  1500    // microseconds
#define TX_MARGIN_DELAY                 1000    // Packet overlap margin in microseconds
#define TX_SCHEDULER_DELAY              40000   // Pre-delay to program packet for TX in microseconds
#define TX_MAX_ADVANCE_DELAY            ((SCHEDULER_NUM_BEACON_IN_QUEUE + 1) * 128 * 1E6) // Maximum advance delay accepted for a TX packet, compared to current time
#define BEACON_GUARD                    3000000 // Interval where no ping slot can be placed, to ensure beacon can be sent
#define BEACON_RESERVED                 2120000 // Time on air of the beacon, with some margin

#define SCHEDULER_QUEUE_MAX             32  // Maximum number of packets to be stored in JiT queue
#define SCHEDULER_NUM_BEACON_IN_QUEUE   3   // Number of beacons to be loaded in JiT queue at any tim

#define DEF_COUNT   32

ScheduleItem::ScheduleItem()
    : pktType(SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A), preDelay(0), postDelay(0)
{
}

Scheduler::Scheduler(std::function<ScheduleItem*()> create)
    : count(0)
{
    onCreate = std::move(create);
    setSize(DEF_COUNT);
}

Scheduler::Scheduler(std::function<ScheduleItem*()> create, std::size_t sz)
    : count(0)
{
    onCreate = create;
    setSize(sz);
}

Scheduler::~Scheduler()
{
    setSize(0);
}

std::size_t Scheduler::size()
{
    return queue.size();
}

void Scheduler::setSize(std::size_t size)
{
    auto sz = queue.size();

    if (size < sz) {
        for (auto i = size; i < sz; i++)
            delete queue[i];
    }
    queue.resize(size);
    if (size > sz) {
        for (auto i = sz; i < size; i++)
            queue[i] = onCreate();
    }
}

bool Scheduler::isFull()
{
    return count >= queue.size();
}

bool Scheduler::isEmpty() const
{
    return count == 0;
}

void Scheduler::sortTime()
{
    std::qsort(queue.data(), queue.size(), sizeof(ScheduleItem *),
        [](const void *a, const void *b) {
            auto aa = *(const ScheduleItem **) a;
            return (int) (
                (*(const ScheduleItem **) a)->getCountUs() - (*(const ScheduleItem **) b)->getCountUs());
        });
}

static inline bool jit_collision_test(
    uint32_t p1_count_us,
    uint32_t p1_pre_delay,
    uint32_t p1_post_delay,
    uint32_t p2_count_us,
    uint32_t p2_pre_delay,
    uint32_t p2_post_delay
) {
    if (((p1_count_us - p2_count_us) <= (p1_pre_delay + p2_post_delay + TX_MARGIN_DELAY)) ||
        ((p2_count_us - p1_count_us) <= (p2_pre_delay + p1_post_delay + TX_MARGIN_DELAY))) {
        return true;
    } else {
        return false;
    }
}

#define CONTEXT_FSK_sync_word_size  3

enum SCHEDULER_ERROR Scheduler::enqueue(
    uint32_t time_us,
    ScheduleItem &item
)
{
    int i = 0;
    uint32_t packet_post_delay = 0;
    uint32_t packet_pre_delay = 0;
    uint32_t target_pre_delay = 0;
    enum SCHEDULER_ERROR err_collision;
    uint32_t asap_count_us;

    if (isFull())
        return SCHEDULER_ERROR_FULL;

    // Compute packet pre/post delays depending on packet's type
    switch (item.pktType) {
        case SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A:
        case SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_B:
        case SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_C:
            packet_pre_delay = TX_START_DELAY + TX_SCHEDULER_DELAY;
            packet_post_delay = item.getTimeOnAir() * 1000UL; // us
            break;
        case SCHEDULER_PKT_TYPE_BEACON:
            /* As defined in LoRaWAN spec */
            packet_pre_delay = TX_START_DELAY + BEACON_GUARD + TX_SCHEDULER_DELAY;
            packet_post_delay = BEACON_RESERVED;
            break;
        default:
            break;
    }
    // An immediate downlink becomes a timestamped downlink "ASAP"
    // Set the packet count_us to the first available slot
    if (item.pktType == SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_C) {
        // change tx_mode to timestamped
        item.setTxMode(TIMESTAMPED);

        /* Search for the ASAP timestamp to be given to the packet */
        asap_count_us = time_us + 2 * TX_SCHEDULER_DELAY; /* margin */
        if (count > 0) {
            // If the jit queue is not empty try to insert it: ASAP = NOW + MARGIN, at the last index of the queue, between 2 downlinks in the queue
            // First, try if the ASAP time collides with an already enqueued downlink
            for (i = 0; i < count; i++) {
                if (jit_collision_test(asap_count_us, packet_pre_delay, packet_post_delay,
                    queue[i]->getCountUs(), queue[i]->preDelay, queue[i]->postDelay)) {
                    break;
                }
            }
            if (i < count) {
                // Search for the best slot then
                for (i = 0; i < count; i++) {
                    asap_count_us = queue[i]->getCountUs() + queue[i]->postDelay + packet_pre_delay + TX_SCHEDULER_DELAY + TX_MARGIN_DELAY;
                    if (i < count - 1) {
                        // Check if packet can be inserted between this index and the next one
                        if (jit_collision_test(asap_count_us, packet_pre_delay, packet_post_delay,
                            queue[i + 1]->getCountUs(), queue[i + 1]->preDelay, queue[i + 1]->postDelay))
                            continue;
                        else
                            break;
                    }
                }
            }
        }
        // Set packet with ASAP timestamp
        item.setCountUs(asap_count_us);
    }

    // Check criteria_1: is it already too late to send this packet ?
    // The packet should arrive at least at (tmst - TX_START_DELAY) to be programmed into concentrator
    // Also add some margin
    if (((item.getCountUs() - time_us)) <= (TX_START_DELAY + TX_MARGIN_DELAY + TX_SCHEDULER_DELAY))
        return SCHEDULER_ERROR_TOO_LATE;

    // Check criteria_2: Does packet timestamp seem plausible compared to current time
    // We do not expect the server to program a downlink too early compared to current time
    // Class A: 1 or 2 seconds time window after RX
    // Class B: 128 seconds time window
    // Class C: no check needed
    // t_packet > t_current + TX_MAX_ADVANCE_DELAY
    if ((item.pktType == SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A) || (item.pktType == SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_B)) {
        if (((item.getCountUs() - time_us)) > TX_MAX_ADVANCE_DELAY) {
            return SCHEDULER_ERROR_TOO_EARLY;
        }
    }

    // Check criteria_3: does this new packet overlap with a packet already enqueued ?
    // Note: - need to take into account packet's pre_delay and post_delay of each packet
    // Beacon guard can be ignored if we try to queue a Class A downlink
    for (i = 0; i < count; i++) {
        // We ignore Beacon Guard for Class A/C downlinks
        if (((item.pktType == SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A) || (item.pktType == SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_C))
            && (queue[i]->pktType == SCHEDULER_PKT_TYPE_BEACON))
            target_pre_delay = TX_START_DELAY;
        else
            target_pre_delay = queue[i]->preDelay;

        // Check if there is a collision
        if (jit_collision_test(item.getCountUs(), packet_pre_delay, packet_post_delay,
                               queue[i]->getCountUs(), target_pre_delay, queue[i]->postDelay)) {
            switch (queue[i]->pktType) {
                case SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A:
                case SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_B:
                case SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_C:
                    err_collision = SCHEDULER_ERROR_COLLISION_PACKET;
                    break;
                case SCHEDULER_PKT_TYPE_BEACON:
                    err_collision = SCHEDULER_ERROR_COLLISION_BEACON;
                    break;
                default:
                    return SCHEDULER_ERROR_INVALID;
            }
            return err_collision;
        }
    }

    // Finally enqueue it
    // Insert packet at the end of the queue
    queue[count]->pktType = item.pktType;
    queue[count]->setItem(item.get());
    queue[count]->preDelay = packet_pre_delay;
    queue[count]->postDelay = packet_post_delay;
    count++;
    // Sort the queue in ascending order of packet timestamp
    sortTime();
    return SCHEDULER_ERROR_OK;
}

enum SCHEDULER_ERROR Scheduler::dequeue(
    ScheduleItem &retItem,
    int index
)
{
    if ((index < 0) || (index >= SCHEDULER_QUEUE_MAX))
        return SCHEDULER_ERROR_INVALID;
    if (isEmpty())
        return SCHEDULER_ERROR_EMPTY;
    // Dequeue requested packet
    retItem.setItem(queue[index]->get());
    retItem.pktType = queue[index]->pktType;
    count--;
    // Replace dequeued packet with last packet of the queue */
    queue[index] = queue[count];
    // Sort queue in ascending order of packet timestamp
    sortTime();
    return SCHEDULER_ERROR_OK;
}

enum SCHEDULER_ERROR Scheduler::peek(
    std::size_t &retIndex,
    uint32_t time_us
) {
    // Return index of node containing a packet inline with given time
    int idx_highest_priority = -1;
    if (isEmpty())
        return SCHEDULER_ERROR_EMPTY;
    // Search for highest priority packet to be sent
    for (auto i = 0; i < count; i++) {
        // First check if that packet is outdated:
        // too much in advance, and was not rejected at enqueue time,
        // it means that we missed it for peeking, we need to drop it
        if ((queue[i]->getCountUs() - time_us) >= TX_MAX_ADVANCE_DELAY) {
            // Drop the packet to avoid lock-up
            count--;
            // Replace dropped packet with last packet of the queue
            queue[i] = queue[count];
            // Sort queue in ascending order of packet timestamp
            sortTime();
            // restart loop  after purge to find packet to be sent
            i = 0;
            continue;
        }
        // Then look for highest priority packet to be sent
        if ((idx_highest_priority == -1)
            || (((queue[i]->getCountUs() - time_us) < (queue[idx_highest_priority]->getCountUs() - time_us)))) {
            idx_highest_priority = i;
        }
    }
    if (idx_highest_priority == -1)
        return SCHEDULER_ERROR_EMPTY;
    // Peek criteria 1: look for a packet to be sent in next TX_SCHEDULER_DELAY ms timeframe
    if ((queue[idx_highest_priority]->getCountUs() - time_us) < TX_SCHEDULER_DELAY)
        retIndex = idx_highest_priority;
    else
        return SCHEDULER_ERROR_TOO_EARLY;
    return SCHEDULER_ERROR_OK;
}
