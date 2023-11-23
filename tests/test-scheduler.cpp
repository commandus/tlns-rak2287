/*
Scheduler sh([]() {
    return new ScheduleItemConcentrator;
}, 32);

void testScheduler(Scheduler &scheduler) {
    ScheduleItemConcentrator item;
    item.setTxMode(IMMEDIATE);
    uint32_t current_concentrator_time = 2000000;   // 2s
    // lgw_get_instcnt(&current_concentrator_time);
    uint32_t gap = 42500;
    item.setCountUs(current_concentrator_time + 1000 + gap);
    scheduler_error_e e = sh.enqueue(current_concentrator_time, item, SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A);
    std::cerr
            << "enqueue e: " << e
            << std::endl;
    item.setTxMode(TIMESTAMPED);
    e = sh.enqueue(925, item, SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A);
    std::cerr
            << "enqueue e: " << e
            << std::endl;
    e = sh.enqueue(926, item, SCHEDULER_PKT_TYPE_DOWNLINK_CLASS_A);
    std::cerr
            << "enqueue e: " << e
            << std::endl;

    std::size_t idx;
    uint32_t us = 10000;
    auto errcode = sh.peek(idx, us);
    std::cerr
            << "errcode: " << errcode
            << " idx: " << idx
            << " us: " << us
            << " size: " << sh.size()
            << " empty: " << sh.isEmpty()
            << " full: " << sh.isFull()
            << std::endl;
}
*/

int main(int argc, char **argv) {

}
