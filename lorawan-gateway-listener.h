#ifndef LORAWAN_GATEWAY_LISTENER_H_
#define LORAWAN_GATEWAY_LISTENER_H_ 1

#include <vector>
#include <functional>
#include <mutex>

#include "gateway-settings.h"
#include "log-intf.h"
#include "gateway-settings.h"
#include "scheduler.h"

#define MEASUREMENT_COUNT_SIZE 23

// listen() flags parameter values
// 1- Do not send messages
#define FLAG_GATEWAY_LISTENER_NO_SEND   1
// 2- Do not send beacons
#define FLAG_GATEWAY_LISTENER_NO_BEACON 2

#define DEF_TRANSMIT_QUEUE_SIZE  32

class TransmitQueue {
private:
    std::mutex mutexEnqueue;    ///< control access to the concentrator's queue
public:
    size_t size;
    std::vector<struct lgw_pkt_tx_s> queue;
    TransmitQueue();
    virtual ~TransmitQueue();
    bool push(struct lgw_pkt_tx_s &value);
    bool pop(struct lgw_pkt_tx_s &value);
};

class LoraGatewayListener {
private:
    int logVerbosity;
    LogIntf *onLog;
    std::function<void(
        const LoraGatewayListener *listener,
        lgw_pkt_rx_s *packet
    )> onUpstream;
    Scheduler* scheduler;
    double xtal_correct;                                ///< XTAL frequency correction coefficient. XTAL(crystal) in timing refers to a quartz crystal.
    int state;                                          ///< set to 2 to stop all threads. 0- stopped, 1- running, 2- request to stop
    LorawanGatewaySettings *config;
    std::mutex mutexEnqueue;                                 ///< control access to the concentrator's queue
    TransmitQueue transmitQueue;

    int doUpstream(lgw_pkt_rx_s *packets, int count);
    void doJitDownstream();
    bool validateMetadata(lgw_pkt_rx_s *p) const;
    void upstreamDownstreamLoop();                    ///< receive Lora packets from end-device(s)
    bool getTxGainLutIndex(uint8_t rf_chain, int8_t rf_power, uint8_t * lut_index) const;
    // Apply config
    int setup();
    /**
     * Invalidate TX packet
     * @param value packet to validate
     * @param downlinkClass return LoRaWAN class
     * @return 0- success
     */
    int invalidateTxPacket(
        lgw_pkt_tx_s &value,
        enum SCHEDULER_PACKET_TYPE &downlinkClass
    );
    int schedulePacketToTransmit(
        struct lgw_pkt_tx_s &pkt
    );
public:
    uint64_t gatewayId;        ///< Gateway EUI
    LoraGatewayListener();
    explicit LoraGatewayListener(std::function<ScheduleItem*()> create, size_t queueSize);
    ~LoraGatewayListener();

    void log(
        int level,
        int errorCode,
        const std::string &message
    ) const;

    // library version
    std::string version();
    // SX1302 Status
    int run();
    void stop();

    void setConfig(
        LorawanGatewaySettings *config
    );

    void setOnUpstream(
        std::function<void(
            const LoraGatewayListener *listener,
            lgw_pkt_rx_s *packet
        )> value
    );
    void setOnLog(
        LogIntf *value,
        int level
    );
    int enqueueTxPacket(
        struct lgw_pkt_tx_s &pkt
    );
};

#endif
