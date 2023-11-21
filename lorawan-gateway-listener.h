#ifndef LORAWAN_GATEWAY_LISTENER_H_
#define LORAWAN_GATEWAY_LISTENER_H_ 1

#include <vector>
#include <functional>
#include <mutex>

#include "gateway-settings.h"
#include "log-intf.h"
#include "packet-forwarder/loragw_hal.h"
#include "packet-forwarder/jitqueue.h"

#define MEASUREMENT_COUNT_SIZE 23

// listen() flags parameter values
// 1- Do not send messages
#define FLAG_GATEWAY_LISTENER_NO_SEND   1
// 2- Do not send beacons
#define FLAG_GATEWAY_LISTENER_NO_BEACON 2

class LoraGatewayListener {
private:
    int logVerbosity;
    LogIntf *onLog;
    std::function<void(
        const LoraGatewayListener *listener,
        lgw_pkt_rx_s *packet
    )> onUpstream;
    struct jit_queue_s jit_queue[LGW_RF_CHAIN_NB];      ///< Just In Time TX scheduling for each radio channel
    double xtal_correct;                                ///< XTAL frequency correction coefficient. XTAL(crystal) in timing refers to a quartz crystal.
    int state;                                          ///< set to 2 to stop all threads. 0- stopped, 1- running, 2- request to stop
    LorawanGatewaySettings *config;
    std::mutex mutexLgw;                                 ///< control access to the concentrator

    int doUpstream(lgw_pkt_rx_s *packets, int count);
    void doJitDownstream();
    bool validateMetadata(lgw_pkt_rx_s *p) const;
    void upstreamDownstreamLoop();                    ///< receive Lora packets from end-device(s)
    bool getTxGainLutIndex(uint8_t rf_chain, int8_t rf_power, uint8_t * lut_index) const;
    // Apply config
    int setup();
public:
    uint64_t gatewayId;        ///< Gateway EUI

    LoraGatewayListener();
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
