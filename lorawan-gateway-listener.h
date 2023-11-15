#ifndef LORAWAN_GATEWAY_LISTENER_H_
#define LORAWAN_GATEWAY_LISTENER_H_ 1

#include <vector>
#include <mutex>
#include <functional>

#include "gateway-settings.h"
#include "log-intf.h"

#define MEASUREMENT_COUNT_SIZE 23

// listen() flags parameter values
// 1- Do not send messages
#define FLAG_GATEWAY_LISTENER_NO_SEND   1
// 2- Do not send beacons
#define FLAG_GATEWAY_LISTENER_NO_BEACON 2

class LGWStatus {
public:
    uint32_t inst_tstamp;     ///< SX1302 counter (INST)
    uint32_t trig_tstamp;     ///< SX1302 counter (PPS)
    float temperature;        ///< Concentrator temperature
};

class TxPacket {
public:
    struct lgw_pkt_tx_s pkt;
    TxPacket();
};

class LoraGatewayListener {
private:
    int logVerbosity;

    std::function<void(
        const LoraGatewayListener *listener,
        const SEMTECH_PROTOCOL_METADATA *metadata,
        const std::string &payload
    )> onUpstream;

    std::function<void(
        const LoraGatewayListener *listener,
        const uint32_t frequency,
        const uint16_t results[LGW_SPECTRAL_SCAN_RESULT_SIZE]
    )> onSpectralScan;

    std::function<void(
        const LoraGatewayListener *listener,
        bool gracefullyStopped
    )> onStop;

    // control access
    std::mutex mLGW;                         ///< control access to the concentrator
    std::mutex mReportSpectralScan;          ///< control access to spectral scan report
    mutable std::mutex mLog;                 ///< control access to log facility
    std::mutex mXTALcorrection;              ///< control access to the XTAL correction

    struct jit_queue_s jit_queue[LGW_RF_CHAIN_NB];  ///< Just In Time TX scheduling for each radio channel
    bool xtal_correct_ok;            ///< set true when XTAL correction is stable enough
    double xtal_correct;             ///< XTAL frequency correction coefficient. XTAL(crystal) in timing refers to a quartz crystal.

    // threads
    void upstreamRunner();              // receive Lora packets from end-device(s)
    void jitRunner();                   // transmit from JIT queue
    void spectralScanRunner();

    bool getTxGainLutIndex(uint8_t rf_chain, int8_t rf_power, uint8_t * lut_index);
protected:
    // Apply config
    int setup();
public:
    // thread control
    bool stopRequest;               ///< set to true to stop all threads
    // thread finish indicators
    bool upstreamThreadRunning;
    bool jitThreadRunning;
    bool spectralScanThreadRunning;

    int lastLgwCode;
    LorawanGatewaySettings *config;
    int flags;

    uint64_t eui;        ///< Gateway EUI

    LoraGatewayListener();
    LoraGatewayListener(LorawanGatewaySettings *cfg);
    ~LoraGatewayListener();

    void log(
        int level,
        int errorCode,
        const std::string &message
    ) const;

    /**
        LGW library version.
        Calls lgw_version_info();
    */
    std::string version();
    // SX1302 Status
    bool getStatus(LGWStatus &status);
    int start();
    int stop(int waitSeconds);
    bool isRunning() const;
    bool isStopped() const;


    void setOnSpectralScan(
        std::function<void(
            const LoraGatewayListener *listener,
            const uint32_t frequency,
            const uint16_t results[LGW_SPECTRAL_SCAN_RESULT_SIZE]
        )> value
    );
    void setOnLog(LogIntf *value);
    void setOnUpstream(
        std::function<void(
            const LoraGatewayListener *listener,
            const SEMTECH_PROTOCOL_METADATA *metadata,
            const std::string &payload
        )> value
    );
    void setOnStop(
        std::function<void(
            const LoraGatewayListener *listener,
            bool gracefullyStopped
        )> value
    );
    void setLogVerbosity(int level);
    int enqueueTxPacket(TxPacket &tx);

    std::string toString() const;

    LogIntf *onLog;
};

#endif
