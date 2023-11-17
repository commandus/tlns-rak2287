/*
 * C++ wrapper of lora_pkt_fwd.c (C)2019 Semtech License: Revised BSD License, see LICENSE.TXT file included in the project
 */
#include <thread>
#include <sstream>

#include "lorawan-gateway-listener.h"
#include "lorawan-error.h"
#include "lorawan-msg.h"

// max number of packets per fetch/send cycle
#define PACKETS_MAX_SIZE         255
// ms waited when a fetch return no packets
#define UPSTREAM_FETCH_DELAY_MS     10
#define INVALID_TEMPERATURE_C       (-273.15)

bool LoraGatewayListener::validateMetadata(
    lgw_pkt_rx_s *p
) const {
    // allocate memory for metadata fetching and processing
    // basic metadata filtering
    switch(p->status) {
        case STAT_CRC_OK:
            if (!config->gateway.forwardCRCValid) {
                return false; // skip that metadata
            }
            break;
        case STAT_CRC_BAD:
            if (!config->gateway.forwardCRCError) {
                return false; // skip that metadata
            }
            break;
        case STAT_NO_CRC:
            if (!config->gateway.forwardCRCDisabled) {
                return false; // skip that metadata
            }
            break;
        default:
            log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_UNKNOWN_STATUS, ERR_LORA_GATEWAY_UNKNOWN_STATUS);
            return false; // skip that metadata
    }

    SEMTECH_PROTOCOL_METADATA metadata;
    metadata.gatewayId = config->gateway.gatewayId;

    // time
    metadata.tmst = p->count_us;

    metadata.t = time(nullptr);

    // Packet concentrator channel, RF chain & RX frequency, 34-36 useful chars
    metadata.rfch = p->rf_chain;
    metadata.chan = p->if_chain;
    metadata.freq = p->freq_hz;

    // Validate metadata status
    switch (p->status) {
        case STAT_CRC_OK:
            metadata.stat = 1;
            break;
        case STAT_CRC_BAD:
            metadata.stat = -1;
            break;
        case STAT_NO_CRC:
            metadata.stat = 0;
            break;
        default:
            log(LOG_ERR, ERR_CODE_LORA_GATEWAY_UNKNOWN_STATUS, ERR_LORA_GATEWAY_UNKNOWN_STATUS);
            metadata.stat = -2;
            return false;
    }

    // Packet modulation, 13-14 useful chars
    if (p->modulation == MOD_LORA) {
        metadata.modu = LORA;
        // Lora datarate & bandwidth, 16-19 useful chars
        switch (p->datarate) {
            case DR_LORA_SF5:
                metadata.spreadingFactor = DRLORA_SF5;
                break;
            case DR_LORA_SF6:
                metadata.spreadingFactor = DRLORA_SF6;
                break;
            case DR_LORA_SF7:
                metadata.spreadingFactor = DRLORA_SF7;
                break;
            case DR_LORA_SF8:
                metadata.spreadingFactor = DRLORA_SF8;
                break;
            case DR_LORA_SF9:
                metadata.spreadingFactor = DRLORA_SF9;
                break;
            case DR_LORA_SF10:
                metadata.spreadingFactor = DRLORA_SF10;
                break;
            case DR_LORA_SF11:
                metadata.spreadingFactor = DRLORA_SF11;
                break;
            case DR_LORA_SF12:
                metadata.spreadingFactor = DRLORA_SF12;
                break;
            default:
                metadata.spreadingFactor = DRLORA_SF5;
                log(LOG_ERR, ERR_CODE_LORA_GATEWAY_UNKNOWN_DATARATE, ERR_LORA_GATEWAY_UNKNOWN_DATARATE);
                return false;
        }
        switch (p->bandwidth) {
            case BW_125KHZ:
                metadata.bandwith = BANDWIDTH_INDEX_125KHZ;
                break;
            case BW_250KHZ:
                metadata.bandwith = BANDWIDTH_INDEX_250KHZ;
                break;
            case BW_500KHZ:
                metadata.bandwith = BANDWIDTH_INDEX_500KHZ;
                break;
            default:
                metadata.bandwith = BANDWIDTH_INDEX_125KHZ;
                log(LOG_ERR, ERR_CODE_LORA_GATEWAY_UNKNOWN_BANDWIDTH, ERR_LORA_GATEWAY_UNKNOWN_BANDWIDTH);
                return false;
        }

        // Packet ECC coding rate, 11-13 useful chars
        switch (p->coderate) {
            case CR_LORA_4_5:
                metadata.codingRate = CRLORA_4_5;
                break;
            case CR_LORA_4_6:
                metadata.codingRate = CRLORA_4_6;
                break;
            case CR_LORA_4_7:
                metadata.codingRate = CRLORA_4_7;
                break;
            case CR_LORA_4_8:
                metadata.codingRate = CRLORA_4_8;
                break;
            case 0: // treat the CR0 case (mostly false sync)
                metadata.codingRate = CRLORA_0FF;
                break;
            default:
                log(LOG_ERR, ERR_CODE_LORA_GATEWAY_UNKNOWN_CODERATE, ERR_LORA_GATEWAY_UNKNOWN_CODERATE);
                return false;
        }

        // Signal RSSI, payload size
        metadata.rssi = (int16_t) p->rssis;

        // Lora SNR
        metadata.lsnr = p->snr;
    } else if (p->modulation == MOD_FSK) {
        metadata.modu = FSK;
        metadata.bps = p->datarate;
    } else {
        log(LOG_ERR, ERR_CODE_LORA_GATEWAY_UNKNOWN_MODULATION, ERR_LORA_GATEWAY_UNKNOWN_MODULATION);
        return false;
    }
    return true;
}

int LoraGatewayListener::doUpstream(
    lgw_pkt_rx_s *packets,
    int count
) {
    // serialize Lora packets metadata and payload
    int packetCountInDatagram = 0;
    for (int i = 0; i < count; ++i) {
        lgw_pkt_rx_s *p = &packets[i];
        if (validateMetadata(p)) {
            if (onUpstream)
                onUpstream(this, p);
            packetCountInDatagram++;
        }
    }
    // do not wait for ACK, let say it received
    return packetCountInDatagram;
}

/**
 * Receive Lora packets from end-device(s)
 */
void LoraGatewayListener::upstreamDownstreamLoop()
{
    log(LOG_DEBUG, LOG_EMBEDDED_GATEWAY, MSG_UPSTREAM_STARTED);
    while (state != 2) {
        struct lgw_pkt_rx_s rxpkt[PACKETS_MAX_SIZE]; // array containing inbound packets + metadata
        // fetch packets
        mutexLgw.lock();
        int receivedPacketsCount = lgw_receive(PACKETS_MAX_SIZE, rxpkt);
        mutexLgw.unlock();
        switch(receivedPacketsCount) {
            case LGW_HAL_ERROR:
                log(LOG_ERR, ERR_CODE_LORA_GATEWAY_FETCH, ERR_LORA_GATEWAY_FETCH);
                // fatal error, exit
                stop();
                return;
            case 0:
                doJitDownstream(); // Transmit packets from JIT queue
                wait_ms(UPSTREAM_FETCH_DELAY_MS);
                continue;
            default:
                doUpstream(rxpkt, receivedPacketsCount);
                doJitDownstream();  // Transmit packets from JIT queue
                break;
        }
    }
    state = 0;
    log(LOG_DEBUG, LOG_EMBEDDED_GATEWAY, MSG_UPSTREAM_FINISHED);
}

#define MIN_LORA_PREAMBLE_LEN       6           // minimum Lora preamble length
#define MIN_FSK_PREAMBLE_LEN        3           // minimum FSK preamble length

static uint16_t crc16(
    const uint8_t *data,
    size_t size
) {
    if (!data)
        return 0;
    const uint16_t crc_poly = 0x1021;
    const uint16_t init_val = 0x0000;
    uint16_t r = init_val;

    for (unsigned int i = 0; i < size; ++i) {
        r ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            r = (r & 0x8000) ? (r << 1) ^ crc_poly : (r << 1);
        }
    }
    return r;
}

bool LoraGatewayListener::getTxGainLutIndex(
    uint8_t rf_chain,
    int8_t rf_power,
    uint8_t *lut_index
) const
{
    uint8_t pow_index;
    int current_best_index = -1;
    uint8_t current_best_match = 0xff;
    int diff;

    if (!lut_index)
        return false;

    // Search requested power in TX gain LUT
    for (pow_index = 0; pow_index < config->sx130x.txLut[rf_chain].size; pow_index++) {
        diff = rf_power - config->sx130x.txLut[rf_chain].lut[pow_index].rf_power;
        if (diff < 0) {
            // The selected power must be lower or equal to requested one
            continue;
        } else {
            // Record the index corresponding to the closest rf_power available in LUT
            if ((current_best_index == -1) || (diff < current_best_match)) {
                current_best_match = diff;
                current_best_index = pow_index;
            }
        }
    }

    // Return corresponding index
    if (current_best_index > -1) {
        *lut_index = (uint8_t)current_best_index;
    } else {
        *lut_index = 0;
        return false;
    }
    return true;
}

/**
 * Validate transmission packet in tx param, if packet is valid, enqueueTxPacket packet to be sent or send immediately
 * @param tx
 * @return
 */
int LoraGatewayListener::enqueueTxPacket(
    struct lgw_pkt_tx_s &pkt
)
{
    // determine packet type (class A, B or C)
    enum jit_pkt_type_e downlinkClass;
    // and calculate appropriate time to send
    switch(pkt.tx_mode) {
        case IMMEDIATE:
            // TX procedure: send immediately
            downlinkClass = JIT_PKT_TYPE_DOWNLINK_CLASS_C;
            break;
        case TIMESTAMPED:
            // tx.pkt.count_us is time stamp
            // Concentrator timestamp is given, we consider it is a Class A downlink
            downlinkClass = JIT_PKT_TYPE_DOWNLINK_CLASS_A;
            break;
        case ON_GPS:
        {
            // otherwise send on GPS time (converted to timestamp packet)
            log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SEND_AT_GPS_TIME_DISABLED, ERR_LORA_GATEWAY_SEND_AT_GPS_TIME_DISABLED);
            return ERR_CODE_LORA_GATEWAY_SEND_AT_GPS_TIME_DISABLED;
        }
        default:
            log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_UNKNOWN_TX_MODE, ERR_LORA_GATEWAY_UNKNOWN_TX_MODE);
            return ERR_CODE_LORA_GATEWAY_UNKNOWN_TX_MODE;
    }

    // Validate is channel allowed
    if (!config->sx130x.rfConfs[pkt.rf_chain].tx_enable) {
        log(LOG_ERR, ERR_CODE_LORA_GATEWAY_TX_CHAIN_DISABLED, ERR_LORA_GATEWAY_TX_CHAIN_DISABLED);
        return ERR_CODE_LORA_GATEWAY_TX_CHAIN_DISABLED;
    }

    // Correct radio transmission power
    pkt.rf_power = (int8_t) (pkt.rf_power - config->sx130x.antennaGain);

    // Validate preamble length
    switch (pkt.modulation) {
        case MOD_LORA:
            // Check minimum Lora preamble length
            if (pkt.preamble < MIN_LORA_PREAMBLE_LEN)
                pkt.preamble = MIN_LORA_PREAMBLE_LEN;
            break;
        case MOD_FSK:
            // Check minimum FSK preamble length
            if (pkt.preamble < MIN_FSK_PREAMBLE_LEN)
                pkt.preamble = MIN_FSK_PREAMBLE_LEN;
            break;
        default:
            log(LOG_ERR, ERR_CODE_LORA_GATEWAY_UNKNOWN_MODULATION, ERR_LORA_GATEWAY_UNKNOWN_MODULATION);
            return ERR_CODE_LORA_GATEWAY_UNKNOWN_MODULATION;
    }

    // reset error/warning results

    // check TX frequency before trying to queue packet
    if ((pkt.freq_hz < config->sx130x.tx_freq_min[pkt.rf_chain]) || (pkt.freq_hz > config->sx130x.tx_freq_max[pkt.rf_chain])) {
        // jit_result = JIT_ERROR_TX_FREQ;
        log(LOG_ERR, ERR_CODE_LORA_GATEWAY_TX_UNSUPPORTED_FREQUENCY, ERR_LORA_GATEWAY_TX_UNSUPPORTED_FREQUENCY);
        return ERR_CODE_LORA_GATEWAY_TX_UNSUPPORTED_FREQUENCY;
    }

    // check TX power before trying to queue packet, send a warning if not supported
    uint8_t tx_lut_idx;
    bool r = getTxGainLutIndex(pkt.rf_chain, pkt.rf_power, &tx_lut_idx);
    if ((!r) || (config->sx130x.txLut[pkt.rf_chain].lut[tx_lut_idx].rf_power != pkt.rf_power)) {
        // this RF power is not supported, throw a warning, and use the closest lower power supported
        log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_TX_UNSUPPORTED_POWER, ERR_LORA_GATEWAY_TX_UNSUPPORTED_POWER);
        pkt.rf_power = config->sx130x.txLut[pkt.rf_chain].lut[tx_lut_idx].rf_power;
    }

    // insert packet to be sent into JIT queue
    // previous gw lib version
    uint32_t current_concentrator_time;
    mutexLgw.lock();
    lgw_get_instcnt(&current_concentrator_time);
    mutexLgw.unlock();
    int jit_result = jit_enqueue(&jit_queue[pkt.rf_chain], current_concentrator_time, &pkt, downlinkClass);
    if (jit_result) {
        log(LOG_ERR, ERR_CODE_LORA_GATEWAY_JIT_ENQUEUE_FAILED, ERR_LORA_GATEWAY_JIT_ENQUEUE_FAILED);
        return ERR_CODE_LORA_GATEWAY_JIT_ENQUEUE_FAILED;
    }
    return CODE_OK;
}

LoraGatewayListener::LoraGatewayListener()
    : logVerbosity(0), onLog(nullptr), onUpstream(nullptr), xtal_correct(1.0), state(0),
    config(nullptr), gatewayId(0)
{
    // JIT queue initialization
    jit_queue_init(&jit_queue[0]);
    jit_queue_init(&jit_queue[1]);
}

LoraGatewayListener::~LoraGatewayListener() = default;

int LoraGatewayListener::setup()
{
    if (!config)
        return ERR_CODE_INSUFFICIENT_PARAMS;
    int lastLgwCode = lgw_board_setconf(&config->sx130x.boardConf);
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_CONFIGURE_BOARD_FAILED;
    if (config->sx130x.tsConf.enable) {
        lastLgwCode = lgw_ftime_setconf(&config->sx130x.tsConf);
        if (lastLgwCode)
            return ERR_CODE_LORA_GATEWAY_CONFIGURE_TIME_STAMP;
    }
    lastLgwCode = lgw_sx1261_setconf(&config->sx1261.sx1261);
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_CONFIGURE_SX1261_RADIO;

    for (int i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (config->sx130x.txLut[i].size) {
            lastLgwCode = lgw_txgain_setconf(i, &config->sx130x.txLut[i]);
            if (lastLgwCode)
                return ERR_CODE_LORA_GATEWAY_CONFIGURE_TX_GAIN_LUT;
        }
    }

    for (int i = 0; i < LGW_RF_CHAIN_NB; i++) {
        lastLgwCode = lgw_rxrf_setconf(i, &config->sx130x.rfConfs[i]);
        if (lastLgwCode)
            return ERR_CODE_LORA_GATEWAY_CONFIGURE_INVALID_RADIO;
    }
    lastLgwCode = lgw_demod_setconf(&config->sx130x.demodConf);
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_CONFIGURE_DEMODULATION;

    for (int i = 0; i < LGW_MULTI_NB; i++) {
        lastLgwCode = lgw_rxif_setconf(i, &config->sx130x.ifConfs[i]);
        if (lastLgwCode)
            return ERR_CODE_LORA_GATEWAY_CONFIGURE_MULTI_SF_CHANNEL;
    }
    if (config->sx130x.ifStdConf.enable) {
        lastLgwCode = lgw_rxif_setconf(8, &config->sx130x.ifStdConf);
        if (lastLgwCode)
            return ERR_CODE_LORA_GATEWAY_CONFIGURE_STD_CHANNEL;
    } else; // TODO
    if (config->sx130x.ifStdConf.enable) {
        lastLgwCode = lgw_rxif_setconf(9, &config->sx130x.ifFSKConf);
        if (lastLgwCode)
            return ERR_CODE_LORA_GATEWAY_CONFIGURE_FSK_CHANNEL;
    } else; // TODO
    lastLgwCode = lgw_debug_setconf(&config->debug);
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_CONFIGURE_DEBUG;
    return CODE_OK;
}

std::string LoraGatewayListener::version()
{
    mutexLgw.lock();
    auto r = lgw_version_info();
    mutexLgw.unlock();
    return r;
}

///< SX1302 Concentrator temperature
float LoraGatewayListener::devTemperature()
{
    float r;
    mutexLgw.lock();
    int lastLgwCode = lgw_get_temperature(&r);
    mutexLgw.unlock();
    if (lastLgwCode)
        r = INVALID_TEMPERATURE_C;
    return r;
}

///< SX1302 counter (INST)
uint32_t LoraGatewayListener::devCounterInst()
{
    uint32_t r;
    mutexLgw.lock();
    int lastLgwCode = lgw_get_instcnt(&r);
    mutexLgw.unlock();
    if (lastLgwCode)
        r = 0;
    return r;
}

///< SX1302 counter (PPS)
uint32_t LoraGatewayListener::devCounterTrig()
{
    uint32_t r;
    mutexLgw.lock();
    int lastLgwCode = lgw_get_instcnt(&r);
    mutexLgw.unlock();
    if (lastLgwCode)
        r = 0;
    return r;
}

int LoraGatewayListener::run()
{
    if (!config)
        return ERR_CODE_NO_CONFIG;
    state = 1;  // running
    // load config
    int r = setup();
    if (r)
        return r;
    // starting the concentrator
    int lastLgwCode = lgw_start();
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_START_FAILED;

    // get the concentrator EUI
    lastLgwCode = lgw_get_eui(&gatewayId);
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_GET_EUI;
    if (onLog)
        onLog->onStarted(this, gatewayId, config->name);
    upstreamDownstreamLoop();
    bool success = lgw_stop() == 0;
    if (onLog)
        onLog->onFinished(this, success ? ERR_LORA_GATEWAY_SHUTDOWN_SUCCESS : ERR_LORA_GATEWAY_SHUTDOWN_TIMEOUT);
    return success ? CODE_OK : ERR_CODE_LORA_GATEWAY_SHUTDOWN_TIMEOUT;
}

void LoraGatewayListener::stop()
{
    state = 2;  // stop request
}

void LoraGatewayListener::log(
    int level,
    int errorCode,
    const std::string &message
) const
{
    if (!onLog || (level > logVerbosity))
        return;
    onLog->onInfo((void *) this, LOG_EMBEDDED_GATEWAY, level, errorCode, message);
}

void LoraGatewayListener::setOnLog(
    LogIntf *value,
    int level
)
{
    onLog = value;
    logVerbosity = level;
}

void LoraGatewayListener::setConfig(
        LorawanGatewaySettings *value
)
{
    config = value;
}

void LoraGatewayListener::setOnUpstream(
    std::function<void(
        const LoraGatewayListener *listener,
        lgw_pkt_rx_s *packet
    )> value
)
{
    // no prevent mutex required
    onUpstream = value;
}

void LoraGatewayListener::doJitDownstream() {
    for (auto & i : jit_queue) {
        // transfer data and metadata to the concentrator, and schedule TX
        uint32_t current_concentrator_time;
        mutexLgw.lock();
        lgw_get_instcnt(&current_concentrator_time);
        mutexLgw.unlock();
        int jitPacketIndex = -1;
        enum jit_error_e jit_result = jit_peek(&i, current_concentrator_time, &jitPacketIndex);
        if (jit_result == JIT_ERROR_OK) {
            if (jitPacketIndex >= 0) {
                struct lgw_pkt_tx_s pkt{};
                enum jit_pkt_type_e pkt_type;
                jit_result = jit_dequeue(&i, jitPacketIndex, &pkt, &pkt_type);
                if (jit_result == JIT_ERROR_OK) {
                    // update beacon stats
                    if (pkt_type == JIT_PKT_TYPE_BEACON) {
                        // Compensate beacon frequency with xtal error
                        pkt.freq_hz = (uint32_t)(xtal_correct * (double) pkt.freq_hz);

                        // write to log
                        std::stringstream ss;
                        ss << "Beacon pkt " << pkt.freq_hz << " Hz (xtal_correct=" << xtal_correct;
                        log(LOG_INFO, 0, ss.str());

                        // Update statistics
                        // write to log
                        std::stringstream ss2;
                        ss2 << MSG_BEACON_DEQUEUED << pkt.count_us;
                        log(LOG_INFO, 0, ss2.str());
                    }

                    // check if concentrator is free for sending new packet
                    uint8_t tx_status;
                    mutexLgw.lock();
                    int result = lgw_status(pkt.rf_chain, TX_STATUS, &tx_status);
                    mutexLgw.unlock();
                    if (result == LGW_HAL_ERROR) {
                        // write to log
                        log(LOG_INFO, ERR_CODE_LORA_GATEWAY_STATUS_FAILED, ERR_LORA_GATEWAY_STATUS_FAILED);
                    } else {
                        if (tx_status == TX_EMITTING) {
                            // write to log
                            log(LOG_INFO, ERR_CODE_LORA_GATEWAY_EMIT_ALLREADY, ERR_LORA_GATEWAY_EMIT_ALLREADY);
                            continue;
                        } else if (tx_status == TX_SCHEDULED) {
                            // write to log
                            log(LOG_INFO, ERR_CODE_LORA_GATEWAY_SCHEDULED_ALLREADY, ERR_LORA_GATEWAY_SCHEDULED_ALLREADY);
                        }
                    }

                    // send packet to concentrator
                    mutexLgw.lock();
                    result = lgw_send(&pkt);
                    mutexLgw.unlock();
                    if (result != LGW_HAL_SUCCESS) {
                        // write to log
                        log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SEND_FAILED, ERR_LORA_GATEWAY_SEND_FAILED);
                        continue;
                    } else {
                        // write to log
                        log(LOG_INFO, ERR_CODE_LORA_GATEWAY_SENT, ERR_LORA_GATEWAY_SENT);
                    }
                } else {
                    // write to log
                    log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_JIT_DEQUEUE_FAILED, ERR_LORA_GATEWAY_JIT_DEQUEUE_FAILED);
                }
            }
        } else if (jit_result == JIT_ERROR_EMPTY) {
            // Do nothing, it can happen
        } else {
            // write to log
            log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_JIT_PEEK_FAILED, ERR_LORA_GATEWAY_JIT_PEEK_FAILED);
        }
    }
}
