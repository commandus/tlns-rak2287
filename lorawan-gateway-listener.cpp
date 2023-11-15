/*
 * C++ wrapper of lora_pkt_fwd.c (C)2019 Semtech License: Revised BSD License, see LICENSE.TXT file include in the project
 */
#include <iomanip>
#include <thread>
#include <cmath>
#include <cstring>
#include <sstream>

#include "lorawan-gateway-listener.h"
#include "lorawan-error.h"
#include "lorawan-msg.h"

// XTAL correction constants
#define GPS_REF_MAX_AGE     30          // maximum admitted delay in seconds of GPS loss before considering latest GPS sync unusable
#define XERR_INIT_AVG       16          // number of measurements the XTAL correction is averaged on as initial value
#define XERR_FILT_COEF      256         // coefficient for low-pass XTAL error tracking

#define SPECTRAL_SCAN_CHECK_STATUS_DELAY_MS   10
#define JIT_DELAY               10
#define SPECTRAL_SCAN_DELAY_MS  1000
#define GPS_DELAY_MS            1000

TxPacket::TxPacket()
{
    memset(&pkt, 0, sizeof(struct lgw_pkt_tx_s));
}

static const char *DEF_GPS_FAMILY = "ubx7";

void LoraGatewayListener::spectralScanRunner()
{
    if (!config)
        return;
    log(LOG_DEBUG, LOG_EMBEDDED_GATEWAY, MSG_SPECTRAL_SCAN_STARTED);

    uint32_t freqHz = config->sx1261.spectralScan.freq_hz_start;
    uint32_t freqHzStop = config->sx1261.spectralScan.freq_hz_start + config->sx1261.spectralScan.nb_chan * 200E3;
    int16_t levels[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    uint16_t results[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    struct timeval startTime;
    lgw_spectral_scan_status_t status;
    uint8_t tx_status = TX_FREE;
    bool spectralScanStarted;

    while (!stopRequest) {
        // Pace the scan thread (1 sec min), and avoid waiting several seconds when exit
        for (int i = 0; i < (int)(config->sx1261.spectralScan.pace_s ? config->sx1261.spectralScan.pace_s : 1); i++) {
            if (stopRequest)
                break;
            wait_ms(SPECTRAL_SCAN_DELAY_MS);
        }
        spectralScanStarted = false;

        // Start spectral scan (if no downlink programmed)
        mLGW.lock();
        // Check if there is a downlink programmed
        for (int i = 0; i < LGW_RF_CHAIN_NB; i++) {
            if (config->sx130x.rfConfs[i].tx_enable) {
                int x = lgw_status((uint8_t)i, TX_STATUS, &tx_status);
                if (x != LGW_HAL_SUCCESS) {
                    log(LOG_ERR, ERR_CODE_LORA_GATEWAY_GET_TX_STATUS, ERR_LORA_GATEWAY_GET_TX_STATUS);
                } else {
                    if (tx_status == TX_SCHEDULED || tx_status == TX_EMITTING) {
                        log(LOG_ERR, ERR_CODE_LORA_GATEWAY_SKIP_SPECTRAL_SCAN, ERR_LORA_GATEWAY_SKIP_SPECTRAL_SCAN);
                        break;
                    }
                }
            }
        }
        if (tx_status != TX_SCHEDULED && tx_status != TX_EMITTING) {
            int x = lgw_spectral_scan_start(freqHz, config->sx1261.spectralScan.nb_scan);
            if (x != 0) {
                mLGW.unlock();
                // write to log
                log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SPECTRAL_SCAN_START_FAILED, ERR_LORA_GATEWAY_SPECTRAL_SCAN_START_FAILED);
                continue;
            }
            spectralScanStarted = true;
        }
        mLGW.unlock();
        if (spectralScanStarted) {
            // Wait for scan to be completed
            status = LGW_SPECTRAL_SCAN_STATUS_UNKNOWN;
            timeout_start(&startTime);
            do {
                // handle timeout
                if (timeout_check(startTime, 2000) != 0) {
                    // write to log
                    log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SPECTRAL_SCAN_TIMEOUT, ERR_LORA_GATEWAY_SPECTRAL_SCAN_TIMEOUT);
                    break;
                }

                // get spectral scan status
                mLGW.lock();
                int x = lgw_spectral_scan_get_status(&status);
                mLGW.unlock();
                if (x != 0) {
                    // write to log
                    log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SPECTRAL_SCAN_FAILED, ERR_LORA_GATEWAY_SPECTRAL_SCAN_FAILED);
                    break;
                }

                // wait a bit before checking status again
                wait_ms(SPECTRAL_SCAN_CHECK_STATUS_DELAY_MS);
            } while (status != LGW_SPECTRAL_SCAN_STATUS_COMPLETED && status != LGW_SPECTRAL_SCAN_STATUS_ABORTED);
            if (status == LGW_SPECTRAL_SCAN_STATUS_COMPLETED) {
                // Get spectral scan results
                memset(levels, 0, sizeof levels);
                memset(results, 0, sizeof results);
                mLGW.lock();
                int x = lgw_spectral_scan_get_results(levels, results);
                mLGW.unlock();
                if (x != 0) {
                    log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SPECTRAL_SCAN_RESULT, ERR_LORA_GATEWAY_SPECTRAL_SCAN_RESULT);
                    continue; // main while loop
                }
                // print results
                // prevent change onSpectralScan
                mReportSpectralScan.lock();
                if (onSpectralScan) {
                    onSpectralScan(this, freqHz, results);
                }
                mReportSpectralScan.unlock();
                // Next frequency to scan
                freqHz += 200000; // 200kHz channels
                if (freqHz >= freqHzStop) {
                    freqHz = config->sx1261.spectralScan.freq_hz_start;
                }
            } else if (status == LGW_SPECTRAL_SCAN_STATUS_ABORTED) {
                // write to log
                log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SPECTRAL_SCAN_ABORTED, ERR_LORA_GATEWAY_SPECTRAL_SCAN_ABORTED);
            } else {
                // write to log
                log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SPECTRAL_SCAN_UNEXPECTED_STATUS, ERR_LORA_GATEWAY_SPECTRAL_SCAN_UNEXPECTED_STATUS);
            }
        }
    }
    log(LOG_DEBUG, LOG_EMBEDDED_GATEWAY, MSG_SPECTRAL_SCAN_FINISHED);
    spectralScanThreadRunning = false;
}

#define STATUS_SIZE        200
// max number of packets per fetch/send cycle
#define NB_PKT_MAX         255
#define TX_BUFF_SIZE       ((540 * NB_PKT_MAX) + 30 + STATUS_SIZE)
// ms waited when a fetch return no packets
#define UPSTREAM_FETCH_DELAY_MS     10
#define GATEWAY_PROTOCOL    2
#define UNIX_GPS_EPOCH_OFFSET 315964800 // Number of seconds ellapsed between 01.Jan.1970 00:00:00 and 06.Jan.1980 00:00:00

/**
 * Receive Lora packets from end-device(s)
 */
void LoraGatewayListener::upstreamRunner()
{
    SEMTECH_PROTOCOL_METADATA metadata;
    std::string payload;

    metadata.gatewayId = config->gateway.gatewayId;

    log(LOG_DEBUG, LOG_EMBEDDED_GATEWAY, MSG_UPSTREAM_STARTED);

    // allocate memory for metadata fetching and processing
    struct lgw_pkt_rx_s rxpkt[NB_PKT_MAX]; // array containing inbound packets + metadata
    struct lgw_pkt_rx_s *p; // pointer on a RX metadata

    // local copy of GPS time reference
    bool ref_ok = false; // determine if GPS time reference must be used or not
    struct tref local_ref; // time reference used for UTC <-> timestamp conversion

    struct timespec recv_time;

    while (!stopRequest) {
        // fetch packets
        mLGW.lock();
        int nb_pkt = lgw_receive(NB_PKT_MAX, rxpkt);
        mLGW.unlock();
        if (nb_pkt == LGW_HAL_ERROR) {
            log(LOG_ERR, ERR_CODE_LORA_GATEWAY_FETCH, ERR_LORA_GATEWAY_FETCH);
            // fatal error, exit
            stop(0);
            return;
        }

        // wait a short time if no packets, nor status report
        if (nb_pkt == 0) {
            wait_ms(UPSTREAM_FETCH_DELAY_MS);
            continue;
        }

        ref_ok = false;

        // serialize Lora packets metadata and payload
        int pkt_in_dgram = 0;
        for (int i = 0; i < nb_pkt; ++i) {
            p = &rxpkt[i];
            // basic metadata filtering

            switch(p->status) {
                case STAT_CRC_OK:
                    if (!config->gateway.forwardCRCValid) {
                        continue; // skip that metadata
                    }
                    break;
                case STAT_CRC_BAD:
                    if (!config->gateway.forwardCRCError) {
                        continue; // skip that metadata
                    }
                    break;
                case STAT_NO_CRC:
                    if (!config->gateway.forwardCRCDisabled) {
                        continue; // skip that metadata
                    }
                    break;
                default:
                    log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_UNKNOWN_STATUS, ERR_LORA_GATEWAY_UNKNOWN_STATUS);
                    continue; // skip that metadata
                    // exit(EXIT_FAILURE);
            }

            // log(LOG_INFO, ERR_CODE_LORA_GATEWAY_RECEIVED, ERR_LORA_GATEWAY_RECEIVED);

            // time
            metadata.tmst = p->count_us;

            if (ref_ok) {
                // convert metadata timestamp to UTC absolute time
                struct timespec pkt_utc_time;
                int r = lgw_cnt2utc(local_ref, p->count_us, &pkt_utc_time);
                if (r == LGW_GPS_SUCCESS)
                    metadata.t = pkt_utc_time.tv_sec;
                else
                    metadata.t = 0;
                // convert metadata timestamp to GPS absolute time
                struct timespec pkt_gps_time;
                r = lgw_cnt2gps(local_ref, p->count_us, &pkt_gps_time);
                if (r == LGW_GPS_SUCCESS) {
                    // uint64_t pkt_gps_time_ms = pkt_gps_time.tv_sec * 1E3 + pkt_gps_time.tv_nsec / 1E6;
                    metadata.tmst = pkt_gps_time.tv_sec;
                }
            } else {
                metadata.t = time(nullptr);
            }

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
                    continue;
                    return;
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
                        continue;
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
                        continue;
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
                        continue;
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
                continue;
            }

            payload = std::string((char *)p->payload, p->size);
            pkt_in_dgram++;
        }

        // restart fetch sequence without empty call if all packets have been filtered out
        if (pkt_in_dgram == 0)
            continue;

        // log received message (payload ciphered)
        if (onLog) {
            Payload p;
            p.received = metadata.tmst;
            p.frequency = metadata.freq;
            p.rssi = metadata.rssi;
            p.lsnr = metadata.lsnr;
            p.payload = payload;
            onLog->onReceive(p);
        }

        // send to the network server, network server must call onValue
        if (onUpstream)
            onUpstream(this, &metadata, payload);

        // do not wait for ACK, let say it received
    }
    upstreamThreadRunning = false;
}

#define PROTOCOL_VERSION            2           // v1.6
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

static double difftimespec(
    struct timespec end,
    struct timespec beginning
)
{
    double r;
    r = 1E-9 * (double)(end.tv_nsec - beginning.tv_nsec);
    r += (double)(end.tv_sec - beginning.tv_sec);
    return r;
}

bool LoraGatewayListener::getTxGainLutIndex(
    uint8_t rf_chain,
    int8_t rf_power,
    uint8_t *lut_index
)
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
    TxPacket &tx
)
{
    // determine packet type (class A, B or C)
    enum jit_pkt_type_e downlinkClass;
    // and calculate appropriate time to send
    switch(tx.pkt.tx_mode) {
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
            // tx.pkt.count_us is time stamp
            struct tref local_ref; // time reference used for UTC <-> timestamp conversion
            log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SEND_AT_GPS_TIME_DISABLED, ERR_LORA_GATEWAY_SEND_AT_GPS_TIME_DISABLED);
            // TODO inform network server
            return ERR_CODE_LORA_GATEWAY_SEND_AT_GPS_TIME_DISABLED;

            // Convert GPS time from milliseconds to timespec
            double x4;
            double x3 = modf((double) tx.pkt.count_us / 1E3, &x4);
            struct timespec gps_tx; // GPS time that needs to be converted to timestamp
            gps_tx.tv_sec = (time_t) x4; // get seconds from integer part
            gps_tx.tv_nsec = (long) (x3 * 1E9); // get nanoseconds from fractional part

            // transform GPS time to timestamp
            int r = lgw_gps2cnt(local_ref, gps_tx, &(tx.pkt.count_us));
            if (r != LGW_GPS_SUCCESS) {
                log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SEND_AT_GPS_TIME_INVALID, ERR_LORA_GATEWAY_SEND_AT_GPS_TIME_INVALID);
                return ERR_CODE_LORA_GATEWAY_SEND_AT_GPS_TIME_INVALID;
            } else {
                log(LOG_INFO, 0, MSG_LORA_GATEWAY_SEND_AT_GPS_TIME);
            }
            // GPS timestamp is given, we consider it is a Class B downlink
            downlinkClass = JIT_PKT_TYPE_DOWNLINK_CLASS_B;
        }
        default:
            log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_UNKNOWN_TX_MODE, ERR_LORA_GATEWAY_UNKNOWN_TX_MODE);
            return ERR_CODE_LORA_GATEWAY_UNKNOWN_TX_MODE;
    }

    // Validate is channel allowed
    if (!config->sx130x.rfConfs[tx.pkt.rf_chain].tx_enable) {
        log(LOG_ERR, ERR_CODE_LORA_GATEWAY_TX_CHAIN_DISABLED, ERR_LORA_GATEWAY_TX_CHAIN_DISABLED);
        return ERR_CODE_LORA_GATEWAY_TX_CHAIN_DISABLED;
    }

    // Correct radio transmission power
    tx.pkt.rf_power -= config->sx130x.antennaGain;

    // Validate preamble length
    switch (tx.pkt.modulation) {
        case MOD_LORA:
            // Check minimum Lora preamble length
            if (tx.pkt.preamble < MIN_LORA_PREAMBLE_LEN)
                tx.pkt.preamble = MIN_LORA_PREAMBLE_LEN;
            break;
        case MOD_FSK:
            // Check minimum FSK preamble length
            if (tx.pkt.preamble < MIN_FSK_PREAMBLE_LEN)
                tx.pkt.preamble = MIN_FSK_PREAMBLE_LEN;
            break;
        default:
            log(LOG_ERR, ERR_CODE_LORA_GATEWAY_UNKNOWN_MODULATION, ERR_LORA_GATEWAY_UNKNOWN_MODULATION);
            return ERR_CODE_LORA_GATEWAY_UNKNOWN_MODULATION;
            break;
    }

    // record measurement data

    // reset error/warning results
    int jit_result = JIT_ERROR_OK;
    int warning_result = JIT_ERROR_OK;

    // check TX frequency before trying to queue packet
    if ((tx.pkt.freq_hz < config->sx130x.tx_freq_min[tx.pkt.rf_chain]) || (tx.pkt.freq_hz > config->sx130x.tx_freq_max[tx.pkt.rf_chain])) {
        jit_result = JIT_ERROR_TX_FREQ;
        log(LOG_ERR, ERR_CODE_LORA_GATEWAY_TX_UNSUPPORTED_FREQUENCY, ERR_LORA_GATEWAY_TX_UNSUPPORTED_FREQUENCY);
        return ERR_CODE_LORA_GATEWAY_TX_UNSUPPORTED_FREQUENCY;
    }

    // check TX power before trying to queue packet, send a warning if not supported
    if (jit_result == JIT_ERROR_OK) {
        uint8_t tx_lut_idx;
        int r = getTxGainLutIndex(tx.pkt.rf_chain, tx.pkt.rf_power, &tx_lut_idx);
        if ((r < 0) || (config->sx130x.txLut[tx.pkt.rf_chain].lut[tx_lut_idx].rf_power != tx.pkt.rf_power)) {
            // this RF power is not supported, throw a warning, and use the closest lower power supported
            warning_result = JIT_ERROR_TX_POWER;
            log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_TX_UNSUPPORTED_POWER, ERR_LORA_GATEWAY_TX_UNSUPPORTED_POWER);
            tx.pkt.rf_power = config->sx130x.txLut[tx.pkt.rf_chain].lut[tx_lut_idx].rf_power;
        }
    }

    // insert packet to be sent into JIT queue
    if (jit_result == JIT_ERROR_OK) {
        mLGW.lock();
        // previous gw lib version
        uint32_t current_concentrator_time;
        lgw_get_instcnt(&current_concentrator_time);
        lgw_get_instcnt(&current_concentrator_time);
        mLGW.unlock();
        jit_result = jit_enqueue(&jit_queue[tx.pkt.rf_chain], current_concentrator_time, &tx.pkt, downlinkClass);
        if (jit_result) {
            switch (jit_result) {
                case JIT_ERROR_TOO_EARLY:
                    break;
                case JIT_ERROR_TOO_LATE:
                    break;
                case JIT_ERROR_COLLISION_PACKET:
                    break;
                default:
                    break;
            }

            log(LOG_ERR, ERR_CODE_LORA_GATEWAY_JIT_ENQUEUE_FAILED, ERR_LORA_GATEWAY_JIT_ENQUEUE_FAILED);
            return ERR_CODE_LORA_GATEWAY_TX_UNSUPPORTED_FREQUENCY;
        } else {
            // In case of a warning having been raised before, we notify it
            jit_result = warning_result;
        }
    }
    return CODE_OK;
}

/**
 * Transmit packets from JIT queue
 **/
void LoraGatewayListener::jitRunner() {
    log(LOG_DEBUG, LOG_EMBEDDED_GATEWAY, MSG_JIT_QUEUE_STARTED);
    int result = LGW_HAL_SUCCESS;
    struct lgw_pkt_tx_s pkt;
    int pkt_index = -1;
    uint32_t current_concentrator_time;
    enum jit_error_e jit_result;
    enum jit_pkt_type_e pkt_type;
    uint8_t tx_status;

    while (!stopRequest) {
        wait_ms(JIT_DELAY);
        for (int i = 0; i < LGW_RF_CHAIN_NB; i++) {
            // transfer data and metadata to the concentrator, and schedule TX
            mLGW.lock();
            lgw_get_instcnt(&current_concentrator_time);
            mLGW.unlock();
            jit_result = jit_peek(&jit_queue[i], current_concentrator_time, &pkt_index);
            if (jit_result == JIT_ERROR_OK) {
                if (pkt_index > -1) {
                    jit_result = jit_dequeue(&jit_queue[i], pkt_index, &pkt, &pkt_type);
                    if (jit_result == JIT_ERROR_OK) {
                        // update beacon stats
                        if (pkt_type == JIT_PKT_TYPE_BEACON) {
                            // Compensate beacon frequency with xtal error
                            mXTALcorrection.lock();    // prevent xtal_correct
                            pkt.freq_hz = (uint32_t)(xtal_correct * (double) pkt.freq_hz);
                            mXTALcorrection.unlock();

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
                        mLGW.lock(); // may have to wait for a fetch to finish
                        result = lgw_status(pkt.rf_chain, TX_STATUS, &tx_status);
                        mLGW.unlock(); // free concentrator ASAP
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
                        mLGW.lock(); // may have to wait for a fetch to finish
                        if (config->sx1261.spectralScan.enable) {
                            result = lgw_spectral_scan_abort();
                            if (result) {
                                // write to log
                                log(LOG_WARNING, ERR_CODE_LORA_GATEWAY_SPECTRAL_SCAN_ABORT_FAILED, ERR_LORA_GATEWAY_SPECTRAL_SCAN_ABORT_FAILED);
                            }
                        }
                        result = lgw_send(&pkt);
                        mLGW.unlock(); // free concentrator ASAP
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
    jitThreadRunning = false;
    log(LOG_DEBUG, LOG_EMBEDDED_GATEWAY, MSG_JIT_QUEUE_FINISHED);
}

LoraGatewayListener::LoraGatewayListener()
    : logVerbosity(0), onUpstream(nullptr), onSpectralScan(nullptr), onLog(nullptr), stopRequest(false),
      upstreamThreadRunning(false), jitThreadRunning(false),
      spectralScanThreadRunning(false),
      lastLgwCode(0), config(nullptr), flags(0), eui(0),
      xtal_correct_ok(false), xtal_correct(1.0)

{
    // JIT queue initialization
    jit_queue_init(&jit_queue[0]);
    jit_queue_init(&jit_queue[1]);
}

LoraGatewayListener::LoraGatewayListener(LorawanGatewaySettings *cfg)
    : LoraGatewayListener()
{
    config = cfg;
}

LoraGatewayListener::~LoraGatewayListener()
{
}

void LoraGatewayListener::setLogVerbosity(
    int level
)
{
    logVerbosity = level;
}

int LoraGatewayListener::setup()
{
    lastLgwCode = 0;
    if (!config)
        return ERR_CODE_INSUFFICIENT_PARAMS;
    lastLgwCode = lgw_board_setconf(&config->sx130x.boardConf);
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
    return std::string(lgw_version_info());
}

bool LoraGatewayListener::getStatus(LGWStatus &status)
{
    mLGW.lock();
    lastLgwCode = lgw_get_temperature(&status.temperature);
    if (!lastLgwCode) {
        lastLgwCode = lgw_get_instcnt(&status.inst_tstamp);
        if (!lastLgwCode)
            lastLgwCode = lgw_get_trigcnt(&status.trig_tstamp);
    }
    mLGW.unlock();
    return lastLgwCode == 0;
}

int LoraGatewayListener::start()
{
    if (!config)
        return ERR_CODE_NO_CONFIG;
    stopRequest = false;
    lastLgwCode = 0;
    // load config
    int r = setup();
    if (r)
        return r;

    // starting the concentrator
    lastLgwCode = lgw_start();
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_START_FAILED;

    // get the concentrator EUI
    lastLgwCode = lgw_get_eui(&eui);
    if (lastLgwCode)
        return ERR_CODE_LORA_GATEWAY_GET_EUI;

    if (!upstreamThreadRunning) {
        // set indicator on in the main thread (thread may run after isStopped() call)
        upstreamThreadRunning = true;
        std::thread upstreamThread(&LoraGatewayListener::upstreamRunner, this);
        upstreamThread.detach();
    }

    if ((flags & FLAG_GATEWAY_LISTENER_NO_SEND) == 0) {
        if (!jitThreadRunning) {
            jitThreadRunning = true;
            std::thread jitThread(&LoraGatewayListener::jitRunner, this);
            jitThread.detach();
        }
    } else
        jitThreadRunning = false;

    if (config->sx1261.spectralScan.enable) {
        if (!spectralScanThreadRunning) {
            spectralScanThreadRunning = true;
            std::thread spectralScanThread(&LoraGatewayListener::spectralScanRunner, this);
            spectralScanThread.detach();
        }
    }

    if (onLog)
        onLog->onStarted(eui, config->name, 0);
    return 0;
}

bool LoraGatewayListener::isRunning() const
{
    return upstreamThreadRunning
        && (flags & FLAG_GATEWAY_LISTENER_NO_BEACON)
        && ((flags & FLAG_GATEWAY_LISTENER_NO_SEND) || jitThreadRunning)
        && ((!config) || (!config->sx1261.spectralScan.enable) || spectralScanThreadRunning);
}

bool LoraGatewayListener::isStopped() const
{
    return (!upstreamThreadRunning)
       && (!jitThreadRunning)
       && (!spectralScanThreadRunning);
}

#define DEF_WAIT_SECONDS    60

int LoraGatewayListener::stop(int waitSeconds)
{
    if (stopRequest)
        return ERR_CODE_LORA_GATEWAY_STOP_FAILED;
    // wait threads up to 60s
    if (waitSeconds <= 0)
        waitSeconds = DEF_WAIT_SECONDS;

    stopRequest = true;
    // wait threads
    bool success = false;

    for (int i = 0; i < waitSeconds; i++) {
        if (!isStopped()) {
            sleep(1);
            continue;
        }
        success = true;
    }

    success &= lgw_stop() == 0;
    // force close
    upstreamThreadRunning = false;
    jitThreadRunning = false;
    spectralScanThreadRunning = false;

    if (onStop) {
        onStop(this, success);
    }
    if (onLog)
        onLog->onFinished(success ? ERR_LORA_GATEWAY_SHUTDOWN_SUCCESS : ERR_LORA_GATEWAY_SHUTDOWN_TIMEOUT);
    return success ? CODE_OK : ERR_CODE_LORA_GATEWAY_SHUTDOWN_TIMEOUT;
}

void LoraGatewayListener::log(
    int level,
    int errorCode,
    const std::string &message
) const
{
    if (!onLog || (level > logVerbosity))
        return;
    mLog.lock();
    onLog->onInfo((void *) this, LOG_EMBEDDED_GATEWAY, level, errorCode, message);
    mLog.unlock();
}

void LoraGatewayListener::setOnSpectralScan(
    std::function<void(
        const LoraGatewayListener *listener,
        const uint32_t frequency,
        const uint16_t results[LGW_SPECTRAL_SCAN_RESULT_SIZE]
    )> value)
{
    mReportSpectralScan.lock();
    onSpectralScan = value;
    mReportSpectralScan.unlock();
}

void LoraGatewayListener::setOnLog(
    LogIntf *value
)
{
    mLog.lock();
    onLog = value;
    mLog.unlock();
}

void LoraGatewayListener::setOnUpstream(
    std::function<void(
        const LoraGatewayListener *listener,
        const SEMTECH_PROTOCOL_METADATA *metadata,
        const std::string &payload
    )> value
)
{
    // no prevent mutex required
    onUpstream = value;
}

void LoraGatewayListener::setOnStop(
    std::function<void(
        const LoraGatewayListener *listener,
        bool gracefullyStopped
    )> value
)
{
    onStop = value;
}
