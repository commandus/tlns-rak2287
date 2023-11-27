#ifndef GATEWAY_LORA
#define GATEWAY_LORA 1

/*
    Semtech's Lora concentrator configuration structures
    @see gateway-settings.h
*/
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#ifdef __GNUC__
#define restrict __restrict__ // G++ has restrict
#else
#define restrict // C++ in general doesn't
#endif

// X/Open 7, incorporating POSIX 2017
#define _XOPEN_SOURCE 700

#endif

// Structures from the loragw_hal.h

#define LGW_RF_CHAIN_NB     2                   /* number of RF chains */

/* Listen-Before-Talk */
#define LGW_LBT_CHANNEL_NB_MAX 16 /* Maximum number of LBT channels */

/* Maximum size of Tx gain LUT */
#define TX_GAIN_LUT_SIZE_MAX 16

#define LGW_MULTI_NB        8       /* number of LoRa 'multi SF' chains */

/* values available for the 'status' parameter */
/* NOTE: values according to hardware specification */
#define STAT_UNDEFINED  0x00
#define STAT_NO_CRC     0x01
#define STAT_CRC_BAD    0x11
#define STAT_CRC_OK     0x10

/* values available for the 'modulation' parameters */
/* NOTE: arbitrary values */
#define MOD_UNDEFINED   0
#define MOD_CW          0x08
#define MOD_LORA        0x10
#define MOD_FSK         0x20

/* values available for the 'modulation' parameters */
/* NOTE: arbitrary values */
#define MOD_UNDEFINED   0
#define MOD_CW          0x08
#define MOD_LORA        0x10
#define MOD_FSK         0x20

/* values available for the 'datarate' parameters */
/* NOTE: LoRa values used directly to code SF bitmask in 'multi' modem, do not change */
#define DR_UNDEFINED    0
#define DR_LORA_SF5     5
#define DR_LORA_SF6     6
#define DR_LORA_SF7     7
#define DR_LORA_SF8     8
#define DR_LORA_SF9     9
#define DR_LORA_SF10    10
#define DR_LORA_SF11    11
#define DR_LORA_SF12    12
/* NOTE: for FSK directly use baudrate between 500 bauds and 250 kbauds */
#define DR_FSK_MIN      500
#define DR_FSK_MAX      250000

/* values available for the 'bandwidth' parameters (LoRa & FSK) */
/* NOTE: directly encode FSK RX bandwidth, do not change */
#define BW_UNDEFINED    0
#define BW_500KHZ       0x06
#define BW_250KHZ       0x05
#define BW_125KHZ       0x04

/* values available for the 'coderate' parameters (LoRa only) */
/* NOTE: arbitrary values */
#define CR_UNDEFINED    0   /* CR0 exists but is not recommended, so consider it as invalid */
#define CR_LORA_4_5     0x01
#define CR_LORA_4_6     0x02
#define CR_LORA_4_7     0x03
#define CR_LORA_4_8     0x04

/* return status code */
#define LGW_HAL_SUCCESS     0
#define LGW_HAL_ERROR       (-1)
#define LGW_LBT_NOT_ALLOWED 1

/* status code for TX_STATUS */
/* NOTE: arbitrary values */
#define TX_STATUS_UNKNOWN   0
#define TX_OFF              1    /* TX modem disabled, it will ignore commands */
#define TX_FREE             2    /* TX modem is free, ready to receive a command */
#define TX_SCHEDULED        3    /* TX modem is loaded, ready to send the packet after an event and/or delay */
#define TX_EMITTING         4    /* TX modem is emitting */

/* values available for 'select' in the status function */
#define TX_STATUS       1
#define RX_STATUS       2

#define IS_LORA_BW(bw)          (((bw) == BW_125KHZ) || ((bw) == BW_250KHZ) || ((bw) == BW_500KHZ))
#define IS_LORA_DR(dr)          (((dr) == DR_LORA_SF5) || ((dr) == DR_LORA_SF6) || ((dr) == DR_LORA_SF7) || ((dr) == DR_LORA_SF8) || ((dr) == DR_LORA_SF9) || ((dr) == DR_LORA_SF10) || ((dr) == DR_LORA_SF11) || ((dr) == DR_LORA_SF12))
#define IS_LORA_CR(cr)          (((cr) == CR_LORA_4_5) || ((cr) == CR_LORA_4_6) || ((cr) == CR_LORA_4_7) || ((cr) == CR_LORA_4_8))

#define IS_FSK_BW(bw)           (((bw) >= 1) && ((bw) <= 7))
#define IS_FSK_DR(dr)           (((dr) >= DR_FSK_MIN) && ((dr) <= DR_FSK_MAX))

#define IS_TX_MODE(mode)        (((mode) == IMMEDIATE) || ((mode) == TIMESTAMPED) || ((mode) == ON_GPS))

/**
@struct lgw_rssi_tcomp_s
@brief Structure containing all coefficients necessary to compute the offset to be applied on RSSI for current temperature
*/
struct lgw_rssi_tcomp_s {
    float coeff_a;
    float coeff_b;
    float coeff_c;
    float coeff_d;
    float coeff_e;
};

/**
@enum lgw_radio_type_t
@brief Radio types that can be found on the LoRa Gateway
*/
typedef enum {
    LGW_RADIO_TYPE_NONE,
    LGW_RADIO_TYPE_SX1255,
    LGW_RADIO_TYPE_SX1257,
    LGW_RADIO_TYPE_SX1272,
    LGW_RADIO_TYPE_SX1276,
    LGW_RADIO_TYPE_SX1250
} lgw_radio_type_t;

/**
@struct lgw_conf_rxrf_s
@brief Configuration structure for a RF chain
*/
struct lgw_conf_rxrf_s {
    bool                    enable;             /*!> enable or disable that RF chain */
    uint32_t                freq_hz;            /*!> center frequency of the radio in Hz */
    float                   rssi_offset;        /*!> Board-specific RSSI correction factor */
    struct lgw_rssi_tcomp_s rssi_tcomp;         /*!> Board-specific RSSI temperature compensation coefficients */
    lgw_radio_type_t        type;               /*!> Radio type for that RF chain (SX1255, SX1257....) */
    bool                    tx_enable;          /*!> enable or disable TX on that RF chain */
    bool                    single_input_mode;  /*!> Configure the radio in single or differential input mode (SX1250 only) */
};

/**
@enum lgw_lbt_scan_time_t
@brief Radio types that can be found on the LoRa Gateway
*/
typedef enum {
    LGW_LBT_SCAN_TIME_128_US    = 128,
    LGW_LBT_SCAN_TIME_5000_US   = 5000,
} lgw_lbt_scan_time_t;

/**
@brief Structure containing a Listen-Before-Talk channel configuration
*/
struct lgw_conf_chan_lbt_s{
    uint32_t            freq_hz;           /*!> LBT channel frequency */
    uint8_t             bandwidth;         /*!> LBT channel bandwidth */
    lgw_lbt_scan_time_t scan_time_us;      /*!> LBT channel carrier sense time */
    uint16_t            transmit_time_ms;  /*!> LBT channel transmission duration when allowed */
};

/**
@struct lgw_conf_lbt_s
@brief Configuration structure for listen-before-talk
*/
struct lgw_conf_lbt_s {
    bool                        enable;             /*!> enable or disable LBT */
    int8_t                      rssi_target;        /*!> RSSI threshold to detect if channel is busy or not (dBm) */
    uint8_t                     nb_channel;         /*!> number of LBT channels */
    struct lgw_conf_chan_lbt_s  channels[LGW_LBT_CHANNEL_NB_MAX];  /*!> LBT channels configuration */
};

/**
@struct lgw_conf_sx1261_s
@brief Configuration structure for additional SX1261 radio used for LBT and Spectral Scan
*/
struct lgw_conf_sx1261_s {
    bool                        enable;             /*!> enable or disable SX1261 radio */
    char                        spi_path[64];       /*!> Path to access the SPI device to connect to the SX1261 (not used for USB com type) */
    int8_t                      rssi_offset;        /*!> value to be applied to the sx1261 RSSI value (dBm) */
    struct lgw_conf_lbt_s       lbt_conf;           /*!> listen-before-talk configuration */
};

/**
 * spectral scan
 */
typedef struct spectral_scan_s {
    bool enable;            ///< enable spectral scan thread
    uint32_t freq_hz_start; ///< first channel frequency, in Hz
    uint8_t nb_chan;        ///< number of channels to scan (200kHz between each channel)
    uint16_t nb_scan;       ///< number of scan points for each frequency scan
    uint32_t pace_s;        ///< number of seconds between 2 scans in the thread
} spectral_scan_t;

typedef struct sx1261_config_s {
    struct lgw_conf_sx1261_s sx1261;
    spectral_scan_t spectralScan;
    struct lgw_conf_lbt_s lbt;
} sx1261_config_t;

/**
@enum lgw_ftime_mode_t
@brief Fine timestamping modes
*/
typedef enum {
    LGW_FTIME_MODE_HIGH_CAPACITY,   /*!> fine timestamps for SF5 -> SF10 */
    LGW_FTIME_MODE_ALL_SF           /*!> fine timestamps for SF5 -> SF12 */
} lgw_ftime_mode_t;

/**
@struct lgw_conf_ftime_s
@brief Configuration structure for fine timestamping
*/
struct lgw_conf_ftime_s {
    bool enable;              /*!> Enable / Disable fine timestamping */
    lgw_ftime_mode_t mode;    /*!> Fine timestamping mode */
};

typedef enum com_type_e {
    LGW_COM_SPI,
    LGW_COM_USB,
    LGW_COM_UNKNOWN
} lgw_com_type_t;

/**
@struct lgw_conf_board_s
@brief Configuration structure for board specificities
*/
struct lgw_conf_board_s {
    bool            lorawan_public; /*!> Enable ONLY for *public* networks using the LoRa MAC protocol */
    uint8_t         clksrc;         /*!> Index of RF chain which provides clock to concentrator */
    bool            full_duplex;    /*!> Indicates if the gateway operates in full duplex mode or not */
    lgw_com_type_t  com_type;       /*!> The COMmunication interface (SPI/USB) to connect to the SX1302 */
    char            com_path[64];   /*!> Path to access the COM device to connect to the SX1302 */
};

/**
@struct lgw_tx_gain_s
@brief Structure containing all gains of Tx chain
*/
struct lgw_tx_gain_s {
    int8_t  rf_power;   /*!> measured TX power at the board connector, in dBm */
    uint8_t dig_gain;   /*!> (sx125x) 2 bits: control of the digital gain of SX1302 */
    uint8_t pa_gain;    /*!> (sx125x) 2 bits: control of the external PA (SX1302 I/O)
                             (sx1250) 1 bits: enable/disable the external PA (SX1302 I/O) */
    uint8_t dac_gain;   /*!> (sx125x) 2 bits: control of the radio DAC */
    uint8_t mix_gain;   /*!> (sx125x) 4 bits: control of the radio mixer */
    int8_t offset_i;    /*!> (sx125x) calibrated I offset */
    int8_t offset_q;    /*!> (sx125x) calibrated Q offset */
    uint8_t pwr_idx;    /*!> (sx1250) 6 bits: control the radio power index to be used for configuration */
};

/**
@struct lgw_tx_gain_lut_s
@brief Structure defining the Tx gain LUT
*/
struct lgw_tx_gain_lut_s {
    struct lgw_tx_gain_s    lut[TX_GAIN_LUT_SIZE_MAX];  /*!> Array of Tx gain struct */
    uint8_t                 size;                       /*!> Number of LUT indexes */
};

/**
@struct lgw_conf_rxif_s
@brief Configuration structure for an IF chain
*/
struct lgw_conf_rxif_s {
    bool        enable;         /*!> enable or disable that IF chain */
    uint8_t     rf_chain;       /*!> to which RF chain is that IF chain associated */
    int32_t     freq_hz;        /*!> center frequ of the IF chain, relative to RF chain frequency */
    uint8_t     bandwidth;      /*!> RX bandwidth, 0 for default */
    uint32_t    datarate;       /*!> RX datarate, 0 for default */
    uint8_t     sync_word_size; /*!> size of FSK sync word (number of bytes, 0 for default) */
    uint64_t    sync_word;      /*!> FSK sync word (ALIGN RIGHT, eg. 0xC194C1) */
    bool        implicit_hdr;               /*!> LoRa Service implicit header */
    uint8_t     implicit_payload_length;    /*!> LoRa Service implicit header payload length (number of bytes, 0 for default) */
    bool        implicit_crc_en;            /*!> LoRa Service implicit header CRC enable */
    uint8_t     implicit_coderate;          /*!> LoRa Service implicit header coding rate  */
};

/**
@struct lgw_conf_demod_s
@brief Configuration structure for LoRa/FSK demodulators
*/
struct lgw_conf_demod_s {
    uint8_t     multisf_datarate;   /*!> bitmask to enable spreading-factors for correlators (SF12 - SF5) */
};

typedef struct sx130x_config_s {
    struct lgw_conf_board_s boardConf;
    int8_t antennaGain;
    struct lgw_conf_ftime_s tsConf;
    struct lgw_conf_rxrf_s rfConfs[LGW_RF_CHAIN_NB];

    uint32_t tx_freq_min[LGW_RF_CHAIN_NB];
    uint32_t tx_freq_max[LGW_RF_CHAIN_NB];
    struct lgw_tx_gain_lut_s txLut[LGW_RF_CHAIN_NB];
    struct lgw_conf_rxif_s ifConfs[LGW_MULTI_NB];
    struct lgw_conf_rxif_s ifStdConf;
    struct lgw_conf_rxif_s ifFSKConf;
    struct lgw_conf_demod_s demodConf;
} sx130x_config_t;

/**
@struct coord_s
@brief Geodesic coordinates
*/
struct coord_s {
    double  lat;    /*!> latitude [-90,90] (North +, South -) */
    double  lon;    /*!> longitude [-180,180] (East +, West -)*/
    short   alt;    /*!> altitude in meters (WGS 84 geoid ref.) */
};

typedef struct gateway_s {
    uint64_t gatewayId;
    uint16_t serverPortUp;      // not used
    uint16_t serverPortDown;    // not used
    uint32_t keepaliveInterval;
    uint32_t statInterval;
    struct timeval pushTimeoutMs;
    bool forwardCRCValid;
    bool forwardCRCError;
    bool forwardCRCDisabled;
    bool gpsEnabled;
    struct coord_s refGeoCoordinates;
    bool fakeGPS;
    uint32_t beaconPeriod;  ///< send class B beacon period in seconds. 0- do not send beacons, Must be >=6s and <86400
    uint32_t beaconFreqHz;
    uint8_t beaconFreqNb;
    uint32_t beaconFreqStep;
    uint8_t beaconDataRate;
    uint32_t beaconBandwidthHz;
    uint8_t beaconPower;
    uint8_t beaconInfoDesc;
    uint32_t autoQuitThreshold;
} gateway_t;

/**
@struct lgw_conf_debug_s
@brief Configuration structure for debug
*/
struct conf_ref_payload_s {
    uint32_t id;
    uint8_t payload[255];
    uint32_t prev_cnt;
};

struct lgw_conf_debug_s {
    uint8_t                     nb_ref_payload;
    struct conf_ref_payload_s   ref_payload[16];
    char log_file_name[128];
};

typedef struct gateway_settings_s {
    sx1261_config_t sx1261;
    sx130x_config_t sx130x;
    gateway_t gateway;
    struct lgw_conf_debug_s debug;
} gateway_settings_t;

typedef struct gateway_settings_ptr_s {
    sx1261_config_t *sx1261;
    sx130x_config_t *sx130x;
    gateway_t *gateway;
    struct lgw_conf_debug_s *debug;
} gateway_settings_ptr_t;

/*
@struct lgw_pkt_rx_s
@brief Structure containing the metadata of a packet that was received and a pointer to the payload
*/
struct lgw_pkt_rx_s {
    uint32_t    freq_hz;        /*!> central frequency of the IF chain */
    int32_t     freq_offset;
    uint8_t     if_chain;       /*!> by which IF chain was packet received */
    uint8_t     status;         /*!> status of the received packet */
    uint32_t    count_us;       /*!> internal concentrator counter for timestamping, 1 microsecond resolution */
    uint8_t     rf_chain;       /*!> through which RF chain the packet was received */
    uint8_t     modem_id;
    uint8_t     modulation;     /*!> modulation used by the packet */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> RX datarate of the packet (SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    float       rssic;          /*!> average RSSI of the channel in dB */
    float       rssis;          /*!> average RSSI of the signal in dB */
    float       snr;            /*!> average packet SNR, in dB (LoRa only) */
    float       snr_min;        /*!> minimum packet SNR, in dB (LoRa only) */
    float       snr_max;        /*!> maximum packet SNR, in dB (LoRa only) */
    uint16_t    crc;            /*!> CRC that was received in the payload */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t     payload[256];   /*!> buffer containing the payload */
    bool        ftime_received; /*!> a fine timestamp has been received */
    uint32_t    ftime;          /*!> packet fine timestamp (nanoseconds since last PPS) */
};

/**
@struct lgw_pkt_tx_s
@brief Structure containing the configuration of a packet to send and a pointer to the payload
*/
struct lgw_pkt_tx_s {
    uint32_t    freq_hz;        /*!> center frequency of TX */
    uint8_t     tx_mode;        /*!> select on what event/time the TX is triggered */
    uint32_t    count_us;       /*!> timestamp or delay in microseconds for TX trigger */
    uint8_t     rf_chain;       /*!> through which RF chain will the packet be sent */
    int8_t      rf_power;       /*!> TX power, in dBm */
    uint8_t     modulation;     /*!> modulation to use for the packet */
    int8_t      freq_offset;    /*!> frequency offset from Radio Tx frequency (CW mode) */
    uint8_t     bandwidth;      /*!> modulation bandwidth (LoRa only) */
    uint32_t    datarate;       /*!> TX datarate (baudrate for FSK, SF for LoRa) */
    uint8_t     coderate;       /*!> error-correcting code of the packet (LoRa only) */
    bool        invert_pol;     /*!> invert signal polarity, for orthogonal downlinks (LoRa only) */
    uint8_t     f_dev;          /*!> frequency deviation, in kHz (FSK only) */
    uint16_t    preamble;       /*!> set the preamble length, 0 for default */
    bool        no_crc;         /*!> if true, do not send a CRC in the packet */
    bool        no_header;      /*!> if true, enable implicit header mode (LoRa), fixed length (FSK) */
    uint16_t    size;           /*!> payload size in bytes */
    uint8_t     payload[256];   /*!> buffer containing the payload */
};

/**
@brief A non-blocking function that will fetch up to 'max_pkt' packets from the LoRa concentrator FIFO and data buffer
@param max_pkt maximum number of packet that must be retrieved (equal to the size of the array of struct)
@param pkt_data pointer to an array of struct that will receive the packet metadata and payload pointers
@return LGW_HAL_ERROR id the operation failed, else the number of packets retrieved
*/
int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s * pkt_data);

/**
@brief Return instateneous value of internal counter
@param inst_cnt_us pointer to receive timestamp value
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_get_instcnt(uint32_t * inst_cnt_us);

/**
@brief Configure the gateway board
@param conf structure containing the configuration parameters
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_board_setconf(struct lgw_conf_board_s * conf);

/**
@brief Configure the fine timestamping
@param conf pointer to structure defining the config to be applied
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_ftime_setconf(struct lgw_conf_ftime_s * conf);

/*
@brief Configure the SX1261 radio for LBT/Spectral Scan
@param pointer to structure defining the config to be applied
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_sx1261_setconf(struct lgw_conf_sx1261_s * conf);

/**
@brief Configure the Tx gain LUT
@param conf pointer to structure defining the LUT
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_txgain_setconf(uint8_t rf_chain, struct lgw_tx_gain_lut_s * conf);

/**
@brief Configure an RF chain (must configure before start)
@param rf_chain number of the RF chain to configure [0, LGW_RF_CHAIN_NB - 1]
@param conf structure containing the configuration parameters
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s * conf);

/**
@brief Configure LoRa/FSK demodulators
@param conf structure containing the configuration parameters
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_demod_setconf(struct lgw_conf_demod_s * conf);

/**
@brief Configure the debug context
@param conf pointer to structure defining the config to be applied
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_debug_setconf(struct lgw_conf_debug_s * conf);

/**
@brief Connect to the LoRa concentrator, reset it and configure it according to previously set parameters
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_start(void);

/**
@brief Stop the LoRa concentrator and disconnect it
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_stop(void);

/**
@brief Allow user to check the version/options of the library once compiled
@return pointer on a human-readable null terminated string
*/
const char* lgw_version_info(void);

/**
@brief Return the LoRa concentrator EUI
@param eui pointer to receive gatewayId
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_get_eui(uint64_t * eui);

/**
@brief Give the the status of different part of the LoRa concentrator
@param select is used to select what status we want to know
@param code is used to return the status code
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_status(uint8_t rf_chain, uint8_t select, uint8_t * code);

/**
@brief Schedule a packet to be send immediately or after a delay depending on tx_mode
@param pkt_data structure containing the data and metadata for the packet to send
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else

/!\ When sending a packet, there is a delay (approx 1.5ms) for the analog
circuitry to start and be stable. This delay is adjusted by the HAL depending
on the board version (lgw_i_tx_start_delay_us).

In 'timestamp' mode, this is transparent: the modem is started
lgw_i_tx_start_delay_us microseconds before the user-set timestamp value is
reached, the preamble of the packet start right when the internal timestamp
counter reach target value.

In 'immediate' mode, the packet is emitted as soon as possible: transferring the
packet (and its parameters) from the host to the concentrator takes some time,
then there is the lgw_i_tx_start_delay_us, then the packet is emitted.

In 'triggered' mode (aka PPS/GPS mode), the packet, typically a beacon, is
emitted lgw_i_tx_start_delay_us microsenconds after a rising edge of the
trigger signal. Because there is no way to anticipate the triggering event and
start the analog circuitry beforehand, that delay must be taken into account in
the protocol.
*/
int lgw_send(struct lgw_pkt_tx_s * pkt_data);

/**
@brief Configure an IF chain + modem (must configure before start)
@param if_chain number of the IF chain + modem to configure [0, LGW_IF_CHAIN_NB - 1]
@param conf structure containing the configuration parameters
@return LGW_HAL_ERROR id the operation failed, LGW_HAL_SUCCESS else
*/
int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s * conf);

// loragw_aux.h
/**
@brief Wait for a certain time (millisecond accuracy)
@param t number of milliseconds to wait.
*/
void wait_ms(unsigned long t);

#ifdef __cplusplus
}
#endif

#endif
