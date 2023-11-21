#include <cstring>
#include <cmath>

#include "schedule-concentrator.h"

ScheduleConcentrator::ScheduleConcentrator()
    : item{}
{
}

ScheduleConcentrator::ScheduleConcentrator(
    const struct lgw_pkt_tx_s& value
)
{
    memmove(&item, &value, sizeof(struct lgw_pkt_tx_s));
}

void ScheduleConcentrator::getItem(
        void *value
)
{
    if (value)
        memmove(value, &item, sizeof(struct lgw_pkt_tx_s));
}

void ScheduleConcentrator::setItem(
    const void* value
)
{
    memmove(&item, &value, sizeof(struct lgw_pkt_tx_s));
}

uint32_t ScheduleConcentrator::getCountUs() const
{
    return item.count_us;
}

void ScheduleConcentrator::setCountUs(
    uint32_t value
)
{
    item.count_us = value;
}

static uint32_t lora_packet_time_on_air(const uint8_t bw, const uint8_t sf, const uint8_t cr, const uint16_t n_symbol_preamble,
    const bool no_header, const bool no_crc, const uint8_t size,
    double * out_nb_symbols, uint32_t * out_nb_symbols_payload, uint16_t * out_t_symbol_us
)
{
    uint8_t H, DE, n_bit_crc;
    uint8_t bw_pow;
    uint16_t t_symbol_us;
    double n_symbol;
    uint32_t toa_us, n_symbol_payload;

    // Check input parameters
    if (!IS_LORA_DR(sf))
        return 0;
    if (!IS_LORA_BW(bw))
        return 0;
    if (!IS_LORA_CR(cr))
        return 0;

    // Get bandwidth 125KHz divider
    switch (bw) {
        case BW_125KHZ:
            bw_pow = 1;
            break;
        case BW_250KHZ:
            bw_pow = 2;
            break;
        case BW_500KHZ:
            bw_pow = 4;
            break;
        default:
            return 0;
    }

    //  Duration of 1 symbol
    t_symbol_us = (1 << sf) * 8 / bw_pow; /* 2^SF / BW , in microseconds */

    // Packet parameters
    H = (!no_header) ? 1 : 0; // header is always enabled, except for beacons */
    DE = (sf >= 11) ? 1 : 0; // Low data rate optimization enabled for SF11 and SF12
    n_bit_crc = (!no_crc) ? 16 : 0;

    // Number of symbols in the payload
    n_symbol_payload = ceil(fmax((double)( 8 * size + n_bit_crc - 4*sf + ((sf >= 7) ? 8 : 0) + 20*H ), 0.0) /
        (double)( 4 * (sf - 2*DE)) ) * ( cr + 4 ); // Explicitly cast to double to keep precision of the division

    // number of symbols in packet
    n_symbol = (double)n_symbol_preamble + ((sf >= 7) ? 4.25 : 6.25) + 8.0 + (double)n_symbol_payload;

    // Duration of packet in microseconds
    toa_us = (uint32_t)( (double)n_symbol * (double)t_symbol_us );

    // Return details if required
    if (out_nb_symbols)
        *out_nb_symbols = n_symbol;
    if (out_nb_symbols_payload)
        *out_nb_symbols_payload = n_symbol_payload;
    if (out_t_symbol_us)
        *out_t_symbol_us = t_symbol_us;
    return toa_us;
}

#define CONTEXT_FSK_sync_word_size  3
static uint32_t lgwTimeOnAir(
    const struct lgw_pkt_tx_s *packet
) {
    double t_fsk;
    uint32_t toa_ms, toa_us;

    if (!packet)
        return 0;

    if (packet->modulation == MOD_LORA) {
        toa_us = lora_packet_time_on_air(packet->bandwidth, packet->datarate, packet->coderate, packet->preamble, packet->no_header, packet->no_crc, packet->size, NULL, NULL, NULL);
        toa_ms = (uint32_t)( (double)toa_us / 1000.0 + 0.5 );
    } else if (packet->modulation == MOD_FSK) {
        t_fsk = (8 * (double)(packet->preamble + CONTEXT_FSK_sync_word_size + 1 + packet->size + ((packet->no_crc == true) ? 0 : 2)) / (double)packet->datarate) * 1E3;
        toa_ms = (uint32_t)t_fsk + 1; /* add margin for rounding */
    } else
        toa_ms = 0;
    return toa_ms;
}

uint32_t ScheduleConcentrator::getTimeOnAir() const
{
    return lgwTimeOnAir(&item);
}

void ScheduleConcentrator::setTxMode(
    uint8_t value
)
{
    item.tx_mode = value;
}
