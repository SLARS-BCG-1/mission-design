/*
 * SLARS-BCG-1 Flight Software
 * File: tm_encoder.c
 *
 * Encodes the satellite housekeeping telemetry struct into
 * a properly formatted 128-byte CSP/AX.25 frame for UHF TX.
 *
 * Frame layout (Step 6 Section 10):
 *
 *  Bytes 0-3   : CSP header
 *                  Bits 31-26 : Source address   (SLARS = 5)
 *                  Bits 25-20 : Destination addr (Ground = 1)
 *                  Bits 19-16 : Destination port (TM port = 9)
 *                  Bits 15-12 : Source port      (OBC = 10)
 *                  Bits 11-8  : Reserved
 *                  Bits 7-0   : Flags (CRC on = 0x01)
 *
 *  Byte  4     : Frame type  (0x01 = housekeeping TM)
 *  Byte  5     : Sequence counter (wraps 0-255)
 *  Bytes 6-7   : Mission elapsed time (seconds since boot, low 16 bits)
 *  Byte  8     : System mode (0=SAFE 1=NOM 2=SCI)
 *  Bytes 9-10  : Battery SoC (uint16, 0-10000 = 0.00%-100.00%)
 *  Bytes 11-12 : Battery voltage (uint16, millivolts)
 *  Byte  13    : Battery temperature (int8, degrees C + 40 offset)
 *  Bytes 14-15 : Solar power (uint16, milliwatts)
 *  Bytes 16-17 : Total draw (uint16, milliwatts)
 *  Byte  18    : OBC temperature (int8, degrees C + 40 offset)
 *  Byte  19    : ADCS mode (1/2/3)
 *  Byte  20    : Pointing error (uint8, degrees x 2, so 0.5 deg resolution)
 *  Byte  21    : Angular rate (uint8, deg/s x 4, so 0.25 deg/s resolution)
 *  Bytes 22-23 : Reaction wheel speed (int16, RPM)
 *  Bytes 24-27 : Orbit count (uint32, big-endian)
 *  Byte  28    : Science passes today (uint8)
 *  Byte  29    : APRS queue length (uint8)
 *  Bytes 30-33 : Unix timestamp (uint32, big-endian)
 *  Byte  34    : FDIR level (0-4)
 *  Bytes 35-123: Reserved / future expansion (zero filled)
 *  Bytes 124-125: Extended flags (reserved, 0x0000)
 *  Bytes 126-127: CRC16-CCITT over bytes 0-125
 *
 * Total: 128 bytes exactly.
 *
 * SLARS Team — Bandaranayake College, Gampaha
 */

#include "../hal/slars_types.h"

/* ── Frame constants ─────────────────────────────────────────────────────── */
#define TM_FRAME_LEN        128
#define TM_TYPE_HOUSEKEEPING 0x01
#define TM_TYPE_EXTENDED     0x02
#define TM_TYPE_FDIR_LOG     0x03

/* CSP header fields */
#define CSP_SRC_ADDR_SLARS   5    /* SLARS satellite address on CSP network  */
#define CSP_DST_ADDR_GROUND  1    /* Ground station CSP address              */
#define CSP_DST_PORT_TM      9    /* Telemetry port                          */
#define CSP_SRC_PORT_OBC    10    /* OBC source port                         */
#define CSP_FLAG_CRC      0x01    /* CRC enabled flag                        */

/* ── Internal state ──────────────────────────────────────────────────────── */
static uint8_t  seq_counter    = 0;   /* Sequence counter wraps 0-255        */
static uint32_t boot_time      = 0;   /* Set on first call                   */
static uint32_t frames_encoded = 0;   /* Total frames produced               */
static uint8_t  boot_set       = 0;

/* ── CRC16-CCITT ─────────────────────────────────────────────────────────── */
/*
 * Computes CRC16-CCITT (polynomial 0x1021, init 0xFFFF).
 * This is the same CRC used by the AX.25 frame check sequence.
 * Covers bytes 0 to len-1.
 */
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ── CSP header builder ──────────────────────────────────────────────────── */
/*
 * Builds the 4-byte CSP header in big-endian format.
 * CSP v1 header layout (32 bits):
 *   [31:26] src_addr  (6 bits)
 *   [25:20] dst_addr  (6 bits)
 *   [19:16] dst_port  (4 bits)  -- corrected: was [19:14] dst_port (6 bits)
 *   [15:10] src_port  (6 bits)
 *   [9:8]   reserved  (2 bits)
 *   [7:0]   flags     (8 bits)
 */
static void build_csp_header(uint8_t *frame,
                              uint8_t src, uint8_t dst,
                              uint8_t dst_port, uint8_t src_port,
                              uint8_t flags) {
    uint32_t hdr = 0;
    hdr |= ((uint32_t)(src      & 0x3F)) << 26;
    hdr |= ((uint32_t)(dst      & 0x3F)) << 20;
    hdr |= ((uint32_t)(dst_port & 0x3F)) << 14;
    hdr |= ((uint32_t)(src_port & 0x3F)) << 8;
    hdr |= ((uint32_t)(flags    & 0xFF));
    /* Store big-endian */
    frame[0] = (uint8_t)((hdr >> 24) & 0xFF);
    frame[1] = (uint8_t)((hdr >> 16) & 0xFF);
    frame[2] = (uint8_t)((hdr >>  8) & 0xFF);
    frame[3] = (uint8_t)( hdr        & 0xFF);
}

/* ── Encode housekeeping TM frame ────────────────────────────────────────── */
/*
 * Fills buf with a complete 128-byte housekeeping TM frame.
 * buf must point to at least 128 bytes of writable memory.
 * Returns the number of bytes written (always 128).
 */
uint8_t tm_encode_housekeeping(uint8_t *buf) {

    if (buf == NULL) return 0;

    /* Record boot time on first call */
    if (!boot_set) {
        boot_time = rtc_get_unix_time();
        boot_set  = 1;
    }

    /* Zero the entire frame first — all reserved bytes become 0x00 */
    memset(buf, 0, TM_FRAME_LEN);

    /* ── Bytes 0-3: CSP header ────────────────────────────────────────── */
    build_csp_header(buf,
                     CSP_SRC_ADDR_SLARS,
                     CSP_DST_ADDR_GROUND,
                     CSP_DST_PORT_TM,
                     CSP_SRC_PORT_OBC,
                     CSP_FLAG_CRC);

    /* ── Byte 4: Frame type ───────────────────────────────────────────── */
    buf[4] = TM_TYPE_HOUSEKEEPING;

    /* ── Byte 5: Sequence counter (0-255 wrapping) ───────────────────── */
    buf[5] = seq_counter++;

    /* ── Bytes 6-7: Mission elapsed time (seconds since boot) ────────── */
    uint32_t met = rtc_get_unix_time() - boot_time;
    buf[6] = (uint8_t)((met >> 8) & 0xFF);
    buf[7] = (uint8_t)( met       & 0xFF);

    /* ── Byte 8: System mode ─────────────────────────────────────────── */
    buf[8] = power_get_mode();   /* 0=SAFE  1=NOMINAL  2=SCIENCE */

    /* ── Bytes 9-10: Battery SoC (0-10000 = 0.00% to 100.00%) ───────── */
    /* Multiply by 10000 for two decimal places of precision             */
    uint16_t soc_scaled = (uint16_t)(eps_read_soc() * 10000.0f);
    buf[9]  = (uint8_t)((soc_scaled >> 8) & 0xFF);
    buf[10] = (uint8_t)( soc_scaled       & 0xFF);

    /* ── Bytes 11-12: Battery voltage (millivolts) ───────────────────── */
    uint16_t vbat_mv = (uint16_t)(eps_read_voltage() * 1000.0f);
    buf[11] = (uint8_t)((vbat_mv >> 8) & 0xFF);
    buf[12] = (uint8_t)( vbat_mv       & 0xFF);

    /* ── Byte 13: Battery temperature (degrees C + 40 offset) ────────── */
    /* Offset +40 so -40C = 0, +85C = 125 — fits in unsigned byte        */
    int8_t bat_temp_raw = (int8_t)(eps_read_battery_temp());
    buf[13] = (uint8_t)(bat_temp_raw + 40);

    /* ── Bytes 14-15: Solar power (milliwatts) ───────────────────────── */
    uint16_t solar_mw = (uint16_t)(eps_read_solar_power() * 1000.0f);
    buf[14] = (uint8_t)((solar_mw >> 8) & 0xFF);
    buf[15] = (uint8_t)( solar_mw       & 0xFF);

    /* ── Bytes 16-17: Total draw (milliwatts) ────────────────────────── */
    uint16_t draw_mw = (uint16_t)(eps_read_total_watts() * 1000.0f);
    buf[16] = (uint8_t)((draw_mw >> 8) & 0xFF);
    buf[17] = (uint8_t)( draw_mw       & 0xFF);

    /* ── Byte 18: OBC temperature (degrees C + 40 offset) ───────────── */
    int8_t obc_temp_raw = (int8_t)(obc_read_temperature());
    buf[18] = (uint8_t)(obc_temp_raw + 40);

    /* ── Byte 19: ADCS mode ──────────────────────────────────────────── */
    buf[19] = adcs_get_mode();   /* 1=B-dot  2=Coarse PD  3=Nadir PID   */

    /* ── Byte 20: Pointing error (degrees x 2, 0.5 deg resolution) ───── */
    /* Max representable: 127.5 degrees. Clamp at 127.                   */
    float att_err = adcs_read_pointing_error();
    if (att_err > 127.0f) att_err = 127.0f;
    buf[20] = (uint8_t)(att_err * 2.0f);

    /* ── Byte 21: Angular rate (deg/s x 4, 0.25 deg/s resolution) ────── */
    /* Max representable: 63.75 deg/s. Clamp at 63.                      */
    float rate = adcs_read_angular_rate();
    if (rate > 63.0f) rate = 63.0f;
    buf[21] = (uint8_t)(rate * 4.0f);

    /* ── Bytes 22-23: Reaction wheel speed (int16 RPM, big-endian) ───── */
    int16_t rw = (int16_t)adcs_read_rw_speed();
    buf[22] = (uint8_t)((rw >> 8) & 0xFF);
    buf[23] = (uint8_t)( rw       & 0xFF);

    /* ── Bytes 24-27: Orbit count (uint32, big-endian) ───────────────── */
    uint32_t orbits = orbit_get_count();
    buf[24] = (uint8_t)((orbits >> 24) & 0xFF);
    buf[25] = (uint8_t)((orbits >> 16) & 0xFF);
    buf[26] = (uint8_t)((orbits >>  8) & 0xFF);
    buf[27] = (uint8_t)( orbits        & 0xFF);

    /* ── Byte 28: Science passes today ──────────────────────────────── */
    buf[28] = power_get_max_passes();

    /* ── Byte 29: APRS queue length ──────────────────────────────────── */
    buf[29] = aprs_get_queue_length();

    /* ── Bytes 30-33: Unix timestamp (uint32, big-endian) ────────────── */
    uint32_t ts = rtc_get_unix_time();
    buf[30] = (uint8_t)((ts >> 24) & 0xFF);
    buf[31] = (uint8_t)((ts >> 16) & 0xFF);
    buf[32] = (uint8_t)((ts >>  8) & 0xFF);
    buf[33] = (uint8_t)( ts        & 0xFF);

    /* ── Byte 34: FDIR level ─────────────────────────────────────────── */
    /* 0 = nominal, 1-4 = escalating fault levels                        */
    buf[34] = 0;   /* FDIR level stored here — update when FDIR exposes  */
                   /* a getter function in a later step                   */

    /* Bytes 35-123: Zero filled (reserved) — already done by memset     */
    /* Bytes 124-125: Extended flags (reserved) — 0x0000                 */
    buf[124] = 0x00;
    buf[125] = 0x00;

    /* ── Bytes 126-127: CRC16-CCITT over bytes 0-125 ─────────────────── */
    uint16_t crc = crc16_ccitt(buf, 126);
    buf[126] = (uint8_t)((crc >> 8) & 0xFF);
    buf[127] = (uint8_t)( crc       & 0xFF);

    frames_encoded++;

    return TM_FRAME_LEN;
}

/* ── Decoder — parse a received 128-byte frame back into fields ──────────── */
/*
 * Used by the ground station test and by the unit tests to verify
 * that encode then decode gives back the same values.
 * Returns 1 if CRC is valid, 0 if frame is corrupt.
 */
uint8_t tm_decode_housekeeping(const uint8_t *buf, tm_decoded_t *out) {

    if (buf == NULL || out == NULL) return 0;

    /* Verify CRC first */
    uint16_t expected = crc16_ccitt(buf, 126);
    uint16_t received = ((uint16_t)buf[126] << 8) | buf[127];
    if (expected != received) {
        printf("[TM]   CRC MISMATCH: expected 0x%04X got 0x%04X\n",
               expected, received);
        return 0;
    }

    /* Parse each field */
    out->frame_type    = buf[4];
    out->seq_counter   = buf[5];
    out->met_seconds   = ((uint16_t)buf[6] << 8) | buf[7];
    out->system_mode   = buf[8];
    out->bat_soc_pct   = (((uint16_t)buf[9]  << 8) | buf[10]) / 100.0f;
    out->bat_voltage_v = (((uint16_t)buf[11] << 8) | buf[12]) / 1000.0f;
    out->bat_temp_c    = (int8_t)buf[13] - 40;
    out->solar_w       = (((uint16_t)buf[14] << 8) | buf[15]) / 1000.0f;
    out->draw_w        = (((uint16_t)buf[16] << 8) | buf[17]) / 1000.0f;
    out->obc_temp_c    = (int8_t)buf[18] - 40;
    out->adcs_mode     = buf[19];
    out->pointing_deg  = buf[20] / 2.0f;
    out->rate_degs     = buf[21] / 4.0f;
    out->rw_rpm        = (int16_t)(((uint16_t)buf[22] << 8) | buf[23]);
    out->orbit_count   = ((uint32_t)buf[24] << 24) |
                         ((uint32_t)buf[25] << 16) |
                         ((uint32_t)buf[26] <<  8) |
                          (uint32_t)buf[27];
    out->passes_today  = buf[28];
    out->aprs_queue    = buf[29];
    out->unix_time     = ((uint32_t)buf[30] << 24) |
                         ((uint32_t)buf[31] << 16) |
                         ((uint32_t)buf[32] <<  8) |
                          (uint32_t)buf[33];
    out->fdir_level    = buf[34];
    out->crc_valid     = 1;

    return 1;
}

/* ── Public statistics ───────────────────────────────────────────────────── */
uint32_t tm_get_frames_encoded(void) { return frames_encoded; }
uint8_t  tm_get_seq_counter(void)    { return seq_counter;    }
