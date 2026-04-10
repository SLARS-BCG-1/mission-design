/*
 * SLARS-BCG-1 Flight Software
 * File: tc_dispatcher.c
 *
 * Receives telecommand frames from the UHF radio and routes
 * them to the correct task function.
 *
 * All 32 commands are defined in Step 6 Section 10a.
 * Each command ID is a single byte (0x01 to 0x15).
 *
 * Frame format (Step 6 Section 10):
 *   Byte 0    : Command ID (0x01 to 0x15)
 *   Bytes 1-N : Parameters (variable by command)
 *   Last 4B   : CRC32 authentication
 *
 * Author: Bandaranayake College, Gampaha
 */

#include "../hal/slars_types.h"

/* TC command IDs — from Step 6 Section 10a */
#define TC_SET_MODE               0x01
#define TC_UPLINK_TLE             0x02
#define TC_REBOOT_OBC             0x03
#define TC_RESET_SUBSYSTEM        0x04
#define TC_SET_MAX_PASSES         0x05
#define TC_FORCE_BEACON           0x06
#define TC_REQUEST_EXTENDED_TM    0x07
#define TC_ENABLE_PAYLOAD         0x08
#define TC_DISABLE_PAYLOAD        0x09
#define TC_SET_ADCS_MODE          0x0A
#define TC_CLEAR_APRS_BUFFER      0x0B
#define TC_START_SBAND_DL         0x0C
#define TC_FORMAT_FLASH           0x0D
#define TC_SET_DOPPLER_OFFSET     0x0E
#define TC_UPDATE_IGRF            0x0F
#define TC_SET_MTQ_DIPOLE         0x10
#define TC_SET_RW_SPEED           0x11
#define TC_READ_FDIR_LOG          0x12
#define TC_SET_BEACON_INTERVAL    0x13
#define TC_TRIGGER_DESATURATION   0x14
#define TC_SET_SL_BBOX            0x15

/* Safety confirm code for dangerous TC_FORMAT_FLASH command */
#define FORMAT_FLASH_CONFIRM      0xDEAD

/* TC execution statistics */
static uint32_t tc_count_total   = 0;
static uint32_t tc_count_invalid = 0;
static uint32_t last_tc_id       = 0;

/* ── Internal helpers ──────────────────────────────────────────────────── */

/* Safe byte reader — prevents buffer overread */
static uint8_t read_u8(const uint8_t *buf, uint8_t len,
                        uint8_t offset, uint8_t defval) {
    return (offset < len) ? buf[offset] : defval;
}

static uint16_t read_u16(const uint8_t *buf, uint8_t len,
                          uint8_t offset, uint16_t defval) {
    if (offset + 1 >= len) return defval;
    return (uint16_t)((buf[offset] << 8) | buf[offset+1]);
}

/* ── CRC32 verification ────────────────────────────────────────────────── */
/*
 * All TCs are authenticated with CRC32 (Step 6 Section 10).
 * In simulation this always returns 1 (valid).
 * On real hardware: compute CRC32 over bytes 0 to len-5,
 * compare with the 4 bytes at buf[len-4].
 */
static uint8_t verify_tc_crc(const uint8_t *buf, uint8_t len) {
#ifdef SIMULATION_MODE
    (void)buf; (void)len;
    return 1;  /* Always valid in simulation */
#else
    /* TODO: implement CRC32 verification using real hardware CRC unit */
    (void)buf; (void)len;
    return 1;
#endif
}

/* ── The main TC dispatcher ────────────────────────────────────────────── */
/*
 * Called by comms_manager_task() whenever a TC frame arrives.
 * buf  = raw bytes received over UHF
 * len  = total byte count including command ID and CRC
 */
void tc_dispatch(const uint8_t *buf, uint8_t len) {

    /* Minimum valid TC is 1 byte (command ID) */
    if (buf == NULL || len < 1) {
        tc_count_invalid++;
        printf("[TC]   INVALID: null or zero-length frame\n");
        return;
    }

    /* Verify CRC authentication */
    if (!verify_tc_crc(buf, len)) {
        tc_count_invalid++;
        printf("[TC]   REJECTED: CRC authentication failed\n");
        fdir_log("TC_CRC_FAIL: id=0x%02X", buf[0]);
        return;
    }

    uint8_t cmd_id = buf[0];
    tc_count_total++;
    last_tc_id = cmd_id;

    printf("[TC]   Executing 0x%02X (total=%lu)\n",
           cmd_id, (unsigned long)tc_count_total);

    switch (cmd_id) {

        /* ── 0x01  SET_MODE ──────────────────────────────────────────── */
        /*  param: 1 byte — 0=SAFE  1=NOMINAL  2=SCIENCE                */
        case TC_SET_MODE: {
            uint8_t new_mode = read_u8(buf, len, 1, MODE_NOMINAL);
            if (new_mode > 2) {
                printf("[TC]   SET_MODE: invalid mode %d\n", new_mode);
                break;
            }
            system_set_mode(new_mode);
            const char *names[] = {"SAFE", "NOMINAL", "SCIENCE"};
            printf("[TC]   SET_MODE -> %s\n", names[new_mode]);
            break;
        }

        /* ── 0x02  UPLINK_TLE ────────────────────────────────────────── */
        /*  param: 138 bytes — two-line TLE string                       */
        case TC_UPLINK_TLE: {
            if (len < 3) {
                printf("[TC]   UPLINK_TLE: too short\n");
                break;
            }
            /* TLE is 2 lines of 69 chars each */
            char l1[70] = {0};
            char l2[70] = {0};
            uint8_t copy_len = (len - 1) > 138 ? 138 : (len - 1);
            if (copy_len >= 69) {
                memcpy(l1, buf + 1, 69);
                l1[69] = '\0';
            }
            if (copy_len >= 138) {
                memcpy(l2, buf + 70, 69);
                l2[69] = '\0';
            }
            payload_scheduler_update_tle(l1, l2);
            printf("[TC]   UPLINK_TLE: new TLE loaded\n");
            break;
        }

        /* ── 0x03  REBOOT_OBC ────────────────────────────────────────── */
        /*  param: 1 byte — delay in seconds (0-255)                     */
        case TC_REBOOT_OBC: {
            uint8_t delay_s = read_u8(buf, len, 1, 5);
            printf("[TC]   REBOOT_OBC: rebooting in %d seconds\n", delay_s);
            fdir_log("TC_REBOOT: delay=%d", delay_s);
            /* On real hardware: schedule watchdog timeout after delay_s  */
            /* In simulation: print and continue                          */
            break;
        }

        /* ── 0x04  RESET_SUBSYSTEM ───────────────────────────────────── */
        /*  param: 1 byte — subsystem ID (0=EPS 1=ADCS 2=COMMS ...)     */
        case TC_RESET_SUBSYSTEM: {
            uint8_t sys_id = read_u8(buf, len, 1, 0);
            fdir_power_cycle_subsystem(sys_id);
            printf("[TC]   RESET_SUBSYSTEM: subsystem %d power cycled\n",
                   sys_id);
            break;
        }

        /* ── 0x05  SET_MAX_PASSES ────────────────────────────────────── */
        /*  param: 1 byte — max science passes per day (1-6)             */
        case TC_SET_MAX_PASSES: {
            uint8_t n = read_u8(buf, len, 1, 3);
            if (n < 1 || n > 6) {
                printf("[TC]   SET_MAX_PASSES: invalid value %d (need 1-6)\n",
                       n);
                break;
            }
            payload_scheduler_set_max_passes(n);
            printf("[TC]   SET_MAX_PASSES -> %d passes/day\n", n);
            break;
        }

        /* ── 0x06  FORCE_BEACON ──────────────────────────────────────── */
        /*  No parameters — immediately transmit TM housekeeping frame   */
        case TC_FORCE_BEACON: {
            printf("[TC]   FORCE_BEACON: transmitting now\n");
            /*
             * On real hardware: set a flag that comms_manager checks
             * In simulation: the comms_manager will handle it next cycle
             */
            break;
        }

        /* ── 0x07  REQUEST_EXTENDED_TM ───────────────────────────────── */
        /*  param: 1 byte — page number (0-7)                            */
        case TC_REQUEST_EXTENDED_TM: {
            uint8_t page = read_u8(buf, len, 1, 0);
            printf("[TC]   REQUEST_EXTENDED_TM: page %d\n", page);
            /*
             * Extended TM pages (Step 6 Section 10):
             *   0: Full EPS readings
             *   1: Full ADCS state (quaternion, all rates)
             *   2: FDIR event log (last 50 entries)
             *   3: Flash usage statistics
             *   4: APRS message queue summary
             *   5: Orbit propagator state
             *   6: COMMS statistics
             *   7: Thermal readings
             */
            break;
        }

        /* ── 0x08  ENABLE_PAYLOAD ────────────────────────────────────── */
        /*  param: 1 byte — 0=Imager  1=SDR relay  2=AIS receiver       */
        case TC_ENABLE_PAYLOAD: {
            uint8_t pid = read_u8(buf, len, 1, 0);
            switch (pid) {
                case 0: imager_enable();  printf("[TC]   ENABLE_PAYLOAD: imager ON\n");      break;
                case 1: sdr_enable_tx();  printf("[TC]   ENABLE_PAYLOAD: SDR relay ON\n");   break;
                case 2:                   printf("[TC]   ENABLE_PAYLOAD: AIS already ON\n"); break;
                default: printf("[TC]   ENABLE_PAYLOAD: unknown ID %d\n", pid);
            }
            break;
        }

        /* ── 0x09  DISABLE_PAYLOAD ───────────────────────────────────── */
        /*  param: 1 byte — 0=Imager  1=SDR relay  2=AIS receiver       */
        case TC_DISABLE_PAYLOAD: {
            uint8_t pid = read_u8(buf, len, 1, 0);
            switch (pid) {
                case 0: imager_disable();  printf("[TC]   DISABLE_PAYLOAD: imager OFF\n");  break;
                case 1: sdr_disable_tx();  printf("[TC]   DISABLE_PAYLOAD: SDR relay OFF\n");break;
                case 2:                    printf("[TC]   DISABLE_PAYLOAD: AIS disabled\n"); break;
                default: printf("[TC]   DISABLE_PAYLOAD: unknown ID %d\n", pid);
            }
            break;
        }

        /* ── 0x0A  SET_ADCS_MODE ─────────────────────────────────────── */
        /*  param: 1 byte — 1=B-dot  2=Coarse PD  3=Nadir PID           */
        case TC_SET_ADCS_MODE: {
            uint8_t m = read_u8(buf, len, 1, 2);
            if (m < 1 || m > 3) {
                printf("[TC]   SET_ADCS_MODE: invalid mode %d\n", m);
                break;
            }
            adcs_set_mode(m);
            const char *mn[] = {"", "B-dot", "Coarse PD", "Nadir PID"};
            printf("[TC]   SET_ADCS_MODE -> Mode %d (%s)\n", m, mn[m]);
            break;
        }

        /* ── 0x0B  CLEAR_APRS_BUFFER ─────────────────────────────────── */
        /*  No parameters — flush all buffered APRS emergency messages   */
        case TC_CLEAR_APRS_BUFFER: {
            sim_aprs_queue = 0;
            printf("[TC]   CLEAR_APRS_BUFFER: queue flushed\n");
            break;
        }

        /* ── 0x0C  START_SBAND_DL ────────────────────────────────────── */
        /*  param: 2 bytes — file ID (0-999)                             */
        case TC_START_SBAND_DL: {
            uint16_t file_id = read_u16(buf, len, 1, 0);
            comms_trigger_sband_downlink();
            printf("[TC]   START_SBAND_DL: file %d queued\n", file_id);
            break;
        }

        /* ── 0x0D  FORMAT_FLASH ──────────────────────────────────────── */
        /*  param: 2 bytes — must equal 0xDEAD to confirm                */
        /*  DANGEROUS: erases all science data on the satellite          */
        case TC_FORMAT_FLASH: {
            uint16_t confirm = read_u16(buf, len, 1, 0);
            if (confirm != FORMAT_FLASH_CONFIRM) {
                printf("[TC]   FORMAT_FLASH: REJECTED — wrong confirm code\n");
                printf("[TC]   (Send 0xDEAD in bytes 1-2 to confirm)\n");
                break;
            }
            printf("[TC]   FORMAT_FLASH: ERASING ALL FLASH DATA\n");
            fdir_log("TC_FORMAT_FLASH: confirmed by ground");
            /* On real hardware: call lfs_format() here */
            break;
        }

        /* ── 0x0E  SET_DOPPLER_OFFSET ────────────────────────────────── */
        /*  param: 4 bytes — signed offset in Hz (-100000 to +100000)   */
        case TC_SET_DOPPLER_OFFSET: {
            /* Read 4-byte signed big-endian integer */
            if (len < 5) { printf("[TC]   SET_DOPPLER_OFFSET: too short\n"); break; }
            int32_t offset_hz = ((int32_t)buf[1] << 24) |
                                ((int32_t)buf[2] << 16) |
                                ((int32_t)buf[3] << 8)  |
                                 (int32_t)buf[4];
            sband_set_frequency_offset(offset_hz);
            printf("[TC]   SET_DOPPLER_OFFSET: %d Hz\n", offset_hz);
            break;
        }

        /* ── 0x0F  UPDATE_IGRF ───────────────────────────────────────── */
        /*  param: up to 512 KB — new IGRF-13 coefficient table          */
        case TC_UPDATE_IGRF: {
            printf("[TC]   UPDATE_IGRF: loading %d bytes of coefficient data\n",
                   len - 1);
            /* On real hardware: write buf+1 to NOR flash IGRF sector     */
            break;
        }

        /* ── 0x10  SET_MTQ_DIPOLE ────────────────────────────────────── */
        /*  param: 1 byte axis (0=X 1=Y 2=Z) + 4 bytes float dipole     */
        case TC_SET_MTQ_DIPOLE: {
            uint8_t axis = read_u8(buf, len, 1, 0);
            printf("[TC]   SET_MTQ_DIPOLE: axis=%d\n", axis);
            /* On real hardware: extract float from buf[2..5], command MTQ */
            break;
        }

        /* ── 0x11  SET_RW_SPEED ──────────────────────────────────────── */
        /*  param: 2 bytes — signed RPM (-6000 to +6000)                 */
        case TC_SET_RW_SPEED: {
            int16_t rpm = (int16_t)read_u16(buf, len, 1, 0);
            if (rpm > 6000)  rpm =  6000;
            if (rpm < -6000) rpm = -6000;
            sim_rw_speed = rpm;
            printf("[TC]   SET_RW_SPEED: %d RPM\n", rpm);
            break;
        }

        /* ── 0x12  READ_FDIR_LOG ─────────────────────────────────────── */
        /*  param: 1 byte — number of entries to downlink (1-50)         */
        case TC_READ_FDIR_LOG: {
            uint8_t entries = read_u8(buf, len, 1, 10);
            if (entries > 50) entries = 50;
            printf("[TC]   READ_FDIR_LOG: downlinking last %d entries\n",
                   entries);
            /* On real hardware: read from NOR flash FDIR circular buffer */
            break;
        }

        /* ── 0x13  SET_BEACON_INTERVAL ───────────────────────────────── */
        /*  param: 1 byte — interval in seconds (10-120)                 */
        case TC_SET_BEACON_INTERVAL: {
            uint8_t secs = read_u8(buf, len, 1, 30);
            if (secs < 10)  secs = 10;
            if (secs > 120) secs = 120;
            printf("[TC]   SET_BEACON_INTERVAL: %d seconds\n", secs);
            /* On real hardware: update the comms_manager beacon timer    */
            break;
        }

        /* ── 0x14  TRIGGER_DESATURATION ─────────────────────────────── */
        /*  No parameters — immediately begin RW desaturation            */
        case TC_TRIGGER_DESATURATION: {
            printf("[TC]   TRIGGER_DESATURATION: starting now\n");
            adcs_begin_desaturation();
            break;
        }

        /* ── 0x15  SET_SL_BBOX ───────────────────────────────────────── */
        /*  param: 4 floats — lat_min, lat_max, lon_min, lon_max         */
        case TC_SET_SL_BBOX: {
            printf("[TC]   SET_SL_BBOX: updated Sri Lanka bounding box\n");
            /* On real hardware: extract 4 floats from buf[1..16]         */
            /* and update the bounding box in payload_scheduler           */
            break;
        }

        /* ── Unknown command ─────────────────────────────────────────── */
        default: {
            tc_count_invalid++;
            printf("[TC]   UNKNOWN command: 0x%02X — ignored\n", cmd_id);
            fdir_log("TC_UNKNOWN: id=0x%02X", cmd_id);
            break;
        }
    }
}

/* ── Public statistics — ground station can request these ─────────────── */
uint32_t tc_get_total_count(void)   { return tc_count_total;   }
uint32_t tc_get_invalid_count(void) { return tc_count_invalid; }
uint32_t tc_get_last_id(void)       { return last_tc_id;        }
