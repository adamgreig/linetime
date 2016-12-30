/*
 * uBlox GPS receiver
 * 2014, 2016 Adam Greig
 */

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "ublox.h"
#include "ch.h"
#include "hal.h"

/* UBX sync bytes */
#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* UBX Classes */
#define UBX_NAV 0x01
#define UBX_RXM 0x02
#define UBX_INF 0x04
#define UBX_ACK 0x05
#define UBX_CFG 0x06
#define UBX_UPD 0x09
#define UBX_MON 0x0A
#define UBX_AID 0x0B
#define UBX_TIM 0x0D
#define UBX_MGA 0x13
#define UBX_LOG 0x21
#define NMEA_CLASS 0xF0

/* Selection of UBX IDs */
#define UBX_ACK_NAK     0x00
#define UBX_ACK_ACK     0x01
#define UBX_CFG_PRT     0x00
#define UBX_CFG_MSG     0x01
#define UBX_CFG_TP5     0x31
#define UBX_CFG_NAV5    0x24
#define UBX_CFG_RATE    0x08
#define UBX_CFG_GNSS    0x3E
#define UBX_CFG_SBAS    0x16
#define UBX_NAV_PVT     0x07
#define UBX_NAV_TIMELS  0x26
#define UBX_TIM_TP      0x01

static SerialDriver* ublox_seriald;
struct ublox_tp_time_t ublox_upcoming_tp_time;

/* UBX Decoding State Machine States */
typedef enum {
    STATE_IDLE = 0, STATE_SYNC1, STATE_SYNC2,
    STATE_CLASS, STATE_ID, STATE_L1, STATE_L2,
    STATE_PAYLOAD, STATE_CK_A, NUM_STATES
} ubx_state;

/* Structs for various UBX messages */

/* UBX-CFG-NAV5
 * Set navigation fix settings.
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[36];
        struct {
            uint16_t mask;
            uint8_t dyn_model;
            uint8_t fix_mode;
            int32_t fixed_alt;
            uint32_t fixed_alt_var;
            int8_t min_elev;
            uint8_t dr_limit;
            uint16_t p_dop, t_dop, p_acc, t_acc;
            uint8_t static_hold_thres;
            uint8_t dgps_timeout;
            uint8_t cno_thresh_num_svs, cno_thresh;
            uint16_t reserved;
            uint16_t static_hold_max_dist;
            uint8_t utc_standard;
            uint8_t reserved3;
            uint32_t reserved4;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_cfg_nav5_t;
#define UBX_CFG_NAV5_DYN_MODEL_PORTABLE     (0)
#define UBX_CFG_NAV5_DYN_MODEL_STATIONARY   (2)
#define UBX_CFG_NAV5_DYN_MODEL_PEDESTRIAN   (3)
#define UBX_CFG_NAV5_DYN_MODEL_AUTOMOTIVE   (4)
#define UBX_CFG_NAV5_DYN_MODEL_SEA          (5)
#define UBX_CFG_NAV5_DYN_MODEL_AIRBORNE_1G  (6)
#define UBX_CFG_NAV5_DYN_MODEL_AIRBORNE_2G  (7)
#define UBX_CFG_NAV5_DYN_MODEL_AIRBORNE_4G  (8)
#define UBX_CFG_NAV5_UTC_STANDARD_AUTO      (0)
#define UBX_CFG_NAV5_UTC_STANDARD_USNO      (3)
#define UBX_CFG_NAV5_UTC_STANDARD_USSR      (6)
#define UBX_CFG_NAV5_UTC_STANDARD_NTSC      (7)

/* UBX-CFG-GNSS
 * GNSS system configuration
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[44];
        struct {
            uint8_t msg_ver;
            uint8_t num_trk_ch_hw;
            uint8_t num_trk_ch_use;
            uint8_t num_config_blocks;
            uint8_t gps_gnss_id;
            uint8_t gps_res_trk_ch;
            uint8_t gps_max_trk_ch;
            uint8_t gps_reserved1;
            uint32_t gps_flags;
            uint8_t sbas_gnss_id;
            uint8_t sbas_res_trk_ch;
            uint8_t sbas_max_trk_ch;
            uint8_t sbas_reserved1;
            uint32_t sbas_flags;
            uint8_t beidou_gnss_id;
            uint8_t beidou_res_trk_ch;
            uint8_t beidou_max_trk_ch;
            uint8_t beidou_reserved1;
            uint32_t beidou_flags;
            uint8_t qzss_gnss_id;
            uint8_t qzss_res_trk_ch;
            uint8_t qzss_max_trk_ch;
            uint8_t qzss_reserved1;
            uint32_t qzss_flags;
            uint8_t glonass_gnss_id;
            uint8_t glonass_res_trk_ch;
            uint8_t glonass_max_trk_ch;
            uint8_t glonass_reserved1;
            uint32_t glonass_flags;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_cfg_gnss_t;
#define UBX_CFG_GNSS_FLAGS_ENABLE(x) (x&1)
#define UBX_CFG_GNSS_GNSS_ID_GPS     (0)
#define UBX_CFG_GNSS_GNSS_ID_SBAS    (1)
#define UBX_CFG_GNSS_GNSS_ID_BEIDOU  (3)
#define UBX_CFG_GNSS_GNSS_ID_QZSS    (5)
#define UBX_CFG_GNSS_GNSS_ID_GLONASS (6)

/* UBX-CFG-TP5
 * Timepulse settings.
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[32];
        struct {
            uint8_t tp_idx;
            uint8_t version;
            uint8_t reserved1[2];
            int16_t ant_cable_delay;
            int16_t rf_group_delay;
            uint32_t freq_period;
            uint32_t freq_period_lock;
            uint32_t pulse_len_ratio;
            uint32_t pulse_len_ratio_lock;
            int32_t user_config_delay;
            uint32_t flags;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_cfg_tp5_t;
#define UBX_CFG_TP5_FLAGS_ACTIVE                (1<<0)
#define UBX_CFG_TP5_FLAGS_LOCK_GNSS_FREQ        (1<<1)
#define UBX_CFG_TP5_FLAGS_LOCKED_OTHER_SET      (1<<2)
#define UBX_CFG_TP5_FLAGS_IS_FREQ               (1<<3)
#define UBX_CFG_TP5_FLAGS_IS_LENGTH             (1<<4)
#define UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW          (1<<5)
#define UBX_CFG_TP5_FLAGS_POLARITY              (1<<6)
#define UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_UTC     (0<<7)
#define UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_GPS     (1<<7)
#define UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_GLONASS (2<<7)
#define UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_BEIDOU  (3<<7)
#define UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_GALILEO (4<<7)

/* UBX-CFG-SBAS
 * SBAS configuration
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[8];
        struct {
            uint8_t mode;
            uint8_t usage;
            uint8_t max_sbas;
            uint8_t scanmode2;
            uint32_t scanmode1;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_cfg_sbas_t;
#define UBX_CFG_SBAS_MODE_ENABLED(x)    ((x<<0)&1)
#define UBX_CFG_SBAS_MODE_TEST(x)       ((x<<1)&2)
#define UBX_CFG_SBAS_USAGE_RANGE(x)     ((x<<0)&1)
#define UBX_CFG_SBAS_USAGE_DIFF_CORR(x) ((x<<1)&2)
#define UBX_CFG_SBAS_USAGE_INTEGRITY(x) ((x<<2)&4)

/* UBX-CFG-MSG
 * Change rate (or disable) automatic delivery of messages
 * to the current port.
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[3];
        struct {
            uint8_t msg_class;
            uint8_t msg_id;
            uint8_t rate;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_cfg_msg_t;


/* UBX-CFG-PRT
 * Change port settings including protocols.
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[20];
        struct {
            uint8_t port_id;
            uint8_t reserved0;
            uint16_t tx_ready;
            uint32_t mode;
            uint32_t baud_rate;
            uint16_t in_proto_mask;
            uint16_t out_proto_mask;
            uint16_t flags;
            uint16_t reserved5;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_cfg_prt_t;

/* UBX-CFG-RATE
 * Change solution rate
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[6];
        struct {
            uint16_t meas_rate;
            uint16_t nav_rate;
            uint16_t time_ref;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_cfg_rate_t;

/* UBX-ACK
 * ACK/NAK messages after trying to set a config.
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[2];
        struct {
            uint8_t cls_id;
            uint8_t msg_id;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_ack_t;

/* UBX-NAV-PVT
 * Contains fix quality, position and time information.
 * Everything you want in one message.
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[92];
        struct {
            uint32_t i_tow;
            uint16_t year;
            uint8_t month, day, hour, minute, second;
            uint8_t valid;
            uint32_t t_acc;
            int32_t nano;
            uint8_t fix_type;
            uint8_t flags;
            uint8_t reserved1;
            uint8_t num_sv;
            int32_t lon, lat;
            int32_t height, h_msl;
            uint32_t h_acc, v_acc;
            int32_t velN, velE, velD, gspeed;
            int32_t head_mot;
            uint32_t s_acc;
            uint32_t head_acc;
            uint16_t p_dop;
            uint16_t reserved2;
            uint32_t reserved3;
            int32_t head_veh;
            uint32_t reserved4;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_nav_pvt_t;

/* UBX-NAV-TIMELS
 * Contains information about any upcoming leap seconds.
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[24];
        struct {
            uint32_t i_tow;
            uint8_t version;
            uint8_t reserved1[3];
            uint8_t src_of_curr_ls;
            int8_t curr_ls;
            uint8_t src_of_ls_change;
            int8_t ls_change;
            int32_t time_to_ls_event;
            uint16_t date_of_ls_gps_wn;
            uint16_t date_of_ls_gps_dn;
            uint8_t reserved2[3];
            uint8_t valid;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_nav_timels_t;
#define UBX_NAV_TIMELS_VALID_VALID_CURR_LS          (1<<0)
#define UBX_NAV_TIMELS_VALID_VALID_TIME_TO_LS_EVENT (1<<1)
#define UBX_NAV_TIMELS_CURR_SRC_DEFAULT             (0)
#define UBX_NAV_TIMELS_CURR_SRC_GPS_VS_GLONASS      (1)
#define UBX_NAV_TIMELS_CURR_SRC_GPS                 (2)
#define UBX_NAV_TIMELS_CURR_SRC_SBAS                (3)
#define UBX_NAV_TIMELS_CURR_SRC_BEIDOU              (4)
#define UBX_NAV_TIMELS_CURR_SRC_GALILEO             (5)
#define UBX_NAV_TIMELS_CURR_SRC_AIDED_DATA          (6)
#define UBX_NAV_TIMELS_CURR_SRC_CONFIGURATION       (7)
#define UBX_NAV_TIMELS_CURR_SRC_UNKNOWN             (255)
#define UBX_NAV_TIMELS_CHANGE_SRC_NO_SOURCE         (0)
#define UBX_NAV_TIMELS_CHANGE_SRC_GPS               (2)
#define UBX_NAV_TIMELS_CHANGE_SRC_SBAS              (3)
#define UBX_NAV_TIMELS_CHANGE_SRC_BEIDOU            (4)
#define UBX_NAV_TIMELS_CHANGE_SRC_GALILEO           (5)
#define UBX_NAV_TIMELS_CHANGE_SRC_GLONASS           (6)


/* UBX-TIM-TP
 * Timepulse timedata
 */
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2, class, id;
    uint16_t length;
    union {
        uint8_t payload[16];
        struct {
            uint32_t tow_ms;
            uint32_t tow_sub_ms;
            int32_t q_err;
            uint16_t week;
            uint8_t flags;
            uint8_t ref_info;
        } __attribute__((packed));
    };
    uint8_t ck_a, ck_b;
} ubx_tim_tp_t;
#define UBX_TIM_TP_FLAGS_TIMEBASE_UTC               (1<<0)
#define UBX_TIM_TP_FLAGS_UTC_AVAILABLE              (1<<1)
#define UBX_TIM_TP_REF_INFO_TIME_REF_GNSS_GPS       (0<<0)
#define UBX_TIM_TP_REF_INFO_TIME_REF_GNSS_GLONASS   (1<<0)
#define UBX_TIM_TP_REF_INFO_TIME_REF_GNSS_BAIDOU    (2<<0)
#define UBX_TIM_TP_REF_INFO_TIME_REF_GNSS_UNKNOWN   (15<<0)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_NA         (0<<4)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_CRL        (1<<4)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_NIST       (2<<4)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_USNO       (3<<4)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_BIPM       (4<<4)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_EURO       (5<<4)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_SU         (6<<4)
#define UBX_TIM_TP_REF_INFO_UTC_STANDARD_UNKNOWN    (15<<4)

enum ublox_result {
    UBLOX_WAIT,
    UBLOX_RXLEN_TOO_LONG,
    UBLOX_BAD_CHECKSUM,
    UBLOX_ACK, UBLOX_NAK,
    UBLOX_NAV_PVT, UBLOX_NAV_TIMELS,
    UBLOX_TIM_TP,
    UBLOX_CFG_NAV5,
    UBLOX_UNHANDLED,
    UBLOX_ERROR
};

static uint16_t ublox_fletcher_8(uint16_t chk, uint8_t *buf, uint8_t n);
static void ublox_checksum(uint8_t *buf);
static bool ublox_transmit(uint8_t *buf);
static enum ublox_result ublox_state_machine(void);
static bool ublox_expect(enum ublox_result expected, const char* errmsg);
static bool ublox_tx_expect(uint8_t* buf, enum ublox_result exp, const char* err);
static bool ublox_configure(void);
static bool ublox_configure_prt(void);
static bool ublox_configure_nav5(void);
static bool ublox_configure_rate(void);
static bool ublox_configure_gnss(void);
static bool ublox_configure_sbas(void);
static bool ublox_configure_tp5(void);
static bool ublox_configure_msg(void);
static void ublox_error(const char* err);

static SerialConfig serial_cfg = {
    .speed = 9600,
    .cr1 = 0,
    .cr2 = 0,
    .cr3 = 0,
};

static void ublox_error(const char* err)
{
    /* TODO: fill in error handler */
    chSysHalt(err);
}

/* Run the Fletcher-8 checksum, initialised to chk, over n bytes of buf */
static uint16_t ublox_fletcher_8(uint16_t chk, uint8_t *buf, uint8_t n)
{
    int i;
    uint8_t ck_a = chk & 0xff, ck_b = chk>>8;

    /* Run Fletcher-8 algorithm */
    for(i=0; i<n; i++) {
        ck_a += buf[i];
        ck_b += ck_a;
    }

    return (ck_b<<8) | (ck_a);
}

/* Computes the Fletcher-8 checksum over buf, using its length fields
 * to determine how much to read, returning the new checksum.
 */
static void ublox_checksum(uint8_t *buf)
{
    uint16_t plen;

    /* Check SYNC bytes are correct */
    if(buf[0] != UBX_SYNC1 && buf[1] != UBX_SYNC2)
        return;

    /* Extract payload length */
    plen = ((uint16_t*)buf)[2];

    uint16_t ck = ublox_fletcher_8(0, &buf[2], plen+4);

    /* Write new checksum to the buffer */
    buf[plen+6] = ck;
    buf[plen+7] = ck >> 8;
}

/* Transmit a UBX message over the Serial.
 * Message length is determined from the UBX length field.
 * Checksum is added automatically.
 */
static bool ublox_transmit(uint8_t *buf)
{
    size_t n, nwritten;
    systime_t timeout;

    /* Add checksum to outgoing message */
    ublox_checksum(buf);

    /* Determine length and thus suitable timeout in systicks (ms) */
    n = 8 + ((uint16_t*)buf)[2];
    timeout = MS2ST(n*2);

    /* Transmit message */
    nwritten = sdWriteTimeout(ublox_seriald, buf, n, timeout);
    if(nwritten != n) {
        ublox_error("didn't tx full buffer");
    }

    return nwritten == n;
}

/* Run new byte b through the UBX decoding state machine. Note that this
 * function preserves static state and processes new messages as appropriate
 * once received.
 */
uint8_t rxbuf[256] = {0};
uint8_t rxbufidx = 0;
static enum ublox_result ublox_state_machine()
{
    uint8_t b = sdGet(ublox_seriald);
    rxbuf[rxbufidx++] = b;
    static ubx_state state = STATE_IDLE;

    static uint8_t class, id;
    static uint16_t length;
    static uint16_t length_remaining;
    static uint8_t payload[128];
    static uint8_t ck_a, ck_b;
    static uint16_t ck;

    ubx_cfg_nav5_t cfg_nav5;
    ubx_nav_pvt_t nav_pvt;
    ubx_nav_timels_t nav_timels;
    ubx_tim_tp_t tim_tp;

    switch(state) {
        case STATE_IDLE:
            if(b == UBX_SYNC1)
                state = STATE_SYNC1;
            break;

        case STATE_SYNC1:
            if(b == UBX_SYNC2)
                state = STATE_SYNC2;
            else
                state = STATE_IDLE;
            break;

        case STATE_SYNC2:
            class = b;
            state = STATE_CLASS;
            break;

        case STATE_CLASS:
            id = b;
            state = STATE_ID;
            break;

        case STATE_ID:
            length = (uint16_t)b;
            state = STATE_L1;
            break;

        case STATE_L1:
            length |= (uint16_t)b << 8;
            if(length >= 128) {
                state = STATE_IDLE;
                rxbufidx = 0;
                ublox_error("rx length field >= 128");
                return UBLOX_RXLEN_TOO_LONG;
            }
            length_remaining = length;
            state = STATE_PAYLOAD;
            break;

        case STATE_PAYLOAD:
            if(length_remaining) {
                payload[length - length_remaining--] = b;
            } else {
                ck_a = b;
                state = STATE_CK_A;
            }
            break;

        case STATE_CK_A:
            ck_b = b;
            state = STATE_IDLE;

            /* verify checksum */
            ck = ublox_fletcher_8(0, &class, 1);
            ck = ublox_fletcher_8(ck, &id, 1);
            ck = ublox_fletcher_8(ck, (uint8_t*)&length, 2);
            ck = ublox_fletcher_8(ck, payload, length);
            if(ck_a != (ck&0xFF) || ck_b != (ck>>8)) {
                ublox_error("rx checksum invalid");
                state = STATE_IDLE;
                rxbufidx = 0;
                return UBLOX_BAD_CHECKSUM;
            }

            state = STATE_IDLE;
            rxbufidx = 0;

            switch(class) {
                case UBX_ACK:
                    if(id == UBX_ACK_ACK) {
                        return UBLOX_ACK;
                    } else if(id == UBX_ACK_NAK) {
                        return UBLOX_NAK;
                    } else {
                        ublox_error("unknown ack msg");
                        return UBLOX_UNHANDLED;
                    }
                    break;
                case UBX_NAV:
                    if(id == UBX_NAV_PVT) {
                        memcpy(nav_pvt.payload, payload, length);
                        /* TODO: handle receiving a PVT */
                        return UBLOX_NAV_PVT;
                    } else if(id == UBX_NAV_TIMELS) {
                        memcpy(nav_timels.payload, payload, length);
                        /* TODO: handle receiving a TIMELS */
                        return UBLOX_NAV_TIMELS;
                    } else {
                        ublox_error("unknown nav msg");
                        return UBLOX_UNHANDLED;
                    }
                    break;
                case UBX_TIM:
                    if(id == UBX_TIM_TP) {
                        memcpy(tim_tp.payload, payload, length);
                        if(tim_tp.flags & UBX_TIM_TP_FLAGS_TIMEBASE_UTC &&
                           tim_tp.flags & UBX_TIM_TP_FLAGS_UTC_AVAILABLE &&
                           tim_tp.ref_info & UBX_TIM_TP_REF_INFO_UTC_STANDARD_USNO)
                        {
                            ublox_upcoming_tp_time.week = tim_tp.week;
                            ublox_upcoming_tp_time.tow_sub_ms =
                                ((uint64_t)tim_tp.tow_ms << 32)
                                | tim_tp.tow_sub_ms;
                            ublox_upcoming_tp_time.valid = true;
                        } else {
                            ublox_upcoming_tp_time.valid = false;
                        }
                        return UBLOX_TIM_TP;
                    } else {
                        ublox_error("unknown tim msg");
                        return UBLOX_UNHANDLED;
                    }
                    break;
                case UBX_CFG:
                    if(id == UBX_CFG_NAV5) {
                        memcpy(cfg_nav5.payload, payload, length);
                        if(cfg_nav5.dyn_model !=
                           UBX_CFG_NAV5_DYN_MODEL_STATIONARY)
                        {
                            ublox_error("received cfg_nav5 not stationary");
                        }
                        if(cfg_nav5.utc_standard !=
                           UBX_CFG_NAV5_UTC_STANDARD_USNO)
                        {
                            ublox_error("received cfg_nav5 not USNO");
                        }
                        return UBLOX_CFG_NAV5;
                    } else {
                        ublox_error("unknown cfg msg");
                        return UBLOX_UNHANDLED;
                    }
                    break;
                default:
                    break;
            }
            break;

        default:
            state = STATE_IDLE;
            rxbufidx = 0;
            ublox_error("state machine default");
            return UBLOX_ERROR;

    }

    return UBLOX_WAIT;
}

static bool ublox_expect(enum ublox_result expected, const char* errmsg)
{
    enum ublox_result r;
    do {
        r = ublox_state_machine();
    } while(r == UBLOX_WAIT);

    if(r != expected) {
        ublox_error(errmsg);
        return false;
    }

    return true;
}

static bool ublox_tx_expect(uint8_t* buf, enum ublox_result exp, const char* err)
{
    if(!ublox_transmit(buf)) {
        return false;
    }

    if(!ublox_expect(exp, err)) {
        return false;
    }

    return true;
}

static bool ublox_configure(void)
{
    if(!ublox_configure_prt()) {
        return false;
    }

    if(!ublox_configure_nav5()) {
        return false;
    }

    if(!ublox_configure_rate()) {
        return false;
    }

    if(!ublox_configure_gnss()) {
        return false;
    }

    if(!ublox_configure_sbas()) {
        return false;
    }

    if(!ublox_configure_tp5()) {
        return false;
    }

    if(!ublox_configure_msg()) {
        return false;
    }

    return true;
}

static bool ublox_configure_prt(void)
{
    ubx_cfg_prt_t prt = {0};

    /* Disable NMEA on UART */
    prt.sync1 = UBX_SYNC1;
    prt.sync2 = UBX_SYNC2;
    prt.class = UBX_CFG;
    prt.id = UBX_CFG_PRT;
    prt.length = sizeof(prt.payload);

    /* Program UART1 */
    prt.port_id = 1;
    /* Don't use TXReady GPIO */
    prt.tx_ready = 0;
    /* 8 bits, no polarity, 1 stop bit */
    prt.mode = (1<<4) | (3<<6) | (4<<9);
    /* 9600 baud */
    prt.baud_rate = 9600;
    /* only receive UBX protocol */
    prt.in_proto_mask = (1<<0);
    /* only send UBX protocol */
    prt.out_proto_mask = (1<<0);
    /* no weird timeout */
    prt.flags = 0;

    /* Don't wait for an ACK for this as there'll be loads of shit
     * from NMEA mode wasting the buffer */
    if(!ublox_transmit((uint8_t*)&prt)) {
        return false;
    }

    /* Wait for it to stop barfing NMEA */
    chThdSleepMilliseconds(100);

    /* Clear the read buffer */
    while(sdGetTimeout(ublox_seriald, TIME_IMMEDIATE) != Q_TIMEOUT);

    return true;
}

static bool ublox_configure_nav5(void)
{
    ubx_cfg_nav5_t nav5 = {0};

    /* Set to stationary mode and UTC from USNO (GPS) */
    nav5.sync1 = UBX_SYNC1;
    nav5.sync2 = UBX_SYNC2;
    nav5.class = UBX_CFG;
    nav5.id = UBX_CFG_NAV5;
    nav5.length = sizeof(nav5.payload);

    /* Mask settings to just dynModel and utcStandard */
    nav5.mask = (1<<0) | (1<<10);

    /* Set dynModel and utcStandard */
    nav5.dyn_model = UBX_CFG_NAV5_DYN_MODEL_STATIONARY;
    nav5.utc_standard = UBX_CFG_NAV5_UTC_STANDARD_USNO;

    return ublox_tx_expect((uint8_t*)&nav5, UBLOX_ACK,
                           "configure_nav5 didn't get ack");
}

static bool ublox_configure_rate(void)
{
    ubx_cfg_rate_t rate = {0};

    /* Set solution rate to 1Hz */
    rate.sync1 = UBX_SYNC1;
    rate.sync2 = UBX_SYNC2;
    rate.class = UBX_CFG;
    rate.id = UBX_CFG_RATE;
    rate.length = sizeof(rate.payload);

    /* 1000ms between solutions, 1 solution per cycle, lock to UTC */
    rate.meas_rate = 1000;
    rate.nav_rate = 1;
    rate.time_ref = 0;

    return ublox_tx_expect((uint8_t*)&rate, UBLOX_ACK,
                           "configure_rate didn't get ack");
}

static bool ublox_configure_gnss(void)
{
    ubx_cfg_gnss_t gnss = {0};

    gnss.sync1 = UBX_SYNC1;
    gnss.sync2 = UBX_SYNC2;
    gnss.class = UBX_CFG;
    gnss.id = UBX_CFG_GNSS;
    gnss.length = sizeof(gnss.payload);

    gnss.msg_ver = 0;
    gnss.num_trk_ch_hw = 32;
    gnss.num_trk_ch_use = 32;
    gnss.num_config_blocks = 5;

    /* Enable GPS, use all channels */
    gnss.gps_gnss_id = UBX_CFG_GNSS_GNSS_ID_GPS;
    gnss.gps_res_trk_ch = 32;
    gnss.gps_max_trk_ch = 32;
    gnss.gps_flags = UBX_CFG_GNSS_FLAGS_ENABLE(1);

    /* Leave all other GNSS systems disabled */
    gnss.sbas_gnss_id = UBX_CFG_GNSS_GNSS_ID_SBAS;
    gnss.beidou_gnss_id = UBX_CFG_GNSS_GNSS_ID_BEIDOU;
    gnss.qzss_gnss_id = UBX_CFG_GNSS_GNSS_ID_QZSS;
    gnss.glonass_gnss_id = UBX_CFG_GNSS_GNSS_ID_GLONASS;

    return ublox_tx_expect((uint8_t*)&gnss, UBLOX_ACK,
                           "configure_gnss didn't get ack");
}

static bool ublox_configure_sbas(void)
{
    ubx_cfg_sbas_t sbas = {0};

    sbas.sync1 = UBX_SYNC1;
    sbas.sync2 = UBX_SYNC2;
    sbas.class = UBX_CFG;
    sbas.id = UBX_CFG_SBAS;
    sbas.length = sizeof(sbas.payload);

    /* Disable SBAS */
    sbas.mode = UBX_CFG_SBAS_MODE_ENABLED(0);

    return ublox_tx_expect((uint8_t*)&sbas, UBLOX_ACK,
                           "configure_sbas didn't get ack");
}

static bool ublox_configure_tp5(void)
{
    ubx_cfg_tp5_t tp5 = {0};

    tp5.sync1 = UBX_SYNC1;
    tp5.sync2 = UBX_SYNC2;
    tp5.class = UBX_CFG;
    tp5.id = UBX_CFG_TP5;
    tp5.length = sizeof(tp5.payload);

    /* Enable 1MHz output on TIMEPULSE, always output.
     */
    tp5.tp_idx               = 0;
    tp5.version              = 0;
    tp5.ant_cable_delay      = 15;
    tp5.freq_period          = 1000000;
    tp5.pulse_len_ratio      = 0xffffffff >> 1;
    tp5.freq_period_lock     = 1000000;
    tp5.pulse_len_ratio_lock = 0xffffffff >> 1;
    tp5.user_config_delay    = 0;
    tp5.flags = (
        UBX_CFG_TP5_FLAGS_ACTIVE                    |
        UBX_CFG_TP5_FLAGS_LOCK_GNSS_FREQ            |
        UBX_CFG_TP5_FLAGS_IS_FREQ                   |
        UBX_CFG_TP5_FLAGS_POLARITY                  |
        UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_UTC);

    if(!ublox_tx_expect((uint8_t*)&tp5, UBLOX_ACK,
                        "configure_tp5 didn't get first ack")) {
        return false;
    }

    /* Enable 1PPS output on TIMEPULSE,
     * only when GNSS lock is valid,
     * rising edge at top of second,
     * aligned to top of second.
     */
    tp5.tp_idx               = 1;
    tp5.version              = 0;
    tp5.ant_cable_delay      = 15;
    tp5.freq_period          = 0;
    tp5.pulse_len_ratio      = 1000;
    tp5.freq_period_lock     = 1;
    tp5.pulse_len_ratio_lock = 1000;
    tp5.flags = (
        UBX_CFG_TP5_FLAGS_ACTIVE                    |
        UBX_CFG_TP5_FLAGS_LOCK_GNSS_FREQ            |
        UBX_CFG_TP5_FLAGS_LOCKED_OTHER_SET          |
        UBX_CFG_TP5_FLAGS_IS_FREQ                   |
        UBX_CFG_TP5_FLAGS_IS_LENGTH                 |
        UBX_CFG_TP5_FLAGS_ALIGN_TO_TOW              |
        UBX_CFG_TP5_FLAGS_POLARITY                  |
        UBX_CFG_TP5_FLAGS_GRID_UTC_GNSS_UTC);

    return ublox_tx_expect((uint8_t*)&tp5, UBLOX_ACK,
                           "configure_tp5 didn't get second ack");
}

static bool ublox_configure_msg(void)
{
    ubx_cfg_msg_t msg = {0};

    msg.sync1 = UBX_SYNC1;
    msg.sync2 = UBX_SYNC2;
    msg.class = UBX_CFG;
    msg.id = UBX_CFG_MSG;
    msg.length = sizeof(msg.payload);

    /* Set NAV_PVT to 1 per second */
    msg.msg_class = UBX_NAV;
    msg.msg_id    = UBX_NAV_PVT;
    msg.rate      = 1;

    if(!ublox_tx_expect((uint8_t*)&msg, UBLOX_ACK,
                        "configure_msg didn't get first ack")) {
        return false;
    }

    /* Set NAV TIMELS to 1 per second */
/* Sadly this is only available in protocol 18 and up, but the FW2.01
 * modules I have are protocol 15...
 */
#if 0
    msg.msg_class = UBX_NAV;
    msg.msg_id    = UBX_NAV_TIMELS;
    msg.rate      = 1;

    if(!ublox_tx_expect((uint8_t*)&msg, UBLOX_ACK,
                        "configure_msg didn't get second ack")) {
        return false;
    }
#endif

    /* Set TIM_TP to 1 per second */
    msg.msg_class = UBX_TIM;
    msg.msg_id    = UBX_TIM_TP;
    msg.rate      = 1;

    return ublox_tx_expect((uint8_t*)&msg, UBLOX_ACK,
                           "configure_msg didn't get third ack");
}

static THD_WORKING_AREA(ublox_thd_wa, 512);
static THD_FUNCTION(ublox_thd, arg) {
    (void)arg;
    /* We'll reset the uBlox so it's in a known state */
    palClearLine(LINE_GPS_RESET);
    chThdSleepMilliseconds(100);
    palSetLine(LINE_GPS_RESET);
    chThdSleepMilliseconds(500);

    sdStart(ublox_seriald, &serial_cfg);

    while(!ublox_configure()) {
        ublox_error("configuration failed");
        chThdSleepMilliseconds(1000);
    }

    while(true) {
        ublox_state_machine();
    }
}

void ublox_init(SerialDriver* seriald) {
    /* Store a reference to the serial driver to use */
    ublox_seriald = seriald;

    /* Ensure the time is not yet valid */
    ublox_upcoming_tp_time.valid = false;

    /* Start up the ublox processing thread */
    chThdCreateStatic(ublox_thd_wa, sizeof(ublox_thd_wa), NORMALPRIO,
                      ublox_thd, NULL);
}
