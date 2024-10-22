#include <stdint.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "ff.h"
#include "chprintf.h"
#include "microsd.h"

#define LOG_MAGIC 0xABCDEF00

static bool microsd_card_init(FATFS* fs);
static void microsd_card_try_init(FATFS* fs);
static void microsd_card_deinit(void);

/* Heap object and buffer (used to store pending items) */
static memory_heap_t microsd_heap;
static uint8_t microsd_heap_buf[16*1024]
    __attribute__((aligned(sizeof(stkalign_t))));

/* Mailbox object and buffer */
static msg_t microsd_mbox_buf[256]
    __attribute__((aligned(sizeof(stkalign_t))));
static MAILBOX_DECL(microsd_mbox, microsd_mbox_buf, 256);

/* MicroSD write buffer */
static uint8_t microsd_card_buf[16*1024]
    __attribute__((section(".sram2")))
    __attribute__((aligned(sizeof(stkalign_t))));

static bool microsd_card_init(FATFS* fs)
{
    static SDCConfig sdc_cfg = {
        .scratchpad = NULL,
        .bus_width = SDC_MODE_1BIT,
    };
    sdcStart(&SDCD1, &sdc_cfg);

    if(sdcConnect(&SDCD1) != MSG_OK) {
        return false;
    }

    return f_mount(fs, "A", 0) == FR_OK;
}

static void microsd_card_try_init(FATFS* fs)
{
    while(!microsd_card_init(fs)) {
        microsd_card_deinit();
        chThdSleepMilliseconds(200);
    }
}

static void microsd_card_deinit()
{
    f_mount(NULL, "A", 0);
    sdcDisconnect(&SDCD1);
    sdcStop(&SDCD1);
}

FRESULT microsd_open_file_inc(FIL* fp)
{
    FRESULT err;
    uint8_t mode = FA_WRITE | FA_CREATE_NEW;
    uint32_t file_idx = 0;
    char fname[32];

    while(true) {
        file_idx++;
        chsnprintf(fname, 25, "log_%05d.bin", file_idx);
        err = f_open(fp, fname, mode);
        if(err == FR_EXIST) {
            continue;
        } else {
            return err;
        }
    }
}

FRESULT microsd_write_buf(FIL* fp)
{
    FRESULT err;
    size_t bytes_written;

    err = f_write(fp, microsd_card_buf, sizeof(microsd_card_buf),
                  &bytes_written);
    f_sync(fp);

    if(bytes_written != sizeof(microsd_card_buf)) {
        err = FR_DENIED;
    }

    return err;
}

static THD_WORKING_AREA(microsd_thd_wa, 4096);
static THD_FUNCTION(microsd_thd, arg)
{
    (void)arg;
    chRegSetThreadName("microsd");

    FATFS file_system;
    FIL file;
    msg_t mailbox_r;
    void* mailbox_p;
    uint8_t* card_buf_p = microsd_card_buf;
    size_t entry_len;

    palSetLine(LINE_LED_YLW);
    microsd_card_try_init(&file_system);
    while(microsd_open_file_inc(&file) != FR_OK);
    palClearLine(LINE_LED_YLW);

    while(true) {
        /* Block getting a new pointer from mailbox */
        mailbox_r = chMBFetch(&microsd_mbox, (msg_t*)&mailbox_p,
                              TIME_INFINITE);
        if(mailbox_r != MSG_OK || mailbox_p == 0) {
            continue;
        }

        /* Extract size of this entry and sanity check it */
        entry_len = ((uint32_t*)mailbox_p)[1] + 8;
        if(entry_len > 1024) {
            continue;
        }

        /* If there's not enough space for it in the write buffer,
         * dump the write buffer to card.
         */
        size_t buf_used = (size_t)(card_buf_p - microsd_card_buf);
        size_t buf_space = sizeof(microsd_card_buf) - buf_used;
        if(buf_space < entry_len) {
            palSetLine(LINE_LED_YLW);

            /* Zero out the unused buffer space */
            memset(card_buf_p, 0, buf_space);

            FRESULT err = microsd_write_buf(&file);

            /* If writing failed, reopen the card and keep retrying */
            while(err != FR_OK) {
                f_close(&file);
                microsd_card_deinit();
                microsd_card_try_init(&file_system);
                if(microsd_open_file_inc(&file)) {
                    err = microsd_write_buf(&file);
                }
            }

            card_buf_p = microsd_card_buf;
            palClearLine(LINE_LED_YLW);
        }

        /* Copy entry into write buffer */
        memcpy(card_buf_p, mailbox_p, entry_len);
        card_buf_p += entry_len;

        /* Free entry from heap */
        chHeapFree(mailbox_p);
    }
}

void microsd_init()
{
    chHeapObjectInit(&microsd_heap, microsd_heap_buf,
                     sizeof(microsd_heap_buf));
    memset(microsd_card_buf, 0, sizeof(microsd_card_buf));

    chThdCreateStatic(microsd_thd_wa, sizeof(microsd_thd_wa),
                      NORMALPRIO, microsd_thd, NULL);
}

void microsd_log(uint8_t tag, size_t len, void* data)
{
    /* Allocate space for new thing on heap, copy it in. */
    uint32_t* p = (uint32_t*)chHeapAlloc(&microsd_heap, len + 8);
    if(p == NULL) {
        return;
    }

    /* Copy the new thing into the allocated space */
    p[0] = LOG_MAGIC | tag;
    p[1] = len;
    memcpy(&p[2], data, len);

    /* Try to post the new thing, free immediately if we can't */
    if(chMBPost(&microsd_mbox, (intptr_t)p, TIME_IMMEDIATE) != MSG_OK) {
        chHeapFree(p);
    }
}
