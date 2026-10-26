// Generic interrupt based serial uart helper code
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/io.h" // readb
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "board/pgm.h" // READP
#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_wake_tasks
#include "serial_irq.h" // serial_enable_tx_irq

#define RX_BUFFER_SIZE 192
#if CONFIG_SERIAL_RS485_TDD
#define TX_BUFFER_SIZE 240
#else
#define TX_BUFFER_SIZE 96
#endif

static uint8_t receive_buf[RX_BUFFER_SIZE], receive_pos;
static uint8_t transmit_buf[TX_BUFFER_SIZE], transmit_pos, transmit_max;
#if CONFIG_SERIAL_RS485_TDD
static uint8_t transmit_commit;
DECL_CONSTANT("SERIAL_RS485_TDD", 1);
#endif

DECL_CONSTANT("SERIAL_BAUD", CONFIG_SERIAL_BAUD);
DECL_CONSTANT("RECEIVE_WINDOW", RX_BUFFER_SIZE);

// Rx interrupt - store read data
void
serial_rx_byte(uint_fast8_t data)
{
    if (data == MESSAGE_SYNC)
        sched_wake_tasks();
    if (receive_pos >= sizeof(receive_buf))
        // Serial overflow - ignore it as crc error will force retransmit
        return;
    receive_buf[receive_pos++] = data;
}

// Tx interrupt - get next byte to transmit
int
serial_get_tx_byte(uint8_t *pdata)
{
#if CONFIG_SERIAL_RS485_TDD
    if (transmit_pos >= transmit_commit)
        return -1;
#else
    if (transmit_pos >= transmit_max)
        return -1;
#endif
    *pdata = transmit_buf[transmit_pos++];
    return 0;
}

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(uint_fast8_t len)
{
    uint_fast8_t copied = 0;
    for (;;) {
        uint_fast8_t rpos = readb(&receive_pos);
        uint_fast8_t needcopy = rpos - len;
        if (needcopy) {
            memmove(&receive_buf[copied], &receive_buf[copied + len]
                    , needcopy - copied);
            copied = needcopy;
            sched_wake_tasks();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readb(&receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

// Process any incoming commands
void
console_task(void)
{
    uint_fast8_t rpos = readb(&receive_pos), pop_count;
    int_fast8_t ret = command_find_block(receive_buf, rpos, &pop_count);
    if (ret > 0)
        command_dispatch(receive_buf, pop_count);
    if (ret) {
        if (CONFIG_HAVE_BOOTLOADER_REQUEST && ret < 0 && pop_count == 32
            && !memcmp(receive_buf, " \x1c Request Serial Bootloader!! ~", 32))
            bootloader_request();
        console_pop_input(pop_count);
        if (ret > 0)
            command_send_ack();
#if CONFIG_SERIAL_RS485_TDD
        // Our turn on the bus only after a valid block (in sequence, or
        // not: -2, nak queued): a stray byte at a bus turnaround must not
        // start a transmission that would collide with the host's next
        // block
        if (ret > 0 || ret == -2) {
            writeb(&transmit_commit, readb(&transmit_max));
            if (readb(&transmit_pos) < readb(&transmit_commit))
                serial_enable_tx_irq();
        }
#endif
    }
}
DECL_TASK(console_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Verify space for message
    uint_fast8_t tpos = readb(&transmit_pos), tmax = readb(&transmit_max);
    if (tpos >= tmax) {
        tpos = tmax = 0;
        writeb(&transmit_max, 0);
#if CONFIG_SERIAL_RS485_TDD
        writeb(&transmit_commit, 0);
#endif
        writeb(&transmit_pos, 0);
    }
    uint_fast8_t max_size = READP(ce->max_size);
    uint_fast8_t limit = sizeof(transmit_buf);
#if CONFIG_SERIAL_RS485_TDD
    if (max_size > MESSAGE_MIN)
        limit -= MESSAGE_MIN;
#endif
    if (tmax + max_size > limit) {
        if (tmax - tpos + READP(ce->min_size) > limit)
            // Not enough space for message
            return;
        // Disable TX irq and move buffer
        writeb(&transmit_max, 0);
#if CONFIG_SERIAL_RS485_TDD
        uint_fast8_t tcommit = readb(&transmit_commit);
        writeb(&transmit_commit, 0);
#endif
        tpos = readb(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        writeb(&transmit_pos, 0);
#if CONFIG_SERIAL_RS485_TDD
        tcommit = tcommit > tpos ? tcommit - tpos : 0;
        writeb(&transmit_commit, tcommit);
        writeb(&transmit_max, tmax);
        if (tcommit)
            serial_enable_tx_irq();
#else
        writeb(&transmit_max, tmax);
        serial_enable_tx_irq();
#endif
    }

    // Generate message
    uint8_t *buf = &transmit_buf[tmax];
    uint_fast8_t msglen = command_encode_and_frame(
        buf, limit - tmax, ce, args);
    if (!msglen)
        return;

    // Start message transmit
    writeb(&transmit_max, tmax + msglen);
#if !CONFIG_SERIAL_RS485_TDD
    serial_enable_tx_irq();
#endif
}
