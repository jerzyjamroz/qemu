/*
 * QTest I2C driver
 *
 * Copyright (c) 2016 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "libqos/i2c.h"


#include "qemu/bswap.h"
#include "libqtest.h"

typedef struct E500I2C {
    I2CAdapter parent;

    uint64_t addr;
} E500I2C;

static void e500_i2c_send(I2CAdapter *i2c, uint8_t addr,
                          const uint8_t *buf, uint16_t len)
{
    E500I2C *s = (E500I2C *)i2c;

    writeb(s->addr + 0x8, 0xb0); /* Enable and START a write */
    writeb(s->addr + 0x10, addr & 0xfe); /* Send address for write */

    while (len--) {
        writeb(s->addr + 0x10, *buf++);
    }

    writeb(s->addr + 0x8, 0x80); /* STOP but leave enabled */
}

static void e500_i2c_recv(I2CAdapter *i2c, uint8_t addr,
                          uint8_t *buf, uint16_t len)
{
    E500I2C *s = (E500I2C *)i2c;

    writeb(s->addr + 0x8, 0xa0); /* Enable and START a read */
    writeb(s->addr + 0x10, addr | 1); /* Send address for read */

    /* reads are "pipelined" so the initial value is junk */
    readb(s->addr + 0x10);

    while (len--) {
        *buf++ = readb(s->addr + 0x10);
    }

    writeb(s->addr + 0x8, 0x80); /* STOP but leave enabled */
}

I2CAdapter *e500_i2c_create(uint64_t ccsr_base)
{
    E500I2C *s = g_malloc0(sizeof(*s));
    I2CAdapter *i2c = (I2CAdapter *)s;

    s->addr = ccsr_base + 0x3000;

    i2c->send = e500_i2c_send;
    i2c->recv = e500_i2c_recv;

    return i2c;
}
