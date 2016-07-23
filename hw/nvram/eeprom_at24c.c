/*
 * *AT24C* series I2C EEPROM
 *
 * Copyright (c) 2015 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "hw/nvram/eeprom_at24c.h"

/* #define DEBUG_AT24C */

#ifdef DEBUG_AT24C
#define DPRINTK(FMT, ...) printf(TYPE_AT24C_EE " : " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

#define LOG(MSK, FMT, ...) qemu_log_mask(MSK, TYPE_AT24C_EE " : " FMT, \
                            ## __VA_ARGS__)

#define TYPE_AT24C_EE "at24c-eeprom"
#define AT24C_EE(obj) OBJECT_CHECK(EEPROMState, (obj), TYPE_AT24C_EE)

typedef struct EEPROMState {
    I2CSlave parent_obj;

    uint16_t cur;
    uint32_t rsize;
    bool writable;
    uint8_t haveaddr;

    uint8_t *mem;
} EEPROMState;

static
int at24c_eeprom_event(I2CSlave *s, enum i2c_event event)
{
    EEPROMState *ee = container_of(s, EEPROMState, parent_obj);

    switch (event) {
    case I2C_START_SEND:
    case I2C_START_RECV:
    case I2C_FINISH:
        ee->haveaddr = 0;
        DPRINTK("clear\n");
        break;
    case I2C_NACK:
        break;
    }
    return 0;
}

static
int at24c_eeprom_recv(I2CSlave *s)
{
    EEPROMState *ee = container_of(s, EEPROMState, parent_obj);
    int ret;

    if (ee->cur < ee->rsize) {
        /* read 1 or more bytes */

        ret = ee->mem[ee->cur++];
        DPRINTK("Recv %02x %c\n", ret, ret);

    } else {
        ret = -1;
        DPRINTK("Recv error\n");
    }
    return ret;
}

static
int at24c_eeprom_send(I2CSlave *s, uint8_t data)
{
    EEPROMState *ee = container_of(s, EEPROMState, parent_obj);

    if (ee->haveaddr < 2) {
        ee->cur <<= 8;
        ee->cur |= data;
        ee->haveaddr++;
        if (ee->haveaddr == 2) {
            DPRINTK("Set pointer %04x\n", ee->cur);
        }
        return 0;

    } else if (ee->cur < ee->rsize && ee->writable) {
        DPRINTK("Send %02x\n", data);
        ee->mem[ee->cur++] = data;
        return 0;

    } else {
        DPRINTK("Send error %02x\n", data);
        return -1;
    }
}

static
int at24c_eeprom_init(I2CSlave *i2c)
{
    EEPROMState *ee = AT24C_EE(i2c);

    ee->mem = g_malloc0(ee->rsize);
    return 0;
}

static Property ee_props[] = {
    DEFINE_PROP_UINT32("rom-size", EEPROMState, rsize, 0),
    DEFINE_PROP_BOOL("writable", EEPROMState, writable, false),
    DEFINE_PROP_END_OF_LIST()
};

static
void at24c_eeprom_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = &at24c_eeprom_init;
    k->event = &at24c_eeprom_event;
    k->recv = &at24c_eeprom_recv;
    k->send = &at24c_eeprom_send;

    dc->props = ee_props;
    /* TODO: handle reset?  that to do? */
}

static
const TypeInfo at24c_eeprom_type = {
    .name = TYPE_AT24C_EE,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(EEPROMState),
    .class_size = sizeof(I2CSlaveClass),
    .class_init = at24c_eeprom_class_init,
};

static void at24c_eeprom_register(void)
{
    type_register_static(&at24c_eeprom_type);
}

type_init(at24c_eeprom_register)

/* Allows to populate eeprom programatically */
void at24c_eeprom_write(DeviceState *dev,
                           uint32_t start, uint32_t count,
                           const char *buf)
{
    EEPROMState *ee = AT24C_EE(dev);

    if (start >= ee->rsize || ee->rsize - start < count) {
        hw_error("eeprom write out of bounds.  Allowed [0:%u) "
                 "requested [%u:%u)\n", (unsigned)ee->rsize,
                 (unsigned)start, (unsigned)(start + count));
    }

    memcpy(ee->mem + start, buf, count);
}
