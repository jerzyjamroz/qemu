/*
 * MPC8540 I2C bus interface
 * As described in
 * MPC8540 PowerQUICC III Integrated Host Processor Reference Manual, Rev. 1
 * Part 2 chapter 11
 *
 * Copyright (c) 2015 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "hw/sysbus.h"

/* #define DEBUG_LVL 0 */

#ifdef DEBUG_LVL
#define DPRINTK(LVL, FMT, ...) do { if ((LVL) <= DEBUG_LVL) { \
    printf(TYPE_MPC8540_I2C " : " FMT, ## __VA_ARGS__); } } while (0)
#else
#define DPRINTK(LVL, FMT, ...) do {} while (0)
#endif

#define LOG(MSK, FMT, ...) qemu_log_mask(MSK, TYPE_MPC8540_I2C \
    " : " FMT, ## __VA_ARGS__)

#define TYPE_MPC8540_I2C "mpc8540-i2c"
#define MPC8540_I2C(obj) OBJECT_CHECK(I2CState, (obj), TYPE_MPC8540_I2C)

typedef struct I2CState {
    SysBusDevice parent_obj;

    I2CBus *bus;

    uint8_t ctrl, sts;
    uint8_t freq, filt;
    uint8_t dbuf;

    qemu_irq irq;

    MemoryRegion mmio;
} I2CState;

static
void mpc8540_update_irq(I2CState *i2c)
{
    int ena = i2c->ctrl & 0x40,
        sts = i2c->sts & 0x02,
        act = !!(ena && sts);

    DPRINTK(1, "IRQ %c ena %c sts %c\n",
            act ? 'X' : '_',
            ena ? 'X' : '_',
            sts ? 'X' : '_');

    qemu_set_irq(i2c->irq, act);
}

static
uint64_t mpc8540_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    I2CState *i2c = opaque;
    uint32_t val, offset = addr;

    switch (offset) {
    case 0x0: /* ADDR */
        val = 0;
        break;
    case 0x4: /* Freq Div. */
        val = i2c->freq;
        break;
    case 0x8: /* CONTROL */
        val = i2c->ctrl & ~0x06;
        break;
    case 0xc: /* STATUS */
        val = i2c->sts;
        break;
    case 0x10: /* DATA */
        /* Reads are "pipelined" and so return the previous value of the
         * register
         */
        val = i2c->dbuf;
        if ((i2c->ctrl & 0x80) && (i2c->sts & 0x20)) { /* enabled and busy */
            if (!i2c_bus_busy(i2c->bus) || (i2c->ctrl & 0x10)) {
                LOG(LOG_GUEST_ERROR, "Read during addr or tx\n");
                i2c->dbuf = 0xff;
            } else {
                int ret = i2c_recv(i2c->bus);
                i2c->dbuf = (uint8_t)ret;
                DPRINTK(0, "READ %02x ('%c')\n", i2c->dbuf, (char)i2c->dbuf);
                i2c->sts |= 0x02;
                i2c->sts &= ~0x01;
                mpc8540_update_irq(i2c);
            }
        } else {
            i2c->dbuf = 0xff;
            LOG(LOG_GUEST_ERROR, "Read when not enabled or busy\n");
        }
        break;
    case 0x14: /* FILTER */
        val = i2c->filt;
        break;
    default:
        val = 0xff;
    }

    DPRINTK(offset == 0xc ? 2 : 1, " read %08x -> %08x\n",
            (unsigned)offset, (unsigned)val);
    return val;
}

static
void mpc8540_i2c_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    I2CState *i2c = opaque;
    uint32_t offset = addr;

    DPRINTK(1, " write %08x <- %08x\n", (unsigned)offset, (unsigned)val);

    switch (offset) {
    case 0x0: /* ADDR */
        break;
    case 0x4: /* Freq Div. */
        i2c->freq = val & 0x3f;
        break;
    case 0x8: /* CONTROL CCR */
        val &= ~0x02;
        if ((val & 0x20) && !(i2c->ctrl & 0x20)) {
            /* MSTA 0 -> 1 is START */

            i2c->sts |= 0x20; /* set MBB (busy) */
            DPRINTK(0, "START\n");
            i2c_end_transfer(i2c->bus); /* paranoia */
        }
        if (!(val & 0x20) && (i2c->ctrl & 0x20)) {
            /* MSTA 1 -> 0 is STOP */

            i2c->sts &= ~0x20; /* clear MBB (busy) */
            DPRINTK(0, "STOP\n");
            i2c_end_transfer(i2c->bus);
        }
        if (val & 0x04) {
            i2c_end_transfer(i2c->bus);
            i2c->sts |= 0x20; /* set MBB (busy) */
            DPRINTK(0, "REP START\n");
        }
        i2c->ctrl = val;
        mpc8540_update_irq(i2c);
        break;
    case 0xc: /* STATUS CSR */
        val &= 0x12;
        i2c->sts &= ~0x12;
        i2c->sts |= val;
        mpc8540_update_irq(i2c);
        break;
    case 0x10: /* DATA CDR */
        if ((i2c->ctrl & 0x80) && (i2c->sts & 0x20)) { /* enabled and busy */
            if (!i2c_bus_busy(i2c->bus)) {
                if (i2c_start_transfer(i2c->bus, val >> 1, val & 1)) {
                    LOG(LOG_GUEST_ERROR, "ADDR no device %02x\n",
                        (unsigned)(val & 0xfe));
                } else {
                    DPRINTK(0, "ADDR %02x\n", (unsigned)(val & 0xfe));
                }
                i2c->sts |= 0x02;
                i2c->sts &= ~0x01;

            } else if (i2c->ctrl & 0x10) {
                DPRINTK(0, "WRITE %02x\n", (unsigned)val);
                i2c_send(i2c->bus, val);
                i2c->sts |= 0x02;
                i2c->sts &= ~0x01;
            } else {
                LOG(LOG_GUEST_ERROR, "Write during read\n");
            }
            mpc8540_update_irq(i2c);
        } else {
            LOG(LOG_GUEST_ERROR, "Write when not enabled or busy\n");
        }
        break;
    case 0x14: /* FILTER */
        val &= 0x3f;
        i2c->filt = val;
        break;
    }
}

static const MemoryRegionOps i2c_ops = {
    .read = mpc8540_i2c_read,
    .write = mpc8540_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static
void mpc8540_i2c_reset(DeviceState *dev)
{
    I2CState *i2c = MPC8540_I2C(dev);

    i2c->sts = 0x81; /* transfer complete and ack received */
}

static int mpc8540_i2c_inst_init(SysBusDevice *dev)
{
    I2CState *i2c = MPC8540_I2C(dev);

    /* TODO: inplace or cleanup */
    i2c->bus = i2c_init_bus(&dev->parent_obj, "bus");

    memory_region_init_io(&i2c->mmio, &dev->parent_obj.parent_obj,
                          &i2c_ops, i2c, TYPE_MPC8540_I2C, 0x18);

    sysbus_init_mmio(dev, &i2c->mmio);
    sysbus_init_irq(dev, &i2c->irq);
    return 0;
}

static void mpc8540_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = &mpc8540_i2c_inst_init;
    dc->reset = &mpc8540_i2c_reset;
}

static const TypeInfo mpc8540_i2c_type = {
    .name = TYPE_MPC8540_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(I2CState),
    .class_size = sizeof(SysBusDeviceClass),
    .class_init = mpc8540_i2c_class_init,
};

static void mpc8540_i2c_register(void)
{
    type_register_static(&mpc8540_i2c_type);
}

type_init(mpc8540_i2c_register)
