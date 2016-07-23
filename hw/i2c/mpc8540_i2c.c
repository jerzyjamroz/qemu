/*
 * MPC8540 I2C bus interface
 * As described in
 * MPC8540 PowerQUICC III Integrated Host Processor Reference Manual, Rev. 1
 * Part 2 chapter 11
 *
 * Compatible I2C controllers are found on other Freescale chips
 * including mpc8544 and P2010.
 *
 * Copyright (c) 2015 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/i2c/i2c.h"
#include "hw/sysbus.h"
#include "qemu/error-report.h"

#include "trace.h"

/* #define DEBUG_LVL 0 */

#ifdef DEBUG_LVL
#define DPRINTK(LVL, FMT, ...) do { \
    if ((LVL) <= DEBUG_LVL) {\
        info_report(TYPE_MPC8540_I2C " : " FMT, ## __VA_ARGS__); \
    } } while (0)
#else
#define DPRINTK(LVL, FMT, ...) do {} while (0)
#endif

#define LOG(MSK, FMT, ...) qemu_log_mask(MSK, TYPE_MPC8540_I2C \
    " : " FMT "\n", ## __VA_ARGS__)

#define TYPE_MPC8540_I2C "mpc8540-i2c"
#define MPC8540_I2C(obj) OBJECT_CHECK(MPC8540I2CState, (obj), TYPE_MPC8540_I2C)

/* offsets relative to CCSR offset 0x3000 */
#define R_I2CADR (0)
#define R_I2CFDR (4)
#define R_I2CCR  (8)
#define R_I2CSR  (0xc)
#define R_I2CDR  (0x10)
#define R_I2CDFSRR (0x14)

FIELD(I2CCR, MEN, 7, 1)
FIELD(I2CCR, MIEN, 6, 1)
FIELD(I2CCR, MSTA, 5, 1)
FIELD(I2CCR, MTX, 4, 1)
FIELD(I2CCR, TXAK, 3, 1)
FIELD(I2CCR, RSTA, 2, 1)
FIELD(I2CCR, BCST, 0, 1)

FIELD(I2CSR, MCF, 7, 1)
FIELD(I2CSR, MAAS, 6, 1)
FIELD(I2CSR, MBB, 5, 1)
FIELD(I2CSR, MAL, 4, 1)
FIELD(I2CSR, BCSTM, 3, 1)
FIELD(I2CSR, SRW, 2, 1)
FIELD(I2CSR, MIF, 1, 1)
FIELD(I2CSR, RXAK, 0, 1)

typedef struct MPC8540I2CState {
    SysBusDevice parent_obj;

    I2CBus *bus;

    uint8_t ctrl, sts;
    uint8_t freq, filt;
    /* Reads are pipelined, this is the next data value */
    uint8_t dbuf, dbuf_valid;

    qemu_irq irq;

    MemoryRegion mmio;
} MPC8540I2CState;

#define I2CCR(BIT) FIELD_EX32(i2c->ctrl, I2CCR, BIT)
#define I2CSR(BIT) FIELD_EX32(i2c->sts, I2CSR, BIT)

#define I2CSR_SET(BIT, VAL) do {\
        i2c->sts = FIELD_DP32(i2c->sts, I2CSR, BIT, VAL);\
    } while (0)

static
void mpc8540_update_irq(MPC8540I2CState *i2c)
{
    int ena = i2c->ctrl & 0x40,
        sts = i2c->sts & 0x02,
        act = !!(ena && sts);

    DPRINTK(1, "IRQ %c ena %c sts %c",
            act ? 'X' : '_',
            ena ? 'X' : '_',
            sts ? 'X' : '_');

    qemu_set_irq(i2c->irq, act);
}

static
uint64_t mpc8540_i2c_read(void *opaque, hwaddr addr, unsigned size)
{
    MPC8540I2CState *i2c = opaque;
    uint32_t val;

    switch (addr) {
    case R_I2CADR: /* ADDR */
        val = 0;
        break;
    case R_I2CFDR: /* Freq Div. */
        val = i2c->freq;
        break;
    case R_I2CCR: /* CONTROL */
        val = i2c->ctrl & ~0x06;
        break;
    case R_I2CSR: /* STATUS */
        val = i2c->sts;
        break;
    case R_I2CDR: /* DATA */
        /* Reads are "pipelined" and so return the previous value of the
         * register
         */
        val = i2c->dbuf;
        if (I2CCR(MEN) && I2CSR(MBB)) { /* enabled and busy */
            if (!i2c_bus_busy(i2c->bus) || I2CCR(MTX)) {
                if (!i2c->dbuf_valid) {
                    LOG(LOG_GUEST_ERROR, "Read during addr or tx");
                }
                i2c->dbuf = 0xff;
                i2c->dbuf_valid = false;
            } else {
                int ret = i2c_recv(i2c->bus);
                i2c->dbuf = (uint8_t)ret;
                i2c->dbuf_valid = true;
                trace_mpc8540_i2c_read(i2c->dbuf);
                I2CSR_SET(MIF, 1);
                I2CSR_SET(RXAK, 0);
                mpc8540_update_irq(i2c);
            }
        } else {
            i2c->dbuf = 0xff;
            i2c->dbuf_valid = false;
            LOG(LOG_GUEST_ERROR, "Read when not enabled or busy");
        }
        break;
    case R_I2CDFSRR: /* FILTER */
        val = i2c->filt;
        break;
    default:
        val = 0xff;
    }

    DPRINTK(addr == 0xc ? 2 : 1, " read %08x -> %08x",
            (unsigned)addr, (unsigned)val);
    return val;
}

static
void mpc8540_i2c_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    MPC8540I2CState *i2c = opaque;

    DPRINTK(1, " write %08x <- %08x", (unsigned)addr, (unsigned)val);

    switch (addr) {
    case R_I2CADR: /* ADDR */
        break;
    case R_I2CFDR: /* Freq Div. */
        i2c->freq = val & 0x3f;
        break;
    case R_I2CCR: /* CONTROL CCR */
        if (!FIELD_EX32(val, I2CCR, MEN)) {
            DPRINTK(0, "Not Enabled");

        } else if (!I2CCR(MSTA) && FIELD_EX32(val, I2CCR, MSTA)) {
            /* MSTA 0 -> 1 is START */

            I2CSR_SET(MBB, 1);
            if (I2CCR(MTX)) {
                trace_mpc8540_i2c_event("START Tx");
            } else {
                trace_mpc8540_i2c_event("START Rx");
            }
            i2c_end_transfer(i2c->bus); /* paranoia */

        } else if (I2CCR(MSTA) && !FIELD_EX32(val, I2CCR, MSTA)) {
            /* MSTA 1 -> 0 is STOP */

            I2CSR_SET(MBB, 0);
            trace_mpc8540_i2c_event("STOP");
            i2c_end_transfer(i2c->bus);

        } else if (I2CCR(MSTA) && FIELD_EX32(val, I2CCR, RSTA)) {
            i2c_end_transfer(i2c->bus);
            I2CSR_SET(MBB, 1);
            if (I2CCR(MTX)) {
                trace_mpc8540_i2c_event("REPEAT START Tx");
            } else {
                trace_mpc8540_i2c_event("REPEAT START Rx");
            }

        }
        /* RSTA always reads zero, bit 1 unusd */
        val &= 0xf9;
        i2c->ctrl = val;
        mpc8540_update_irq(i2c);
        break;
    case R_I2CSR: /* STATUS CSR */
        /* only MAL and MIF are writable */
        val &= 0x12;
        i2c->sts &= ~0x12;
        i2c->sts |= val;
        mpc8540_update_irq(i2c);
        break;
    case R_I2CDR: /* DATA CDR */
        if (I2CCR(MEN) && I2CSR(MBB)) { /* enabled and busy */
            if (!i2c_bus_busy(i2c->bus)) {
                if (i2c_start_transfer(i2c->bus, val >> 1, val & 1)) {
                    LOG(LOG_GUEST_ERROR, "I2C no device %02x",
                        (unsigned)(val & 0xfe));
                } else {
                    trace_mpc8540_i2c_address((unsigned)(val & 0xfe),
                                              (val & 0x1) ? 'R' : 'T');
                }
                I2CSR_SET(MIF, 1);
                I2CSR_SET(RXAK, 0);

            } else if (I2CCR(MTX)) {
                trace_mpc8540_i2c_write((unsigned)val);
                i2c_send(i2c->bus, val);
                I2CSR_SET(MIF, 1);
                I2CSR_SET(RXAK, 0);
            } else {
                LOG(LOG_GUEST_ERROR, "I2CDR Write during read");
            }
            mpc8540_update_irq(i2c);
        } else {
            LOG(LOG_GUEST_ERROR, "I2CDR Write when not enabled or busy");
        }
        break;
    case R_I2CDFSRR: /* FILTER */
        val &= 0x3f;
        i2c->filt = val;
        break;
    }

    DPRINTK(1, "I2CCR = %02x I2SCR = %02x", i2c->ctrl, i2c->sts);
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
    MPC8540I2CState *i2c = MPC8540_I2C(dev);

    i2c->sts = 0x81; /* transfer complete and ack received */
    i2c->dbuf_valid = false;
}

static void mpc8540_i2c_inst_init(DeviceState *dev, Error **errp)
{
    MPC8540I2CState *i2c = MPC8540_I2C(dev);

    i2c->bus = i2c_init_bus(dev, "bus");

    memory_region_init_io(&i2c->mmio, OBJECT(dev),
                          &i2c_ops, i2c, TYPE_MPC8540_I2C, 0x18);

    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &i2c->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &i2c->irq);
}

static void mpc8540_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = &mpc8540_i2c_inst_init;
    dc->reset = &mpc8540_i2c_reset;
}

static const TypeInfo mpc8540_i2c_type = {
    .name = TYPE_MPC8540_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MPC8540I2CState),
    .class_size = sizeof(SysBusDeviceClass),
    .class_init = mpc8540_i2c_class_init,
};

static void mpc8540_i2c_register(void)
{
    type_register_static(&mpc8540_i2c_type);
}

type_init(mpc8540_i2c_register)
