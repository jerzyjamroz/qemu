/*
 * QEMU MRF EVR emulation
 *
 * Michael Davisdaver
 * Copyright (c) 2015
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */


#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/bswap.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qemu/error-report.h"

#include "mrf_core.h"

//#define MRF_DEBUG 1

#define TYPE_MRF_EVR "mrf-evr"

#ifdef MRF_DEBUG
#define	DBGOUT(LVL,fmt, ...) do{if(MRF_DEBUG>=(LVL)) info_report(TYPE_MRF_EVR ": %s() " fmt, __func__, ## __VA_ARGS__); } while(0)
#else
#define	DBGOUT(LVL,fmt, ...) do {} while (0)
#endif

#define ERR(mask, fmt, ...)  qemu_log_mask(mask, TYPE_MRF_EVR ": " fmt "\n", ## __VA_ARGS__)

#define NELEM(N) (sizeof(N)/sizeof((N)[0]))

#define MRF_EVR(obj) \
    OBJECT_CHECK(EVRState, (obj), TYPE_MRF_EVR)

typedef struct {
    uint32_t sec, count;
    uint8_t code;
} EVRFifoEntry;

typedef struct EVRState {
    /*< private >*/
    MRFCore parent_obj;

    CharBackend chr; /* event link */

    qemu_irq irq;

    /* evr */
    uint8_t mrftype;
    uint32_t fwversion;
    /* bus bridge type
     * 0 - external PCI, LE byte order, ignore PCI IRQ mask
     * 1 - internal PCI, we manage byte order and honor PCI IRQ mask
     * 2 - VME, we manage byte order, ignore PCI IRQ mask
     */
    uint8_t bridgetype;

    bool masterena;
    bool evrbe;
    bool pciint;
    bool linkup;
    bool linkupprev;

    EVRFifoEntry fifo[512];
    unsigned fifo_in, fifo_out; /* buffer pointers */

    // next index in dbuf RX array to write
    uint32_t dbuf_ptr;

    uint32_t evrreg[64*1024/4];

    uint8_t rxbuf[4];
    unsigned rxbuflen;
} EVRState;

static
void mrf_evr_update(EVRState *s)
{
    uint32_t rawirq = 0, actirq, ena;

    if(s->linkup) {
        s->evrreg[0x00>>2] &= ~0x00010000; /* clear link violation */
    } else {
        s->evrreg[0x00>>2] |= 0x00010000; /* set link violation */
        rawirq |= 0x01;
    }
    if(s->linkup != s->linkupprev) {
        rawirq |= 0x40;
        DBGOUT(1,"Link state change %u", s->linkup);
    }
    s->linkupprev = s->linkup;

    if(s->fifo_in!=s->fifo_out)
        rawirq |= 0x08; /* FIFO not empty */

    rawirq = s->evrreg[0x08>>2] = s->evrreg[0x08>>2]|rawirq; /* include already active, unack'd */

    ena = s->evrreg[0x0C>>2];
    actirq = rawirq & ena; /* Mask disabled */

    {
        int level = s->pciint && (ena&0x80000000) && actirq;
        DBGOUT(2,"IRQ %d Flag %08x Act %08x Ena %08x", level,
               (unsigned)rawirq, (unsigned)actirq, (unsigned)ena);
        qemu_set_irq(s->irq, !!level);
    }
}

static void
evr_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    EVRState *s = opaque;
    if(s->evrbe)
        val = bswap32(val);

    if(addr>=0x8200 && addr<0x8400)
        return; /* SFP info, RO */

    if(addr<0x100) {
        DBGOUT(2,"write "TARGET_FMT_plx" = %08"PRIx64
               " from %08"PRIx32,
               addr, val, s->evrreg[addr>>2]);
    }

    switch(addr) {
    /* R/O registers */
    case 0x00: /* Status */
    case 0x2C: /* Version */
    case 0x50:
    case 0x70:
    case 0x74:
    case 0x78:
        return;
    case 0x04: /* Control */
        if(val&0x08) {
            /* FIFO reset */
            s->fifo_in = s->fifo_out = 0;
        }
        s->masterena = !!(val&0x80000000);
        if(s->bridgetype!=0) {
            s->evrbe = !(val&0x02000000);
        } else {
            val &= ~0x02000000;
        }
        val &= 0xff8067f8;
        break;
    /* masked registers */
    case 0x08: /* IRQFlag */
        val &= 0x7f;
        /* clear these bits if set */
        val = s->evrreg[addr>>2] & ~(s->evrreg[addr>>2]&val);
        break;
    case 0x0C: /* IRQEnable */
        if(s->bridgetype==1 && s->fwversion<0xa) {
            s->pciint = !!(val&0x40000000);
        } else if(val&0x40000000) {
            DBGOUT(1, "Setting PCI MIE bit has no effect in IRQEnable");
            val &= ~0x40000000;
        }
        val &= 0xc000007f;
        break;
    case 0x1C: /* PCI Master Interrupt Enable */
        if(s->bridgetype==1 && s->fwversion>=0xa) {
            s->pciint = !!(val&0x40000000);
        } else {
            ERR(LOG_GUEST_ERROR, "PCI MIE not available with FW version 0x%06x", s->fwversion);
        }
        break;
    case 0x20: /* buffer rx */
        val &= 0x0000d000;
        if ((val&0xc000) == 0xc000) {
            ERR(LOG_GUEST_ERROR, "Dbuf RX can't enable+disable");
        }
        if(val&0x00004000) { // Stop
            val &= 0x1000;
            s->dbuf_ptr = 0;
        }
        val |= s->evrreg[addr>>2]&~0x0000f000;
        break;
    }

    s->evrreg[addr>>2] = val;

    switch(addr) {
    case 0x04: /* Control */
    case 0x08: /* IRQFlag */
    case 0x0C: /* IRQEnable */
    case 0x1C: /* PCI MIE */
        mrf_evr_update(s);
        break;
    }

    if(addr<0x100) {
        DBGOUT(2,"  to   %08x", (unsigned)s->evrreg[addr>>2]);
    }
}

static uint64_t
evr_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    EVRState *s = opaque;
    uint32_t ret;

    /* reads with side-effects */
    switch(addr) {
    case 0x78:
        /* Reading event code reg. pops FIFO entry */
        if(s->fifo_in==s->fifo_out) {
            /* Empty */
            DBGOUT(2,"FIFO empty");
            s->evrreg[0x70>>2] = 0xfffffffe;
            s->evrreg[0x74>>2] = 0xfffffffd;
            s->evrreg[0x78>>2] = 0;
        } else {
            EVRFifoEntry *entry = &s->fifo[s->fifo_out];
            s->fifo_out = (s->fifo_out+1)%NELEM(s->fifo);
            s->evrreg[0x70>>2] = entry->sec;
            s->evrreg[0x74>>2] = entry->count;
            s->evrreg[0x78>>2] = entry->code;
            DBGOUT(2,"FIFO dequeue %u %u %u", s->fifo_in, s->fifo_out, entry->code);
        }
    }

    ret = s->evrreg[addr>>2];

    DBGOUT(3, TARGET_FMT_plx" -> %08"PRIx32, addr, ret);

    if(s->evrbe)
        ret = bswap32(ret);
    return ret;
}

static const MemoryRegionOps evr_mmio_ops = {
    .read = evr_mmio_read,
    .write = evr_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static
int chr_link_can_read(void *opaque)
{
    EVRState *d = opaque;
    return sizeof(d->rxbuf)-d->rxbuflen;
}

static
void chr_link_read(void *opaque, const uint8_t *buf, int size)
{
    EVRState *d = opaque;
    uint8_t *pbuf = d->rxbuf;

    DBGOUT(1, "In %s %p %d", __func__, buf, size);

    memcpy(&pbuf[d->rxbuflen], buf, size);
    d->rxbuflen += size;

    for(;d->rxbuflen>=4;d->rxbuflen-=4, pbuf+=4)
    {
        uint8_t id = pbuf[0];
        if(id!=0xe1) {
            DBGOUT(1,"Framing error %u", id);
            continue;
        }
        d->linkup = pbuf[1]&0x01;
        if(d->linkup) {
            unsigned fifo_next = (d->fifo_in+1)%NELEM(d->fifo);
            uint8_t event = pbuf[2];
            uint32_t fifosave = d->evrreg[(0x4000+0x10*event+0)>>2]&0x80000000;

            if(pbuf[1]&0x02)
                DBGOUT(2,"Rx Event %02x DBus %02x", event, pbuf[3]);
            else {
                DBGOUT(2,"Rx Event %02x Data %02x", event, pbuf[3]);
                uint32_t rxctrl = d->evrreg[0x20>>2] & 0xd000;
                //uint8_t *buf = (uint8_t*)&d->evrreg[(0x800>>2)];

                if (rxctrl == 0x9000) {
                    // enabled and receiving

                    if(pbuf[1]&0x4) {
                        DBGOUT(2," Start");
                        /* first byte */
                        d->dbuf_ptr = 0;
                    }

                    if(d->dbuf_ptr<0x800) {
                        unsigned shift = ((d->dbuf_ptr^3)&3)*8;
                        d->evrreg[(0x800 + d->dbuf_ptr)>>2] &= ~(0xff<<shift);
                        d->evrreg[(0x800 + d->dbuf_ptr)>>2] |= ((uint32_t)pbuf[3])<<shift;
                        d->dbuf_ptr++;
                    } else {
                        ERR(LOG_UNIMP, "Dbuf EVR msg too long\n");
                    }

                    if(pbuf[1]&0x08) {
                        DBGOUT(2," End");
                        /* last byte */
                        /* Mark ready, and store size */
                        d->evrreg[0x20>>2] = 0x00005000 | d->dbuf_ptr;
                        d->evrreg[0x08>>2] |=  0x00000020;
                        mrf_evr_update(d);
                    }
                }
            }
            /* TODO: check for Data when not in mux'd mode */

            if(!fifosave || !d->masterena || d->evrreg[0x04>>2]&0x08) {
                /* not mapped, not enabled, or FIFO in reset.  ignore */

            } else if(fifo_next==d->fifo_out) {
                d->evrreg[0x08>>2] |= 0x02; /* FIFO overflow */
                DBGOUT(2,"FIFO overflow");

            } else {
                EVRFifoEntry *ent = &d->fifo[d->fifo_in];
                ent->code = event;
                ent->sec = ent->count = 0; /* TODO */
                d->fifo_in = fifo_next;
                DBGOUT(2,"FIFO enqueue %u %u %u", d->fifo_in, d->fifo_out, event);
            }
        }
    }

    mrf_evr_update(d);

    if(d->rxbuflen && d->rxbuf!=pbuf)
        memmove(d->rxbuf, pbuf, d->rxbuflen);
}

static
void chr_link_event(void *opaque, int event)
{
    EVRState *d = opaque;
    switch(event) {
    case CHR_EVENT_OPENED:
        DBGOUT(1,"linkevent => opened");
        d->linkup = 0;
        mrf_evr_update(d);
        break;
    case CHR_EVENT_CLOSED:
        DBGOUT(1,"linkevent => closed");
        d->linkup = 0;
        mrf_evr_update(d);
        break;
    default:
        break;
    }
}

static
uint32_t sfp_rom[512>>2];

static void mrf_evr_reset(DeviceState *dev)
{
    unsigned i;
    EVRState *s = MRF_EVR(dev);

    DBGOUT(1,"In %s", __FUNCTION__);

    memset(s->evrreg, 0, sizeof(s->evrreg));

    s->evrreg[0x00>>2] = 0x00010000; /* link violation */
    s->evrreg[0x50>>2] = 0x00000200; /* always show PLL locked */
    if(s->bridgetype != 0) {
        s->evrbe = 1;
    }
    if(s->bridgetype != 1) {
        s->pciint = 1;
    }

    s->evrreg[0x2C>>2] = 0x10000000 | ((s->mrftype&0xf)<<24) | s->fwversion;

    /* TODO, mapping ram defaults */

    for(i=0; i<NELEM(sfp_rom); i++) {
        s->evrreg[(0x8200>>2)+i] = sfp_rom[i];
    }
}

static
bool evr_get_endian(Object *obj, Error **err)
{
    EVRState *d = MRF_EVR(obj);
    return d->evrbe;
}

static
void evr_set_endian(Object *obj, bool val, Error **err)
{
    EVRState *d = MRF_EVR(obj);
    DBGOUT(2,"Set endian %u", (unsigned)val);
    d->evrbe = !!val;
}

static
void mrf_evr_init(Object *obj)
{
    EVRState *d = MRF_EVR(obj);
    MRFCore *c = MRF_CORE(obj);

    DBGOUT(1,"In %s", __FUNCTION__);

    memory_region_init_io(&c->core, obj, &evr_mmio_ops, d,
                          "mrf-evr", sizeof(d->evrreg));

    qdev_init_gpio_out(DEVICE(obj), &d->irq, 1);

    object_property_add_bool(OBJECT(obj), "endian", &evr_get_endian, &evr_set_endian, &error_abort);
}

static
void mrf_evr_realize(DeviceState *dev, Error **errp)
{
    EVRState *d = MRF_EVR(dev);

    /* can recursively call chr_link_event */
    qemu_chr_fe_set_handlers(&d->chr, chr_link_can_read, chr_link_read, chr_link_event, NULL, d, NULL, true);
}

static Property mrfevrport_properties[] = {
    DEFINE_PROP_UINT8("bridge-type", EVRState, bridgetype, 0),
    DEFINE_PROP_UINT8("mrf-type", EVRState, mrftype, 1),
    DEFINE_PROP_UINT32("version", EVRState, fwversion, 6),
    DEFINE_PROP_CHR("chardev", EVRState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void mrf_evr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    DBGOUT(1,"In %s", __FUNCTION__);

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Micro Research EVR core";
    dc->realize = mrf_evr_realize;
    dc->reset = mrf_evr_reset;
    dc->props = mrfevrport_properties;
    dc->user_creatable = false;
}

static const TypeInfo mrf_evr_info = {
    .name          = TYPE_MRF_EVR,
    .parent        = TYPE_MRF_CORE,
    .instance_size = sizeof(EVRState),
    .instance_init = mrf_evr_init,
    .class_init    = mrf_evr_class_init,
};

static void mrf_evr_register_types(void)
{
    type_register_static(&mrf_evr_info);
}

type_init(mrf_evr_register_types)

/* Captured from live cPCI-EVR-300 w/ firmware version 7 */
static
uint32_t sfp_rom[512>>2] = {
    0x03040700, 0x00000020, 0x400c1501, 0x2b000000,
    0x0f070000, 0x41564147, 0x4f202020, 0x20202020,
    0x20202020, 0x0000176a, 0x41464252, 0x2d353752,
    0x3541505a, 0x20202020, 0x20202020, 0x0352009b,
    0x001a0000, 0x41383135, 0x31394555, 0x43572020,
    0x20202020, 0x31353035, 0x30362020, 0x68f00121,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x0000407e, 0x005c9002, 0x44010052, 0x00730061,
    0xae0048ed, 0x04ff0001, 0x02ec013e, 0x00000003,
    0x4002661b, 0xef42cd1d, 0xd7bc1867, 0x3feb32cf,
    0x69555776, 0x3f983b40, 0x918bc8ea, 0x3eb0df4d,
    0x7e548804, 0xbf000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x6400d800, 0x5500f600, 0x98586978, 0x8dcc7404,
    0x138803e8, 0x138803e8, 0x1b5801f4, 0x177003e8,
    0xffdc0000, 0x2af801ea, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x3f800000, 0x00000000, 0x01000000,
    0x01000000, 0x01000000, 0x01000000, 0x080000c2,
    0x1f9d8215, 0x0a5a0ed3, 0x0ef10000, 0x00001000,
    0x00000000, 0x00000000, 0x00000042, 0x55594100,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00c000c0, 0x00000000,
};
