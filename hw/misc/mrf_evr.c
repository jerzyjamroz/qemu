/*
 * QEMU MRF PCI EVR emulation
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


#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "net/net.h"
#include "net/checksum.h"
#include "hw/loader.h"
#include "sysemu/sysemu.h"
#include "sysemu/char.h"
#include "qemu/iov.h"

#define MRF_DEBUG

#ifdef MRF_DEBUG
#define	DBGOUT(fmt, ...) fprintf(stderr, "mrf-evr: %s() " fmt, __func__, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#endif

#define NELEM(N) (sizeof(N)/sizeof((N)[0]))

#define TYPE_MRF_EVR_BASE "mrf-pmcevr-230"

#define MRF_EVR(obj) \
    OBJECT_CHECK(MRFPCIState, (obj), TYPE_MRF_EVR_BASE)

#define MRF_EVR_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(MRFEVRBaseClass, (klass), TYPE_MRF_EVR_BASE)
#define MRF_EVR_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(MRFEVRBaseClass, (obj), TYPE_MRF_EVR_BASE)

typedef struct {
    uint32_t sec, count;
    uint8_t code;
} EVRFifoEntry;

typedef struct MRFPCIState {
    /*< private >*/
    PCIDevice parent_obj;

    CharDriverState *chr; /* event link */

    MemoryRegion plx9030;
    MemoryRegion evr;

    /* plx 9030 */
    unsigned int evrbe:1;
    unsigned int pciint:1;
    /* evr */
    unsigned int masterena:1;
    unsigned int linkup:1;
    unsigned int linkupprev:1;

    EVRFifoEntry fifo[512];
    unsigned fifo_in, fifo_out; /* buffer pointers */

    uint32_t evrreg[64*1024/4];

    uint8_t rxbuf[4];
    unsigned rxbuflen;
} MRFPCIState;

typedef struct MRFPCIBaseClass {
    PCIDeviceClass parent_class;
} MRFPCIBaseClass;

static
void mrf_evr_update(MRFPCIState *s)
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
        DBGOUT("Link state change %u\n", s->linkup);
    }
    s->linkupprev = s->linkup;

    if(s->fifo_in!=s->fifo_out)
        rawirq |= 0x08; /* FIFO not empty */

    rawirq = s->evrreg[0x08>>2] = s->evrreg[0x08>>2]|rawirq; /* include already active, unack'd */

    ena = s->evrreg[0x0C>>2];
    actirq = rawirq & ena; /* Mask disabled */

    {
        int level = s->pciint && ena&0x80000000 && actirq;
        DBGOUT("IRQ %d Flag %08x Act %08x Ena %08x\n", level,
               (unsigned)rawirq, (unsigned)actirq, (unsigned)ena);
        pci_set_irq(&s->parent_obj, !!level);
    }
}

static void
plx9030_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    MRFPCIState *s = opaque;
    switch(addr) {
    case 0x28: /* LAS0BRD */
        s->evrbe = !!(val&0x01000000);
        break;
    case 0x4c: /* INTCSR */
        s->pciint = !!(val&0x40);
        mrf_evr_update(s);
        break;
    case 0x54: /* GPIOC */
        break; /* ignore */
    default:
        DBGOUT("invalid write for "TARGET_FMT_plx" %08x\n", addr, (unsigned)val);
    }
    DBGOUT("write "TARGET_FMT_plx" = %08x\n", addr, (unsigned)val);
}

static uint64_t
plx9030_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MRFPCIState *s = opaque;
    switch(addr) {
    case 0x28: /* LAS0BRD */
        return s->evrbe ? 0x01000000 : 0;
    case 0x4c: /* INTCSR */
        return s->pciint ? 0x43 : 0x03; /* INT1 enable, invert polarity, PCI enable/disable */
    case 0x54: /* GPIOC */
        return 0x00249924;
    default:
        DBGOUT("invalid read for "TARGET_FMT_plx"\n", addr);
        return 0;
    }
}

static const MemoryRegionOps plx9030_mmio_ops = {
    .read = plx9030_mmio_read,
    .write = plx9030_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void
evr_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    MRFPCIState *s = opaque;
    if(s->evrbe)
        val = __bswap_32(val);

    if(addr<0x100)
        DBGOUT("write "TARGET_FMT_plx" = %08x\n", addr, (unsigned)val);

    if(addr>0x8200 && addr<0x8300+512)
        return; /* SFP info */

    switch(addr) {
    /* R/O registers */
    case 0x00: /* Status */
    case 0x2C: /* Version */
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
        break;
    /* masked registers */
    case 0x08: /* IRQFlag */
        val &= 0x7f;
        /* clear these bits if set */
        val = s->evrreg[addr>>2] & ~(s->evrreg[addr>>2]&val);
        break;
    case 0x0C: /* IRQEnable */
        val &= 0x8000007f;
        break;
    }

    s->evrreg[addr>>2] = val;

    switch(addr) {
    case 0x04: /* Control */
    case 0x08: /* IRQFlag */
    case 0x0C: /* IRQEnable */
        mrf_evr_update(s);
        break;
    }
}

static uint64_t
evr_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MRFPCIState *s = opaque;
    uint32_t ret;

    /* reads with side-effects */
    switch(addr) {
    case 0x78:
        /* Reading event code reg. pops FIFO entry */
        if(s->fifo_in==s->fifo_out) {
            /* Empty */
            DBGOUT("FIFO empty\n");
            s->evrreg[0x70>>2] = 0xfffffffe;
            s->evrreg[0x74>>2] = 0xfffffffd;
            s->evrreg[0x78>>2] = 0;
        } else {
            EVRFifoEntry *entry = &s->fifo[s->fifo_out];
            s->fifo_out = (s->fifo_out+1)%NELEM(s->fifo);
            s->evrreg[0x70>>2] = entry->sec;
            s->evrreg[0x74>>2] = entry->count;
            s->evrreg[0x78>>2] = entry->code;
            DBGOUT("FIFO dequeue %u %u %u\n", s->fifo_in, s->fifo_out, entry->code);
        }
    }

    ret = s->evrreg[addr>>2];

    if(s->evrbe)
        ret = __bswap_32(ret);
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
    MRFPCIState *d = opaque;
    return sizeof(d->rxbuf)-d->rxbuflen;
}

static
void chr_link_read(void *opaque, const uint8_t *buf, int size)
{
    MRFPCIState *d = opaque;
    uint8_t *pbuf = d->rxbuf;

    memcpy(&pbuf[d->rxbuflen], buf, size);
    d->rxbuflen += size;

    for(;d->rxbuflen>=4;d->rxbuflen-=4, pbuf+=4)
    {
        uint8_t id = pbuf[0];
        if(id!=0xe1) {
            DBGOUT("Framing error %u\n", id);
            continue;
        }
        d->linkup = pbuf[1]&0x01;
        if(d->linkup) {
            unsigned fifo_next = (d->fifo_in+1)%NELEM(d->fifo);
            uint8_t event = pbuf[2];
            uint32_t fifosave = d->evrreg[(0x4000+0x10*event+0)>>2]&0x80000000;

            if(pbuf[1]&0x02)
                DBGOUT("Rx Event %02x DBus %02x\n", event, pbuf[3]);
            else
                DBGOUT("Rx Event %02x Data %02x\n", event, pbuf[3]);
            /* TODO: check for Data when not in mux'd mode */

            if(!fifosave || !d->masterena || d->evrreg[0x04>>2]&0x08) {
                /* not mapped, not enabled, or FIFO in reset.  ignore */

            } else if(fifo_next==d->fifo_out) {
                d->evrreg[0x08>>2] |= 0x02; /* FIFO overflow */
                DBGOUT("FIFO overflow\n");

            } else {
                EVRFifoEntry *ent = &d->fifo[d->fifo_in];
                ent->code = event;
                ent->sec = ent->count = 0; /* TODO */
                d->fifo_in = fifo_next;
                DBGOUT("FIFO enqueue %u %u %u\n", d->fifo_in, d->fifo_out, event);
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
    MRFPCIState *d = opaque;
    switch(event) {
    case CHR_EVENT_OPENED:
        DBGOUT("linkevent => opened\n");
        d->linkup = 0;
        mrf_evr_update(d);
        break;
    case CHR_EVENT_CLOSED:
        DBGOUT("linkevent => closed\n");
        d->linkup = 0;
        mrf_evr_update(d);
        break;
    default:
        break;
    }
}

static void mrf_evr_reset(DeviceState *dev)
{
    MRFPCIState *s = MRF_EVR(dev);

    s->evrreg[0x00>>2] = 0x00010000; /* link violation */
    s->evrreg[0x2C>>2] = 0x11000003; /* PMC EVR w/ FW version 3 */

    /* TODO, mapping ram defaults */
    /* TODO, SFP info */
}

static int mrf_evr_init(PCIDevice *pci_dev)
{
    /*DeviceState *dev = DEVICE(pci_dev);*/
    MRFPCIState *d = MRF_EVR(pci_dev);
    /*PCIDeviceClass *pdc = PCI_DEVICE_GET_CLASS(pci_dev);*/

    pci_dev->config[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    memory_region_init_io(&d->plx9030, OBJECT(d), &plx9030_mmio_ops, d,
                          "plx-ctrl", 128);
    memory_region_init_io(&d->evr, OBJECT(d), &evr_mmio_ops, d,
                          "mrf-evr", sizeof(d->evrreg));

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->plx9030);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->evr);

    if(d->chr) {
        qemu_chr_add_handlers(d->chr, chr_link_can_read, chr_link_read, chr_link_event, d);
    }

    return 0;
}

static Property mrfevrport_properties[] = {
    DEFINE_PROP_CHR("chardev", MRFPCIState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void mrf_evr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    /*MRFPCIBaseClass *e = MRF_EVR_DEVICE_CLASS(klass);*/

    k->vendor_id = 0x10b5; /* PLX */
    k->device_id = 0x9030;
    k->subsystem_vendor_id = 0x1a3e;
    k->subsystem_id = 0x11e6;
    k->class_id = 0x1180;
    /* TODO capabilities? */
    k->revision = 2;
    k->init = &mrf_evr_init;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Micro Research PCI EVR";
    dc->reset = mrf_evr_reset;
    dc->props = mrfevrport_properties;
}

static const TypeInfo mrf_evr_base_info = {
    .name          = TYPE_MRF_EVR_BASE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MRFPCIState),
    .class_size    = sizeof(MRFPCIBaseClass),
    .class_init    = mrf_evr_class_init,
};

static void mrf_evr_register_types(void)
{
    type_register_static(&mrf_evr_base_info);
}

type_init(mrf_evr_register_types)
