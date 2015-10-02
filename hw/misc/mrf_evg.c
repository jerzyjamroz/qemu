/*
 * QEMU MRF PCI EVG emulation
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

#define	WRNOUT(fmt, ...) fprintf(stderr, "mrf-evg: %s() Warning " fmt, __func__, ## __VA_ARGS__)

#ifdef MRF_DEBUG
#define	DBGOUT(fmt, ...) fprintf(stderr, "mrf-evg: %s() " fmt, __func__, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#endif

#define NELEM(N) (sizeof(N)/sizeof((N)[0]))

#define TYPE_MRF_EVG_BASE "mrf-pcievg"

#define MRF_EVG(obj) \
    OBJECT_CHECK(EVGState, (obj), TYPE_MRF_EVG_BASE)

#define MRF_EVG_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(MRFEVGBaseClass, (klass), TYPE_MRF_EVG_BASE)
#define MRF_EVG_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(MRFEVGBaseClass, (obj), TYPE_MRF_EVG_BASE)

typedef struct {
    /*< private >*/
    PCIDevice parent_obj;

    CharDriverState *chr; /* event link */

    MemoryRegion plx9030;
    MemoryRegion evg;

    unsigned int extbridge:1; /* 1 - PLX 9030, 0 - integrated */
    /* plx 9030 */
    unsigned int evgbe:1;
    unsigned int pciint:1;
    /* evg */
    unsigned int masterena:1;
    unsigned int irqactive:1;
    uint8_t dbus;

    uint32_t evgreg[64*1024/4];
} EVGState;

#define TXBUFREM(D) (sizeof(d->txbuf) - d->txbuflen)

static
void mrf_evg_update(EVGState *s)
{
    uint32_t rawirq, actirq, ena;

    rawirq = s->evgreg[0x08>>2]; /* include already active, unack'd */

    ena = s->evgreg[0x0C>>2];
    actirq = rawirq & ena; /* Mask disabled */

    {
        int level = s->pciint && (ena&0x80000000) && actirq;
        DBGOUT("IRQ %d Flag %08x Act %08x Ena %08x\n", level,
               (unsigned)rawirq, (unsigned)actirq, (unsigned)ena);
        s->irqactive = !!level;
        pci_set_irq(&s->parent_obj, !!level);
    }
}

static
void link_send(EVGState *d, const uint8_t *buf, size_t blen)
{
    int ret;

    if(blen==0)
        return; /* success when noting to send */

    /* TODO non-blocking I/O or cothread? */
    ret = qemu_chr_fe_write_all(d->chr, buf, blen);
    if(ret==-1) {
        WRNOUT("link send fails %d\n", ret);
        return ;
    }
}

static
void evg_seq_play(EVGState *d, int seq)
{
    size_t i;
    size_t ctrl = (seq==0 ? 0x70   : 0x74)>>2;
    size_t base = (seq==0 ? 0x8000 : 0xC000)>>2;

    if((d->evgreg[ctrl]&0x02000000) || !(d->evgreg[ctrl]&0x01000000))
        return; /* already running or not enabled */

    DBGOUT("Play sequencer %d\n", seq);

    d->evgreg[ctrl] |=  0x02000000; /* Running */
    d->evgreg[0x08>>2] |= 1<<(8+seq); /* indicate start of seq */
    mrf_evg_update(d);

    for(i=base; i<base+2048*2; i+=2)
    {
        uint8_t buf[4];
        uint32_t evt = d->evgreg[i+1]; /* event code is in odd numbered reg. */

        evt&=0xff;
        if(evt==0x7f)
            break;

        buf[0] = 0xe1;
        buf[1] = 0x03; /* payload is dbus, link active */
        buf[2] = evt;
        buf[3] = d->dbus;
        link_send(d, buf, 4);
    }

    d->evgreg[ctrl] &=  ~0x02000000; /* stopped */
    if(d->evgreg[ctrl]&0x00100000) /* single shot mode */
        d->evgreg[ctrl] &=  ~0x01000000; /* disable */
    d->evgreg[0x08>>2] |= 1<<(12+seq); /* indicate end of seq */
    mrf_evg_update(d);

    DBGOUT("Done sequencer %d\n", seq);
}

static
void evg_dbuf_send(EVGState *d)
{
    uint8_t buf[4];
    uint32_t ctrl = d->evgreg[0x20>>2];
    unsigned i, N = (ctrl>>2)&0x1ff;

    d->evgreg[0x20>>2] &= ~0x00040000;

    // Mode shared and TX enabled
    if((ctrl&0x00030000)!=0x00030000) {
        DBGOUT("DBuf can't send when not enabled 0x%08x\n",(unsigned)ctrl);
        return;
    }

    DBGOUT("DBuf send %u bytes\n", 4*N);

    d->evgreg[0x20>>2] |= 0x00080000; /* running */

    buf[0] = 0xe1;
    buf[1] = 0x05; /* payload is buf, first byte, link active */
    buf[2] = 0; /* event code */

    for(i=0; i<N; i++)
    {
        uint32_t raw = d->evgreg[(0x800>>2)+i];
        unsigned j;

        for(j=0; j<4; j++, raw<<=8)
        {
            buf[3] = (raw>>24)&0xff;
            if(i==N-1 && j==3)
                buf[1] |= 0x08; /* last byte */
            link_send(d, buf, 4);
            buf[1] &= ~0x04; /* not first byte */
        }
    }

    d->evgreg[0x20>>2] &= ~0x00080000;
    d->evgreg[0x20>>2] |=  0x00100000; /* complete */
}

static void
plx9030_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    EVGState *s = opaque;
    switch(addr) {
    case 0x28: /* LAS0BRD */
        s->evgbe = !!(val&0x01000000);
        break;
    case 0x4c: /* INTCSR */
        s->pciint = !!(val&0x40);
        mrf_evg_update(s);
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
    EVGState *s = opaque;
    switch(addr) {
    case 0x28: /* LAS0BRD */
        return s->evgbe ? 0x01000000 : 0;
    case 0x4c: /* INTCSR */
    {
        uint64_t ret = 0x03; /* INT1 enable, active high */
        if(s->pciint) ret |= 0x40; /* PCI interrupt enabled */
        if(s->irqactive) ret |= 0x04; /* INT1 active */
        return ret;
    }
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
evg_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    EVGState *s = opaque;
    if(s->evgbe)
        val = __bswap_32(val);

    if(addr>=0x1200 && addr<0x1400)
        return; /* SFP info, RO */

    DBGOUT("write "TARGET_FMT_plx" = %08x\n", addr, (unsigned)val);
    DBGOUT("  from %08x\n", (unsigned)s->evgreg[addr>>2]);

    switch(addr) {
    /* R/O registers */
    case 0x00: /* Status */
    case 0x2C: /* Version */
        return;
    case 0x04: /* Control */
        s->masterena = !!(val&0x80000000);
        if(!s->extbridge) {
            s->evgbe = !(val&0x02000000);
        }
        val &= 0xf7000000;
        break;
    /* masked registers */
    case 0x08: /* IRQFlag */
        val &= 0x00003363;
        /* clear these bits if set */
        val = s->evgreg[addr>>2] & ~(s->evgreg[addr>>2]&val);
        break;
    case 0x0C: /* IRQEnable */
        if(!s->extbridge) {
            s->pciint = !!(val&0x40000000);
        } else {
            val &= ~0x40000000;
        }
        val &= 0xc0003363;
        break;
    case 0x18: /* Software event */
        val &= 0x1ff; /* SW can't set pending bit */
        val |= s->evgreg[addr>>2]&0x200;
        break;
    case 0x20: /* buffer TX */
        val &= 0x000707fc;
        val |= s->evgreg[addr>>2]&0x00180000;
        break;
    case 0x70: /* sequencer control */
    case 0x74:
        val &= 0x00ff00ff;
        val |= s->evgreg[addr>>2]&0x03000000; /* persist Enable and Running */
        break;
    }

    s->evgreg[addr>>2] = val;

    switch(addr) {
    case 0x04: /* Control */
    case 0x08: /* IRQFlag */
    case 0x0C: /* IRQEnable */
        mrf_evg_update(s);
        break;
    case 0x18: /* Software event */
        /* if enabled, not pending, and not event code zero */
        if((val&0x300)==0x100 && (val&0xff)!=0) {
            uint8_t buf[4];
            s->evgreg[addr>>2] |= 0x200; /* pending */

            buf[0] = 0xe1;
            buf[1] = 0x03; /* payload is dbus, link active */
            buf[2] = val&0xff; /* event */
            buf[3] = s->dbus;
            link_send(s, buf, 4);

            s->evgreg[addr>>2] &= ~0x200; /* not pending */
            DBGOUT("Sent Sw event %u\n", (unsigned)(val&0xff));
        } else
            WRNOUT("Sw event can't send while pending\n");
        break;
    case 0x20: /* buffer TX */
        if(val&0x00040000)
            evg_dbuf_send(s);
        break;
    case 0x70: /* sequencer control */
    case 0x74:
        if(val&0x00010000) { /* Enable */
            s->evgreg[addr>>2] |=  0x01000000;
            s->evgreg[addr>>2] &= ~0x00010000;
        }
        if(val&0x00060000) { /* Disable or reset */
            s->evgreg[addr>>2] &= ~0x03060000;
        }
        if(val&0x00200000) { /* SW trigger */
            int seq = !!(addr&0x4); /* is this write for sequencer 0 or 1? */
            s->evgreg[addr>>2] &= ~0x00200000; /* SW trig. is write only */

            /* run sequencer(s) which are set to soft trig. on this command */
            if((s->evgreg[0x70>>2]&0xff)==17+seq)
                evg_seq_play(s, 0);
            if((s->evgreg[0x74>>2]&0xff)==17+seq)
                evg_seq_play(s, 1);
        }
        break;
    }

    DBGOUT("  to   %08x\n", (unsigned)s->evgreg[addr>>2]);
}

static uint64_t
evg_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    EVGState *s = opaque;
    uint32_t ret;

    /* reads with side-effects */
    /*switch(addr) {
    }*/

    ret = s->evgreg[addr>>2];

    if(s->evgbe)
        ret = __bswap_32(ret);
    return ret;
}

static const MemoryRegionOps evg_mmio_ops = {
    .read = evg_mmio_read,
    .write = evg_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static
int chr_link_can_read(void *opaque)
{
    (void)opaque;
    return 0;
}

static
void chr_link_read(void *opaque, const uint8_t *buf, int size)
{
    (void)opaque;
    (void)buf;
    (void)size;
}

static
void chr_link_event(void *opaque, int event)
{
    EVGState *d = opaque;
    (void)d;
    switch(event) {
    case CHR_EVENT_OPENED:
        DBGOUT("linkevent => opened\n");
    {
        uint8_t buf[4];
        buf[0] = 0xe1;
        buf[1] = 0x03; /* payload is dbus, link active */
        buf[2] = 0; /* idle event */
        buf[3] = d->dbus;
        link_send(d, buf, 4);
    }
        break;
    case CHR_EVENT_CLOSED:
        DBGOUT("linkevent => closed\n");
        break;
    default:
        break;
    }
}

static int mrf_evg_init_230(PCIDevice *pci_dev)
{
    EVGState *d = MRF_EVG(pci_dev);
    d->extbridge = 1;

    pci_dev->config[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    memory_region_init_io(&d->plx9030, OBJECT(d), &plx9030_mmio_ops, d,
                          "plx-ctrl", 128);
    memory_region_init_io(&d->evg, OBJECT(d), &evg_mmio_ops, d,
                          "mrf-evg", sizeof(d->evgreg));

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->plx9030);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->evg);

    if(d->chr) {
        qemu_chr_add_handlers(d->chr, chr_link_can_read, chr_link_read, chr_link_event, d);
    }

    return 0;
}

static int mrf_evg_init_300(PCIDevice *pci_dev)
{
    EVGState *d = MRF_EVG(pci_dev);

    pci_dev->config[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    memory_region_init_io(&d->evg, OBJECT(d), &evg_mmio_ops, d,
                          "mrf-evg", sizeof(d->evgreg));

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->evg);

    if(d->chr) {
        qemu_chr_add_handlers(d->chr, chr_link_can_read, chr_link_read, chr_link_event, d);
    }

    return 0;
}

static void mrf_evg_reset(DeviceState *dev)
{
    EVGState *s = MRF_EVG(dev);

    s->evgreg[0x04>>2] = 0x60000000;
    if(!s->extbridge) {
        s->evgbe = 1;
        s->evgreg[0x2C>>2] = 0x24000007; /* cPCI EVG w/ FW version 7 */
    } else {
        s->evgreg[0x2C>>2] = 0x27000007; /* PCIe EVG w/ FW version 7 */
    }

    /* TODO, SFP info */
}

static Property mrfevgport_properties[] = {
    DEFINE_PROP_CHR("chardev", EVGState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void mrf_evg_class_init_230(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->vendor_id = 0x10b5; /* PLX */
    k->device_id = 0x9030;
    k->subsystem_vendor_id = 0x1a3e;
    k->subsystem_id = 0x20E6;
    k->class_id = 0x1180;
    /* TODO capabilities? */
    k->revision = 2;
    k->init = &mrf_evg_init_230;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Micro Research cPCI-EVG-230";
    dc->reset = mrf_evg_reset;
    dc->props = mrfevgport_properties;
}

static void mrf_evg_class_init_300(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->vendor_id = 0x1a3e; /* MRF */
    k->device_id = 0x252c;
    k->subsystem_vendor_id = 0x1a3e;
    k->subsystem_id = 0x252c;
    k->class_id = 0x1180;
    /* TODO capabilities? */
    k->revision = 1;
    k->init = &mrf_evg_init_300;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Micro Research cPCI-EVG-300";
    dc->reset = mrf_evg_reset;
    dc->props = mrfevgport_properties;
}

static const TypeInfo mrf_evg_base_info = {
    .name          = TYPE_MRF_EVG_BASE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(EVGState),
    .class_size    = sizeof(PCIDeviceClass),
    .abstract      = true,
};

static const TypeInfo mrf_evg_230_info = {
    .name          = "mrf-cpci-evg-230",
    .parent        = TYPE_MRF_EVG_BASE,
    .class_init    = mrf_evg_class_init_230,
};

static const TypeInfo mrf_evg_300_info = {
    .name          = "mrf-cpci-evg-300",
    .parent        = TYPE_MRF_EVG_BASE,
    .class_init    = mrf_evg_class_init_300,
};

static void mrf_evg_register_types(void)
{
    type_register_static(&mrf_evg_base_info);
    type_register_static(&mrf_evg_230_info);
    type_register_static(&mrf_evg_300_info);
}

type_init(mrf_evg_register_types)
