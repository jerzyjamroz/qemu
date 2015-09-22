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

#define TYPE_MRF_EVG_BASE "mrf-pcievg-230"

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

    DBGOUT("write "TARGET_FMT_plx" = %08x\n", addr, (unsigned)val);

    if(addr>=0x1200 && addr<0x1400)
        return; /* SFP info */

    switch(addr) {
    /* R/O registers */
    case 0x00: /* Status */
    case 0x2C: /* Version */
        return;
    case 0x04: /* Control */
        s->masterena = !!(val&0x80000000);
        break;
    /* masked registers */
    case 0x08: /* IRQFlag */
        val &= 0x00003363;
        /* clear these bits if set */
        val = s->evgreg[addr>>2] & ~(s->evgreg[addr>>2]&val);
        break;
    case 0x0C: /* IRQEnable */
        val &= 0x80003363;
        break;
    case 0x18: /* Software event */
        val &= 0x1ff; /* SW can't set pending bit */
        val |= s->evgreg[addr>>2]&0x200;
        break;
    }

    s->evgreg[addr>>2] = val;

    switch(addr) {
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
    }
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

static int mrf_evg_init(PCIDevice *pci_dev)
{
    EVGState *d = MRF_EVG(pci_dev);

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

static void mrf_evg_reset(DeviceState *dev)
{
    EVGState *s = MRF_EVG(dev);

    s->evgreg[0x2C>>2] = 0x27000006; /* PCI EVG w/ FW version 6 */

    /* TODO, SFP info */
}

static Property mrfevgport_properties[] = {
    DEFINE_PROP_CHR("chardev", EVGState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void mrf_evg_class_init(ObjectClass *klass, void *data)
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
    k->init = &mrf_evg_init;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Micro Research PCI EVG";
    dc->reset = mrf_evg_reset;
    dc->props = mrfevgport_properties;
}

static const TypeInfo mrf_evg_base_info = {
    .name          = TYPE_MRF_EVG_BASE,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(EVGState),
    .class_size    = sizeof(PCIDeviceClass),
    .class_init    = mrf_evg_class_init,
};

static void mrf_evg_register_types(void)
{
    type_register_static(&mrf_evg_base_info);
}

type_init(mrf_evg_register_types)
