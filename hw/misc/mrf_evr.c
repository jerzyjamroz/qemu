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
#include "sysemu/dma.h"
#include "qemu/iov.h"

#define MRF_DEBUG

#ifdef MRF_DEBUG
#define	DBGOUT(fmt, ...) fprintf(stderr, "mrf-evr: %s() " fmt, __func__, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#endif


#define TYPE_MRF_EVR_BASE "mrf-pmcevr-230"

#define MRF_EVR(obj) \
    OBJECT_CHECK(MRFPCIState, (obj), TYPE_MRF_EVR_BASE)

#define MRF_EVR_DEVICE_CLASS(klass) \
     OBJECT_CLASS_CHECK(MRFEVRBaseClass, (klass), TYPE_MRF_EVR_BASE)
#define MRF_EVR_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(MRFEVRBaseClass, (obj), TYPE_MRF_EVR_BASE)

typedef struct MRFPCIState {
    /*< private >*/
    PCIDevice parent_obj;

    MemoryRegion plx9030;
    MemoryRegion evr;

    unsigned int evrbe:1;
    unsigned int pciint:1;

    uint32_t evrreg[64*1024/4];
} MRFPCIState;

typedef struct MRFPCIBaseClass {
    PCIDeviceClass parent_class;
} MRFPCIBaseClass;

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
        break;
    case 0x54: /* GPIOC */
        break; /* ignore */
    default:
        DBGOUT("invalid write for "TARGET_FMT_plx" %08x\n", addr, (unsigned)val);
    }
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
    addr >>= 2;
    if(s->evrbe)
        val = __bswap_32(val);

    switch(addr) {
    /* R/O registers */
    case 0x00: /* Status */
    case 0x2C: /* Version */
        return;
    }

    s->evrreg[addr] = val;
}

static uint64_t
evr_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    MRFPCIState *s = opaque;
    uint32_t ret = s->evrreg[addr>>2];
    /* TODO reads with side-effects */
    if(s->evrbe)
        ret = __bswap_32(ret);
    return ret;
}

static const MemoryRegionOps evr_mmio_ops = {
    .read = evr_mmio_read,
    .write = evr_mmio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void mrf_evr_e1000_reset(DeviceState *dev)
{
    MRFPCIState *s = MRF_EVR(dev);

    s->evrreg[0x00] = 0x00010000; /* link violation */
    s->evrreg[0x2C] = 0x10000001; /* PMC EVR */
}

static int mrf_evr_init(PCIDevice *pci_dev)
{
    /*DeviceState *dev = DEVICE(pci_dev);*/
    MRFPCIState *d = MRF_EVR(pci_dev);
    /*PCIDeviceClass *pdc = PCI_DEVICE_GET_CLASS(pci_dev);*/

    pci_dev->config[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    memory_region_init_io(&d->plx9030, OBJECT(d), &plx9030_mmio_ops, d,
                          "plx9030-mmio", 128);
    memory_region_init_io(&d->evr, OBJECT(d), &evr_mmio_ops, d,
                          "mrf-evr-mmio", sizeof(d->evrreg));

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->plx9030);
    pci_register_bar(pci_dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &d->evr);

    return 0;
}

static void mrf_evr_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    /*MRFPCIBaseClass *e = MRF_EVR_DEVICE_CLASS(klass);*/

    k->vendor_id = 0x10b5; /* PLX */
    k->device_id = 0x9030;
    k->subsystem_vendor_id = 0x1a3e;
    k->subsystem_id = 0x11e6;
    k->revision = 1;
    k->init = &mrf_evr_init;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "Micro Research PCI EVR";
    dc->reset = mrf_evr_e1000_reset;
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
