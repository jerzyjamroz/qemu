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
#include "hw/sysbus.h"
#include "hw/pci/pci.h"
#include "net/net.h"
#include "net/checksum.h"
#include "hw/loader.h"
#include "sysemu/sysemu.h"
#include "sysemu/char.h"

#define MRF_DEBUG

#define TYPE_MRF_PCI "mrf-pci"

#ifdef MRF_DEBUG
#define	DBGOUT(fmt, ...) fprintf(stderr, TYPE_MRF_PCI ": %s() " fmt, __func__, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#endif

#define ERR(mask, fmt, ...)  qemu_log_mask(mask, TYPE_MRF_PCI ": " fmt, ## __VA_ARGS__)

#define NELEM(N) (sizeof(N)/sizeof((N)[0]))

#define MRF_PCI(obj) OBJECT_CHECK(MRFPCIState, (obj), TYPE_MRF_PCI)
#define MRF_PCI_CLASS(klass) OBJECT_CLASS_CHECK(MRFPCIClass, klass, TYPE_MRF_PCI)
#define MRF_PCI_GET_CLASS(obj) OBJECT_GET_CLASS(MRFPCIClass, obj, TYPE_MRF_PCI)

typedef struct MRFPCIState {
    PCIDevice parent_obj;

    CharDriverState *chr; /* event link */

    MemoryRegion plx;

    SysBusDevice *core;

    uint8_t fwversion;

    /* plx 9030 */
    unsigned int irqena:1;
    unsigned int irqact:1;

} MRFPCIState;

typedef struct {
    const char *core;
    const char *desc;
    uint16_t vendor_id;
    uint16_t device_id;
    uint16_t class_id;
    uint16_t subsystem_vendor_id;
    uint16_t subsystem_id;
    uint8_t mrf_type;
} mrf_pci_info;

typedef struct MRFPCIClass {
    PCIDeviceClass parent_obj;

    const mrf_pci_info *info;
} MRFPCIClass;

static
void handle_evr_irq(void *opaque, int n, int level)
{
    MRFPCIState *s = opaque;

    s->irqact = !!level;
    pci_set_irq(&s->parent_obj, s->irqact && s->irqena);
}

static void
plx9030_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    MRFPCIState *s = opaque;
    switch(addr) {
    case 0x28: /* LAS0BRD */
        object_property_set_bool(OBJECT(s->core), !!(val&0x01000000), "endian", &error_abort);
        break;
    case 0x4c: /* INTCSR */
        s->irqena = !!(val&0x40);
        handle_evr_irq(opaque, 0, s->irqact);
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
        return object_property_get_bool(OBJECT(s->core), "endian", &error_abort) ? 0x01000000 : 0;
    case 0x4c: /* INTCSR */
    {
        uint64_t ret = 0x03; /* INT1 enable, active high */
        if(s->irqena) ret |= 0x40; /* PCI interrupt enabled */
        if(s->irqact) ret |= 0x04; /* INT1 active */
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

static int mrf_pci_init(PCIDevice *pci_dev)
{
    int mrfbar = 0;
    DeviceState *dev;
    MRFPCIState *d = MRF_PCI(pci_dev);
    MRFPCIClass *k = MRF_PCI_GET_CLASS(pci_dev);

    assert(k->info);

    pci_dev->config[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    dev = qdev_create(NULL, k->info->core);
    d->core = SYS_BUS_DEVICE(dev);

    object_property_add_child(OBJECT(pci_dev), "core", OBJECT(dev), &error_abort);

    if(d->chr) {
        qemu_chr_fe_release(d->chr);
        qdev_prop_set_chr(dev, "chardev", d->chr);
    }
    qdev_prop_set_uint8(dev, "mrf-type", k->info->mrf_type);
    qdev_prop_set_uint8(dev, "version", d->fwversion);

    if(k->info->vendor_id==0x10b5) { /* PLX */
        memory_region_init_io(&d->plx, OBJECT(d), &plx9030_mmio_ops, d,
                              "plx-ctrl", 128);
        pci_register_bar(pci_dev, 0,PCI_BASE_ADDRESS_SPACE_MEMORY, &d->plx);
        mrfbar = 2;
    } else {
        qdev_prop_set_bit(dev, "soft-bridge", 1);
    }

    qdev_init_nofail(dev);

    pci_register_bar(pci_dev, mrfbar, PCI_BASE_ADDRESS_SPACE_MEMORY,
                     sysbus_mmio_get_region(d->core, 0));

    if(k->info->vendor_id==0x10b5) { /* PLX */
        /* PLX bridge masks the raw IRQ */
        qdev_init_gpio_in_named(DEVICE(pci_dev), &handle_evr_irq, "RAWIRQ", 1);
        qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in_named(DEVICE(pci_dev), "RAWIRQ", 0));
    } else {
        /* pass through, masking done in EVR logic (soft-bridge=1) */
        qdev_connect_gpio_out(dev, 0, pci_allocate_irq(pci_dev));
    }
    return 0;
}

static Property mrfpci_properties[] = {
    DEFINE_PROP_UINT8("version", MRFPCIState, fwversion, 6),
    DEFINE_PROP_CHR("chardev", MRFPCIState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void mrf_pci_class_init(ObjectClass *klass, void *data)
{
    mrf_pci_info *info = data;
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);
    MRFPCIClass *mk = MRF_PCI_CLASS(klass);

    assert(info);

    mk->info = info;

    k->vendor_id = info->vendor_id;
    k->device_id = info->device_id;
    k->subsystem_vendor_id = info->subsystem_vendor_id;
    k->subsystem_id = info->subsystem_id;
    k->class_id = info->class_id;
    /* TODO capabilities? */
    k->revision = 2;
    k->init = &mrf_pci_init;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = info->desc;
    dc->props = mrfpci_properties;
}

static const TypeInfo mrf_pci_base_info = {
    .name          = TYPE_MRF_PCI,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(MRFPCIState),
    .class_size    = sizeof(MRFPCIClass),
    .abstract      = true,
};

static
mrf_pci_info pmc_evr_230_info = {
    .core = "mrf-evr",
    .desc = "Micro Research PMC-EVR-230",
    .vendor_id = 0x10b5, /* PLX */
    .device_id = 0x9030,
    .subsystem_vendor_id = 0x1a3e,
    .subsystem_id = 0x11e6,
    .class_id = 0x1180,
    .mrf_type = 0x11,
};

static const TypeInfo mrf_evr_230_info = {
    .name          = "pmc-evr-230",
    .parent        = TYPE_MRF_PCI,
    .class_init    = mrf_pci_class_init,
    .class_data    = &pmc_evr_230_info,
};

static
mrf_pci_info cpci_evr_300_info = {
    .core = "mrf-evr",
    .desc = "Micro Research cPCI-EVR-300",
    .vendor_id = 0x1a3e, /* MRF */
    .device_id = 0x152c,
    .subsystem_vendor_id = 0x1a3e,
    .subsystem_id = 0x152c,
    .class_id = 0x1180,
    .mrf_type = 0x14,
};

static const TypeInfo mrf_evr_300_info = {
    .name          = "cpci-evr-300",
    .parent        = TYPE_MRF_PCI,
    .class_init    = mrf_pci_class_init,
    .class_data    = &cpci_evr_300_info,
};

static
mrf_pci_info cpci_evg_230_info = {
    .core = "mrf-evg",
    .desc = "Micro Research cPCI-EVG-230",
    .vendor_id = 0x10b5, /* PLX */
    .device_id = 0x9030,
    .subsystem_vendor_id = 0x1a3e,
    .subsystem_id = 0x20E6,
    .class_id = 0x1180,
    .mrf_type = 0x24,
};

static const TypeInfo mrf_evg_230_info = {
    .name          = "cpci-evg-230",
    .parent        = TYPE_MRF_PCI,
    .class_init    = mrf_pci_class_init,
    .class_data    = &cpci_evg_230_info,
};

static
mrf_pci_info cpci_evg_300_info = {
    .core = "mrf-evg",
    .desc = "Micro Research cPCI-EVG-300",
    .vendor_id = 0x1a3e, /* MRF */
    .device_id = 0x252c,
    .subsystem_vendor_id = 0x1a3e,
    .subsystem_id = 0x252c,
    .class_id = 0x1180,
    .mrf_type = 0x24,
};

static const TypeInfo mrf_evg_300_info = {
    .name          = "cpci-evg-300",
    .parent        = TYPE_MRF_PCI,
    .class_init    = mrf_pci_class_init,
    .class_data    = &cpci_evg_300_info,
};

static void mrf_pci_register_types(void) {
    type_register_static(&mrf_pci_base_info);
    type_register_static(&mrf_evr_230_info);
    type_register_static(&mrf_evr_300_info);
    type_register_static(&mrf_evg_230_info);
    type_register_static(&mrf_evg_300_info);
}

type_init(mrf_pci_register_types)
