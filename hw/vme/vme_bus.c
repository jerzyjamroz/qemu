/*
 * VME Bus infrastructure
 *
 * Copyright (c) 2015 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/qdev.h"
#include "hw/vme/vme_bus.h"

/* #define DEBUG_VMEBUS */
/* #define DEBUG_UNASSIGNED */

#ifdef DEBUG_VMEBUS
#define DBGOUT(fmt, ...) fprintf(stderr, TYPE_VME_BUS ": " fmt, \
                                 ## __VA_ARGS__)
#else
#define DBGOUT(fmt, ...) do {} while (0)
#endif

#define LOG(mask, fmt, ...) qemu_log_mask(mask, fmt, ##__VA_ARGS__)

/* Initialize a VME Bus in place
 *
 * Creates 7 GPIOs on the parent named "vme-irq#".
 */
void vme_bus_init(VMEBus *pbus, size_t bsize, DeviceState *parent)
{
    qbus_create_inplace(pbus, bsize, TYPE_VME_BUS, parent, "vme-bus");

    /* Attached to parent device (assumed to be bridge dev)
     * since a Bus can't have gpio directly.
     */
    qdev_init_gpio_out_named(parent, pbus->irq, "vme-irq", NUM_VME_IRQ);
}

static
uint64_t berr_read(void *opaque, hwaddr addr, unsigned size)
{
    VMEAddressSpace *space = opaque;
    VMEBus *bus = space->bus;

#ifdef DEBUG_UNASSIGNED
    printf(TYPE_VME_BUS ": BERR AM=0x%02x ADDR=0x%08x read%u\n",
           space->amod, (unsigned)addr, size);
#endif

    bus->berr.amod = space->amod;
    bus->berr.addr = addr;
    bus->berr.write = 0;
    bus->berr.size = size;
    if (bus->berr.cb) {
        (*bus->berr.cb)(bus, bus->berr.cb_arg);
    }

    /* VME data lines float high, so the value read from an
     * unassigned address is well defined.
     */
    return (uint64_t)-1;
}

static
void berr_write(void *opaque, hwaddr addr, uint64_t data, unsigned size)
{
    VMEAddressSpace *space = opaque;
    VMEBus *bus = space->bus;

#ifdef DEBUG_UNASSIGNED
    printf(TYPE_VME_BUS ": AM=0x%02x BERR 0x%08x write%u %08x\n",
           space->amod, (unsigned)addr, size, (unsigned)data);
#endif

    bus->berr.amod = space->amod;
    bus->berr.addr = addr;
    bus->berr.write = 1;
    bus->berr.size = size;
    if (bus->berr.cb) {
        (*bus->berr.cb)(bus, bus->berr.cb_arg);
    }
}

static const MemoryRegionOps berr_ops = {
    .read = berr_read,
    .write = berr_write
};

static
VMEAddressSpace *vme_bus_get_vspace(VMEBus *pbus, uint8_t amod)
{
    /* here lives weird rules for mapping address modifiers
     * to address spaces.
     */
    switch (amod & VME_AM_MSK_OPER) {
    case VME_AM_MSK_A24:
    case VME_AM_MSK_A32:
    /*case VME_AM_MSK_A64:*/
        /* The 2 LSBs select the transfer type (single cycle, block, ...) */
        amod &= VME_AM_MSK_SPACE;
        break;
    default:
        break;
    }
    return pbus->spaces[amod];
}

MemoryRegion *vme_bus_get_region(VMEBus *pbus, uint8_t amod)
{
    VMEAddressSpace *space = vme_bus_get_vspace(pbus, amod);
    return space ? &space->root : NULL;
}

AddressSpace *vme_bus_get_space(VMEBus *pbus, uint8_t amod)
{
    VMEAddressSpace *space = vme_bus_get_vspace(pbus, amod);
    return space ? &space->space : NULL;
}

/* Begin VME IRQ sequence by scanning slaves asserting a level.
 * Returns true on success and stores the vector code in pvect.
 * Returns false if no slave admits to signaling this level,
 * which shouldn't happen in simulation.
 * If it does, the caller (VME bridge) should signal a bus error.
 */
bool vme_bus_get_iack(VMEBus *pbus, uint8_t level, uint32_t *pvect)
{
    bool ret = false;
    unsigned i;
    uint8_t mask;

    assert(level >= 1 && level <= 7);
    level--;
    mask = 1 << level;

    /* IACK daisy chain processing */
    for (i = 1; i <= NUM_VME; i++) {
        VMEDevice *dev = pbus->devices[i];
        /* Slight fudge here.
         * Only call iack() for devices known
         * to assert an IRQ.
         */
        if (!dev || !(dev->irq_status & mask)) {
            continue;
        }
        DBGOUT("IACK poll slot %u\n", i);
        if ((dev->iack(dev, level, pvect))) {
            ret = true;
            break;
        }
    }

    DBGOUT("IACK%u -> 0x%x\n", level, (unsigned)*pvect);
    return ret;
}

/* Called by a slave card to (de)assert an IRQ level. */
void vme_set_irq(VMEDevice *dev, unsigned irqlevel, unsigned val)
{
    VMEBus *bus = dev->bus;
    unsigned mask;
    assert(irqlevel > 0 && irqlevel <= NUM_VME_IRQ);
    irqlevel -= 1;
    mask = 1 << irqlevel;

    if (!dev->iack) {
        fprintf(stderr, "VMEDevice in slot %u must provide iack() to be"
                        " able to assert IRQs\n", dev->slot);
        return;
    }

    if (val) {
        dev->irq_status |= mask;
    } else {
        unsigned i;

        /* this card no longer asserts */
        dev->irq_status &= ~mask;

        /* see if another card still does */
        for (i = 1; i <= NUM_VME; i++) {
            if (bus->devices[i]) {
                val |= !!(bus->devices[i]->irq_status & mask);
            }
        }
    }

    DBGOUT("%s SLOT=%u LVL=%u %c%c\n", __func__, dev->slot, irqlevel + 1,
           orig ? 'X' : '_', val ? 'X' : '_');

    qemu_set_irq(bus->irq[irqlevel], val);
}

void vme_add_region(VMEDevice *dev, uint8_t amod, hwaddr base,
                    MemoryRegion *reg)
{
    VMEBus *bus = dev->bus;
    MemoryRegion *root = vme_bus_get_region(bus, amod);

    if (root) {
        memory_region_add_subregion(root, base, reg);
    } else {
        LOG(LOG_UNIMP, "Address modifier %08x not implemented by VME Bus\n", amod);
    }
}

void vme_del_region(VMEDevice *dev, uint8_t amod, MemoryRegion *reg)
{
    VMEBus *bus = dev->bus;
    MemoryRegion *root = vme_bus_get_region(bus, amod);

    if (root) {
        memory_region_del_subregion(root, reg);
    }
}

typedef struct {
    const char *name;
    uint8_t amod;
    size_t size;
} space_map;

static const
space_map vmaps[] = {
    {
        .name = "vme-a16-sup",
        .amod = VME_AM_A16_SUP,
        .size = 0x10000,
    },
    {
        .name = "vme-a16-usr",
        .amod = VME_AM_A16_USR,
        .size = 0x10000,
    },
    {
        .name = "vme-a24-sup",
        .amod = VME_AM_A24_SUP_MBLT,
        .size = 0x1000000,
    },
    {
        .name = "vme-a24-usr",
        .amod = VME_AM_A24_USR_MBLT,
        .size = 0x1000000,
    },
    {
        .name = "vme-a32-sup",
        .amod = VME_AM_A32_SUP_MBLT,
        .size = 0x100000000,
    },
    {
        .name = "vme-a32-usr",
        .amod = VME_AM_A32_USR_MBLT,
        .size = 0x100000000,
    },
    {
        .name = "vme-cr-csr",
        .amod = VME_AM_CRCSR,
        .size = 0x1000000,
    },
    {.name = NULL}
};

static
void vme_bus_unrealize(BusState *bus, Error **errp)
{
    VMEBus *vme = VME_BUS(bus);
    unsigned i;

    for (i = 0; i < 256; i++) {
        if (vme->spaces[i]) {
            address_space_destroy(&vme->spaces[i]->space);
            g_free(vme->spaces[i]);
        }
    }
}

static
void vme_bus_inst_init(Object *obj)
{
    unsigned i;
    const space_map *cur;
    VMEBus *vme = VME_BUS(obj);

    for (i = 0, cur = vmaps; cur->name; i++, cur++) {
        VMEAddressSpace *space;

        space = vme->spaces[cur->amod] = g_malloc0(sizeof(*space));
        space->bus = vme;
        space->amod = cur->amod;
        memory_region_init_io(&space->root, obj, &berr_ops, space,
                              cur->name, cur->size);
        address_space_init(&space->space, &space->root, cur->name);
    }
}

static
void vme_bus_class_init(ObjectClass *klass, void *data)
{
    BusClass *bc = BUS_CLASS(klass);

    bc->unrealize = &vme_bus_unrealize;
}

static const TypeInfo vme_bus_type_info = {
    .name = TYPE_VME_BUS,
    .parent = TYPE_BUS,
    .class_size = sizeof(VMEBusClass),
    .instance_size = sizeof(VMEBus),
    .class_init = vme_bus_class_init,
    .instance_init = vme_bus_inst_init,
};

static
void vme_device_realize(DeviceState *dev, Error **errp)
{
    VMEDevice *vme = VME(dev);
    VMEDeviceClass *dc = VME_GET_CLASS(dev);

    assert(dev->parent_bus);
    vme->bus = VME_BUS(dev->parent_bus);
    assert(vme->bus);

    if (vme->slot > 0) {
        /* manually specified slot # */
        if (vme->slot > NUM_VME) {
            error_setg(errp, "Slot %u is out of range 1-21", vme->slot);
            /* TODO remove device */
            return;

        } else if (vme->bus->devices[vme->slot]) {
            error_setg(errp, "Slot %u already populated", vme->slot);
            return;
        }

    } else { /* find an unpopulated slot */
        unsigned i;

        for (i = 1; i <= NUM_VME; i++) {
            if (!vme->bus->devices[i]) {
                break;
            }
        }
        if (i > NUM_VME) {
            error_setg(errp, "No space for device, bus full");
            return;
        }
        vme->slot = i;
    }

    vme->bus->devices[vme->slot] = vme;

    if (dc->realize) {
        Error *le = NULL;
        (*dc->realize)(vme, &le);
        if (le) {
            error_propagate(errp, le);
            vme->bus->devices[vme->slot] = NULL;
            return;
        }
    }
}

static
void vme_device_unrealize(DeviceState *dev, Error **errp)
{
    VMEDevice *vme = VME(dev);
    VMEDeviceClass *dc = VME_GET_CLASS(dev);

    if (dc->unrealize) {
        (*dc->unrealize)(vme, errp);
    }
}

static Property vme_props[] = {
    DEFINE_PROP_UINT8("slot", VMEDevice, slot, 0), /* default to auto-assign */
    DEFINE_PROP_END_OF_LIST(),
};

static
void vme_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);

    k->bus_type = TYPE_VME_BUS;
    k->realize = vme_device_realize;
    k->unrealize = vme_device_unrealize;
    k->props = vme_props;
}

static const TypeInfo vme_device_type_info = {
    .name = TYPE_VME,
    .parent = TYPE_DEVICE,
    .class_size = sizeof(VMEDeviceClass),
    .instance_size = sizeof(VMEDevice),
    .class_init = vme_device_class_init,
};

static
void vme_register_types(void)
{
    type_register_static(&vme_bus_type_info);
    type_register_static(&vme_device_type_info);
}

type_init(vme_register_types);
