/*
 * MVME3100 board CPLD (local logic)
 *
 * Copyright (c) 2015 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 *
 * This model was developed according to the
 * MVME3100 Single Board Computer Programmer's Reference
 * P/N: 6806800G37B
 * July 2014
 *
 * And validated against the RTEMS 4.9.6 mvme3100 BSP
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "exec/address-spaces.h"
#include "qemu-common.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"

/* #define DEBUG_3100CPLD */

#define TYPE_CPLD "mvme3100-cpld"

#define CPLD(obj) OBJECT_CHECK(MVMECPLD, (obj), TYPE_CPLD)

#ifdef DEBUG_3100CPLD
#define DPRINTK(FMT, ...) printf(TYPE_CPLD " : " FMT, ## __VA_ARGS__)
#else
#define DPRINTK(FMT, ...) do {} while (0)
#endif

#define LOG(MSK, FMT, ...) qemu_log_mask(MSK, TYPE_CPLD " : " FMT, \
    ## __VA_ARGS__)

#define CPLD_SIZE 0x20

typedef struct {
    SysBusDevice parent_obj;

    uint8_t mem[0x10];
    uint32_t test;

    MemoryRegion mmio;
} MVMECPLD;

static
uint64_t cpld_read(void *opaque, hwaddr addr, unsigned size)
{
    MVMECPLD *self = opaque;
    uint32_t offset = addr;
    uint32_t val, A;

    switch (offset) {
    case 1 ... 0xf:
        val = 0;
        A = offset;
        while (size--) {
            val <<= 8;
            val |= self->mem[A++];
        }
        break;
    case 0x10:
        val = self->test;
        break;
    case 0x14:
        val = ~self->test;
        break;
    default:
        LOG(LOG_UNIMP, "read from unimplimented register %08x\n",
            (unsigned)offset);
    }

    DPRINTK("read %08x -> %08x\n", (unsigned)offset, (unsigned)val);

    return val;
}

static
void cpld_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    MVMECPLD *self = opaque;
    uint32_t offset = addr;

    DPRINTK("write %08x <- %08x\n", (unsigned)offset, (unsigned)val);

    switch (offset) {
    case 0:
        break;
    case 1:
        /* TODO: TSTAT_MASK and EEPROM_WPEEPROM */
        if ((val & 0xe0) == 0xa0) {
            qemu_system_reset_request();
        }
        self->mem[offset >> 2] = val & 0x3;
        break;
    case 2:
        self->mem[offset >> 2] = val & 0xf;
        break;
    case 3:
        self->mem[offset >> 2] = val & 0x18;
        break;
    case 4 ... 9:
        break;
    case 10 ... 13:
        /* TODO: allow date to be changed? */
        break;
    case 0x10:
        self->test = val;
        break;
    case 0x11:
        self->test = ~val;
        break;
    default:
        LOG(LOG_UNIMP, "write to unimplimented register %08x\n",
            (unsigned)offset);
        break;
    }
}

static const MemoryRegionOps cpld_ops = {
    .read = cpld_read,
    .write = cpld_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};
static
void mvme3100_cpld_realize(DeviceState *dev, Error **errp)
{
    MVMECPLD *self = CPLD(dev);

    memory_region_init_io(&self->mmio, OBJECT(self), &cpld_ops, self,
                          TYPE_CPLD, CPLD_SIZE);

    sysbus_init_mmio(&self->parent_obj, &self->mmio);
}

static Property mvme3100_cpld_props[] = {
    DEFINE_PROP_END_OF_LIST()
};

static
void mvme3100_cpld_reset(DeviceState *dev)
{
    MVMECPLD *self = CPLD(dev);

    self->mem[0] = 0; /* Type VME SBC, SAFE_START==0 */
    self->mem[1] = 3;
    self->mem[2] = 1;
    self->mem[3] = 9;
    self->mem[4] = 9;
    self->mem[5] = 0xa9;
    self->mem[6] = 1;
    self->mem[7] = 0xe0; /* TODO, TSEC phy irq status */
    self->mem[8] = 1; /* TODO: PMC presence ...? */
    self->mem[9] = 1; /* TODO: real rev. # */
    self->mem[10] = 15; /* TODO: real date code */
    self->mem[11] = 11;
    self->mem[12] = 14;
    self->mem[13] = 1;
    self->test = 0;
}

static
void mvme3100_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = &mvme3100_cpld_realize;
    dc->reset = &mvme3100_cpld_reset;
    dc->desc = "mvme3100 CPLD logic";
    dc->props = mvme3100_cpld_props;
}

static const TypeInfo mvme3100_cpld_info = {
    .name = TYPE_CPLD,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MVMECPLD),
    .class_size = sizeof(SysBusDeviceClass),
    .class_init = mvme3100_class_init,
};

static
void mvme3100_cpld_register(void)
{
    type_register_static(&mvme3100_cpld_info);
}

type_init(mvme3100_cpld_register)
