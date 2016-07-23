/*
 * VME testing device
 *
 * Copyright (c) 2015 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/qdev.h"
#include "hw/vme/vme_bus.h"

#define TYPE_VMETEST "vme-test"
/* #define DEBUG_VMETEST */

#ifdef DEBUG_VMETEST
#define DBGOUT(fmt, ...) fprintf(stderr, TYPE_VMETEST ": " fmt, \
                                 ## __VA_ARGS__)
#else
#define DBGOUT(fmt, ...) do {} while (0)
#endif

#define LOG(mask, fmt, ...) qemu_log_mask(mask, fmt, ##__VA_ARGS__)

#define VMETEST(i) OBJECT_CHECK(VMETest, (i), TYPE_VMETEST)

typedef struct {
    VMEDevice parent_obj;

    uint16_t base;
    MemoryRegion reg, regs,
                 ram, rams;

    uint8_t level, vector;

    uint32_t box[2], ctrl;
} VMETest;

static
void update_irq(VMETest *test)
{
    int sig = test->level && test->vector && (test->ctrl & 1);

    DBGOUT("IRQ%u %c\n", test->level, sig ? 'X' : '_');

    if (test->level) {
        vme_set_irq(&test->parent_obj, test->level, sig);
    }
}

static
uint64_t test_read(void *opaque, hwaddr addr, unsigned size)
{
    const unsigned offset = addr;
    VMETest *test = opaque;
    uint32_t val;

    switch (offset) {
    case 0 ... 3:
        val = 0x12345678;
        val <<= 8 * offset;
        val >>= 8 * (4 - size);
        break;
    case 4 ... 7:
        val = size;
        break;
    case 8:
        val = test->box[0];
        break;
    case 0xc:
        val = test->box[1];
        break;
    case 0x10:
        val = test->vector;
        break;
    case 0x14:
        val = test->level;
        break;
    case 0x1c:
        val = test->ctrl;
        break;
    default:
        LOG(LOG_GUEST_ERROR, "read from unimplemented %08x\n", offset);
        val = 0xffffffff;
    }

    DBGOUT("read %08x -> %08x\n", offset, (unsigned)val);

    return val;
}

static
void test_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    const unsigned offset = addr;
    VMETest *test = opaque;

    DBGOUT("write %08x <- %08x\n", offset, (unsigned)val);

    switch (offset) {
    case 0 ... 7:
        break;
    case 8 ... 0xb:
        test->box[0] = size;
        break;
    case 0xc:
        test->box[1] = val;
        break;
    case 0x10:
        test->vector = val & 0xff;
        update_irq(test);
        break;
    case 0x14:
        if (test->level && test->level != (val & 7)) {
            vme_set_irq(&test->parent_obj, test->level, 0);
        }
        test->level = val & 7;
        update_irq(test);
        break;
    case 0x1c:
        test->ctrl = val & 3;
        update_irq(test);
        break;
    default:
        LOG(LOG_GUEST_ERROR, "write to unimplemented %08x\n", offset);
    }
}

static const MemoryRegionOps test_ops = {
    .read  = test_read,
    .write = test_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static
bool vme_test_iack(struct VMEDevice *dev, uint8_t lvl, uint32_t *pvect)
{
    VMETest *test = container_of(dev, VMETest, parent_obj);

    if (dev->irq_status) {
        *pvect = test->vector;
        if (test->ctrl & 2) { /* ROAK */
            test->ctrl &= ~1;
            update_irq(test);
        }
        DBGOUT("IACK %u %08x\n", lvl, (unsigned)*pvect);
        return true;
    } else {
        DBGOUT("IACK %u ??\n", lvl);
        return false;
    }
}

static
void vme_test_realize(VMEDevice *vme, Error **errp)
{
    VMETest *test = VMETEST(vme);

    DBGOUT("realize() %p slot=%u\n", test, vme->slot);

    vme->iack = &vme_test_iack;

    memory_region_init_io(&test->reg, &vme->qdev.parent_obj, &test_ops,
                          test, TYPE_VMETEST"-usr", 0x20);
    /* we respond to both supervisor and normal (program) AMs */
    memory_region_init_alias(&test->regs, OBJECT(vme), TYPE_VMETEST"-sup",
                             &test->reg, 0, 0x20);

    memory_region_init_ram(&test->ram, OBJECT(vme), TYPE_VMETEST"-ram-usr",
                           0x100, &error_fatal);
    memory_region_init_alias(&test->rams, OBJECT(vme),
                             TYPE_VMETEST"-ram-sup",  &test->ram,
                             0, 0x100);

    /* We "ignore" supervisor AM bit.
     * Add register range to both AMs.
     */
    vme_add_region(vme, VME_AM_A16_USR, test->base, &test->reg);
    vme_add_region(vme, VME_AM_A16_SUP, test->base, &test->regs);

    vme_add_region(vme, VME_AM_A16_USR, test->base+0x100, &test->ram);
    vme_add_region(vme, VME_AM_A16_SUP, test->base+0x100, &test->rams);
}

static Property vme_test_props[] = {
    DEFINE_PROP_UINT16("base", VMETest, base, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static
void vme_test_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    VMEDeviceClass *vme = VME_CLASS(klass);

    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = "VME test device";
    dc->props = vme_test_props;
    vme->realize = vme_test_realize;
}

static const TypeInfo vme_test_type_info = {
    .name = TYPE_VMETEST,
    .parent = TYPE_VME,
    .class_size = sizeof(VMEDeviceClass),
    .instance_size = sizeof(VMETest),
    .class_init = vme_test_class_init,
};

static
void vme_test_register(void)
{
    type_register_static(&vme_test_type_info);
}

type_init(vme_test_register);
