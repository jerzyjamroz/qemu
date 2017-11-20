/*
 * MPC8540 and MPC8544 specific control registers.
 * Not really part of e500 spec, but common to many
 * Freescale parts w/ e500 cores.
 *
 * Copyright (c) 2017 Michael Davidsaver
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All rights reserved.
 *
 * Authors: Alexander Graf, <alex@csgraf.de>
 *          Michael Davidsaver <mdavidsaver@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 *
 * This model was developed according to:
 *
 * MPC8544E PowerQUICC III Integrated Host Processor Family Reference Manual
 * Rev. 1
 *
 * MPC8540 PowerQUICC III Integrated Host Processor Reference Manual, Rev. 1
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "cpu.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"

/* E500_ denotes registers common to all */

#define E500_PVR         (0xE00A0)
#define E500_SVR         (0xE00A4)

#define MPC8544_RSTCR       (0xE00B0)
#define MPC8544_RSTCR_RESET      (0x02)

typedef struct {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    uint32_t defbase;
} CCSRState;

#define TYPE_E500_CCSR "e500-ccsr"
#define E500_CCSR(obj) OBJECT_CHECK(CCSRState, (obj), TYPE_E500_CCSR)

static uint64_t e500_ccsr_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    PowerPCCPU *cpu = POWERPC_CPU(current_cpu);
    CPUPPCState *env = &cpu->env;

    switch (addr) {
    case E500_PVR:
        return env->spr[SPR_PVR];
    case E500_SVR:
        return env->spr[SPR_E500_SVR];
    }

    qemu_log_mask(LOG_GUEST_ERROR | LOG_UNIMP,
                  "can't read undefined ccsr regster %x\n",
                  (unsigned)addr);
    return 0;
}

static void e500_ccsr_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    PowerPCCPU *cpu = POWERPC_CPU(current_cpu);
    CPUPPCState *env = &cpu->env;
    uint32_t svr = env->spr[SPR_E500_SVR] >> 16;

    switch (svr) {
    case 0: /* generic.  assumed to be mpc8544ds or e500plat board */
    case 0x8034: /* mpc8544 */
    case 0x803C: /* mpc8544E */
        switch (addr) {
        case MPC8544_RSTCR:
            if (value & MPC8544_RSTCR_RESET) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
            return;
        }
    }

    qemu_log_mask(LOG_GUEST_ERROR | LOG_UNIMP,
                  "can't write undefined ccsr regster %x <- %08x\n",
                  (unsigned)addr, (unsigned)value);
}

static const MemoryRegionOps e500_ccsr_ops = {
    .read = e500_ccsr_read,
    .write = e500_ccsr_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static void e500_ccsr_reset(DeviceState *dev)
{
    CCSRState *ccsr = E500_CCSR(dev);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, ccsr->defbase);
}

static void e500_ccsr_initfn(Object *obj)
{
    CCSRState *ccsr = E500_CCSR(obj);

    memory_region_init_io(&ccsr->iomem, obj, &e500_ccsr_ops,
                          ccsr, "e500-ccsr", 1024 * 1024);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &ccsr->iomem);

}

static Property e500_ccsr_props[] = {
    DEFINE_PROP_UINT32("base", CCSRState, defbase, 0xff700000),
    DEFINE_PROP_END_OF_LIST()
};

static
void e500_ccsr_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = e500_ccsr_props;
    dc->reset = e500_ccsr_reset;
}

static const TypeInfo e500_ccsr_info = {
    .name          = TYPE_E500_CCSR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CCSRState),
    .instance_init = e500_ccsr_initfn,
    .class_size    = sizeof(SysBusDeviceClass),
    .class_init    = e500_ccsr_class_initfn
};

static void e500_ccsr_register_types(void)
{
    type_register_static(&e500_ccsr_info);
}

type_init(e500_ccsr_register_types)
