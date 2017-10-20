/*
 * MPC8540 and MPC8544 specific control registers
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
#include "cpu.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "hw/sysbus.h"

/* MPC85xx_ denotes registers common to both */

#define MPC85xx_PVR         (0xE00A0)
#define MPC85xx_SVR         (0xE00A4)

#define MPC8544_RSTCR       (0xE00B0)
#define MPC8544_RSTCR_RESET      (0x02)

typedef struct {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    uint32_t model; /* 8540 or 8544 */

    MemoryRegion iomem;

    uint32_t defbase;
} CCSRState;

#define TYPE_MPC85XX_CCSR "mpc85xx-ccsr"
#define MPC8544_CCSR(obj) OBJECT_CHECK(CCSRState, (obj), TYPE_MPC85XX_CCSR)

static uint64_t mpc85xx_ccsr_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    PowerPCCPU *cpu = POWERPC_CPU(current_cpu);
    CPUPPCState *env = &cpu->env;

    switch (addr) {
    case MPC85xx_PVR:
        return env->spr[SPR_PVR];
    case MPC85xx_SVR:
        return env->spr[SPR_E500_SVR];
    }

    qemu_log_mask(LOG_GUEST_ERROR | LOG_UNIMP,
                  "can't read undefined ccsr regster %x\n",
                  (unsigned)addr);
    return 0;
}

static void mpc85xx_ccsr_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned size)
{
    CCSRState *ccsr = opaque;

    if (ccsr->model == 8544) {
        switch (addr) {
        case MPC8544_RSTCR:
            if (value & MPC8544_RSTCR_RESET) {
                qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
            }
            return;
        }
    }

    qemu_log_mask(LOG_GUEST_ERROR | LOG_UNIMP,
                  "can't write undefined ccsr regster %x\n",
                  (unsigned)addr);
}

static const MemoryRegionOps mpc85xx_ccsr_ops = {
    .read = mpc85xx_ccsr_read,
    .write = mpc85xx_ccsr_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static void mpc85xx_ccsr_reset(DeviceState *dev)
{
    CCSRState *ccsr = MPC8544_CCSR(dev);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, ccsr->defbase);
}

static void mpc85xx_ccsr_initfn(Object *obj)
{
    CCSRState *ccsr = MPC8544_CCSR(obj);

    memory_region_init_io(&ccsr->iomem, obj, &mpc85xx_ccsr_ops,
                          ccsr, "mpc85xx-ccsr", 1024 * 1024);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &ccsr->iomem);

}

static Property mpc85xx_ccsr_props[] = {
    DEFINE_PROP_UINT32("model", CCSRState, model, 8544),
    DEFINE_PROP_UINT32("base", CCSRState, defbase, 0xff700000),
    DEFINE_PROP_END_OF_LIST()
};

static
void mpc85xx_ccsr_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = mpc85xx_ccsr_props;
    dc->reset = mpc85xx_ccsr_reset;
}

static const TypeInfo mpc85xx_ccsr_info = {
    .name          = TYPE_MPC85XX_CCSR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CCSRState),
    .instance_init = mpc85xx_ccsr_initfn,
    .class_size    = sizeof(SysBusDeviceClass),
    .class_init    = mpc85xx_ccsr_class_initfn
};

static void mpc85xx_ccsr_register_types(void)
{
    type_register_static(&mpc85xx_ccsr_info);
}

type_init(mpc85xx_ccsr_register_types)
