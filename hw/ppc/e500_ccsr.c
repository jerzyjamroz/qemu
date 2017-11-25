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
/* Some CCSR offsets duplicated in e500.c */

#define E500_CCSRBAR     (0)

#define E500_CS0_BNDS    (0x2000)

#define E500_CS0_CONFIG  (0x2080)

#define E500_ERR_DETECT  (0x2e40)
#define E500_ERR_DISABLE (0x2e44)

#define E500_PORPLLSR    (0xE0000)
#define E500_PVR         (0xE00A0)
#define E500_SVR         (0xE00A4)

#define MPC8544_RSTCR       (0xE00B0)
#define MPC8544_RSTCR_RESET      (0x02)

#define E500_MPIC_OFFSET   (0x40000ULL)

typedef struct {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    uint32_t defbase, base;
    uint32_t ram_size;
    uint32_t merrd;

    uint32_t porpllsr;

    DeviceState *pic;
} CCSRState;

#define TYPE_E500_CCSR "e500-ccsr"
#define E500_CCSR(obj) OBJECT_CHECK(CCSRState, (obj), TYPE_E500_CCSR)

/* call after changing CCSRState::base */
static void e500_ccsr_post_move(CCSRState *ccsr)
{
    CPUState *cs;

    CPU_FOREACH(cs) {
        PowerPCCPU *cpu = POWERPC_CPU(cs);
        CPUPPCState *env = &cpu->env;

        env->mpic_iack = ccsr->base +
                         E500_MPIC_OFFSET + 0xa0;
    }

    sysbus_mmio_map(SYS_BUS_DEVICE(ccsr), 0, ccsr->base);
}

static uint64_t e500_ccsr_read(void *opaque, hwaddr addr,
                                  unsigned size)
{
    CCSRState *ccsr = opaque;
    PowerPCCPU *cpu = POWERPC_CPU(current_cpu);
    CPUPPCState *env = &cpu->env;

    switch (addr) {
    case E500_CCSRBAR:
        return ccsr->base >> 12;
    case E500_CS0_BNDS:
        /* we model all RAM in a single chip with addresses [0, ram_size) */
        return (ccsr->ram_size - 1) >> 24;
    case E500_CS0_CONFIG:
        return 1 << 31;
    case E500_ERR_DETECT:
        return 0; /* (errors not modeled) */
    case E500_ERR_DISABLE:
        return ccsr->merrd;
    case E500_PORPLLSR:
        if (!ccsr->porpllsr) {
            qemu_log_mask(LOG_UNIMP,
                          "Machine does not provide valid PORPLLSR\n");
        }
        return ccsr->porpllsr;
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
    CCSRState *ccsr = opaque;
    PowerPCCPU *cpu = POWERPC_CPU(current_cpu);
    CPUPPCState *env = &cpu->env;
    uint32_t svr = env->spr[SPR_E500_SVR] >> 16;

    switch (addr) {
    case E500_CCSRBAR:
        value &= 0x000fff00;
        ccsr->base = value << 12;
        e500_ccsr_post_move(ccsr);
        return;
    case E500_ERR_DISABLE:
        ccsr->merrd = value & 0xd;
        return;
    }

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

static int e500_ccsr_post_load(void *opaque, int version_id)
{
    CCSRState *ccsr = opaque;

    e500_ccsr_post_move(ccsr);
    return 0;
}

static void e500_ccsr_reset(DeviceState *dev)
{
    CCSRState *ccsr = E500_CCSR(dev);

    ccsr->base = ccsr->defbase;
    e500_ccsr_post_move(ccsr);
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
    DEFINE_PROP_UINT32("ram-size", CCSRState, ram_size, 0),
    DEFINE_PROP_UINT32("porpllsr", CCSRState, porpllsr, 0),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_e500_ccsr = {
    .name = TYPE_E500_CCSR,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = e500_ccsr_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(base, CCSRState),
        VMSTATE_UINT32(merrd, CCSRState),
        VMSTATE_END_OF_LIST()
    }
};

static
void e500_ccsr_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->props = e500_ccsr_props;
    dc->vmsd = &vmstate_e500_ccsr;
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
