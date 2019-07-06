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
#include "qapi/error.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/boards.h"
#include "sysemu/sysemu.h"
#include "sysemu/kvm.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/ppc/openpic.h"
#include "hw/ppc/openpic_kvm.h"

/* E500_ denotes registers common to all */
/* Some CCSR offsets duplicated in e500.c */

#define E500_CCSRBAR     (0)

#define E500_CS0_BNDS    (0x2000)

#define E500_CS0_CONFIG  (0x2080)

#define E500_ERR_DETECT  (0x2e40)
#define E500_ERR_DISABLE (0x2e44)

#define E500_I2C_OFFSET  (0x3000)

#define E500_DUART_OFFSET(N) (0x4500 + (N) * 0x100)

#define E500_PCI_OFFSET  (0x8000ULL)

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

    /* CCSR base address is 36-bit */
    uint64_t defbase, base;
    uint32_t ram_size;
    uint32_t merrd;

    uint32_t porpllsr;
    uint32_t ccb_freq;

    DeviceState *pic;
    DeviceState *i2c;
    DeviceState *pcihost;
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

static void e500_ccsr_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    CCSRState *ccsr = E500_CCSR(dev);

    /* prepare MPIC */
    assert(current_machine);
    if (kvm_enabled() && machine_kernel_irqchip_allowed(current_machine)) {

        ccsr->pic = qdev_create(NULL, TYPE_KVM_OPENPIC);

        if (!ccsr->pic && machine_kernel_irqchip_required(current_machine)) {
            error_report("kernel_irqchip requested but unavailable: ");
            exit(1);
        }
    }

    if (!ccsr->pic) {
        ccsr->pic = qdev_create(NULL, TYPE_OPENPIC);

        qdev_prop_set_uint32(ccsr->pic, "nb_cpus", smp_cpus);
    }

    if (!ccsr->pic) {
        error_report("Failed to create PIC");
        exit(1);
    }

    object_property_add_child(qdev_get_machine(), "pic", OBJECT(ccsr->pic),
                              &error_fatal);

    object_property_add_alias(obj, "mpic-model",
                              OBJECT(ccsr->pic), "model",
                              &error_fatal);
    /* create i2c controller */
    ccsr->i2c = qdev_create(NULL, "mpc8540-i2c");
    object_property_add_child(qdev_get_machine(), "i2c[*]",
                              OBJECT(ccsr->i2c), NULL);
    qdev_init_nofail(ccsr->i2c);

    /* prepare PCI host bridge */
    ccsr->pcihost = qdev_create(NULL, "e500-pcihost");
    object_property_add_child(qdev_get_machine(), "pci-host", OBJECT(ccsr->pcihost),
                              &error_abort);

    object_property_add_alias(obj, "pci_first_slot",
                              OBJECT(ccsr->pcihost), "first_slot",
                              &error_fatal);
    object_property_add_alias(obj, "pci_first_pin_irq",
                              OBJECT(ccsr->pcihost), "first_pin_irq",
                              &error_fatal);
}

static void e500_ccsr_realize(DeviceState *dev, Error **errp)
{
    CCSRState *ccsr = E500_CCSR(dev);
    SysBusDevice *pic;

    /* Base 1MB CCSR Region */
    memory_region_init_io(&ccsr->iomem, OBJECT(dev), &e500_ccsr_ops,
                          ccsr, "e500-ccsr", 1024 * 1024);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &ccsr->iomem);

    /* realize MPIC */
    qdev_init_nofail(ccsr->pic);
    pic = SYS_BUS_DEVICE(ccsr->pic);

    /* connect MPIC to CPU(s) */
    if (object_dynamic_cast(OBJECT(pic), TYPE_KVM_OPENPIC)) {
        CPUState *cs;
        CPU_FOREACH(cs) {
            if (kvm_openpic_connect_vcpu(ccsr->pic, cs)) {
                error_setg(errp, "%s: failed to connect vcpu to irqchip",
                           __func__);
                return;
            }
        }

    } else {
        CPUState *cs;
        CPU_FOREACH(cs) {
            PowerPCCPU *cpu = POWERPC_CPU(cs);
            CPUPPCState *env = &cpu->env;
            qemu_irq *inputs = (qemu_irq *)env->irq_inputs;
            int base = cs->cpu_index * PPCE500_INPUT_NB;

            sysbus_connect_irq(pic, base + OPENPIC_OUTPUT_INT,
                               inputs[PPCE500_INPUT_INT]);
            sysbus_connect_irq(pic, base + OPENPIC_OUTPUT_CINT,
                               inputs[PPCE500_INPUT_CINT]);
        }
    }

    memory_region_add_subregion(&ccsr->iomem, E500_MPIC_OFFSET,
                                sysbus_mmio_get_region(pic, 0));
    /* Note: MPIC internal interrupts are offset by 16 */

    /* realize PCI host bridge*/
    qdev_init_nofail(ccsr->pcihost);

    memory_region_add_subregion(&ccsr->iomem, E500_PCI_OFFSET,
                                sysbus_mmio_get_region(
                                    SYS_BUS_DEVICE(ccsr->pcihost), 0));

    /* attach I2C controller */
    memory_region_add_subregion(&ccsr->iomem, E500_I2C_OFFSET,
                                sysbus_mmio_get_region(
                                    SYS_BUS_DEVICE(ccsr->i2c), 0));
    sysbus_connect_irq(SYS_BUS_DEVICE(ccsr->i2c), 0,
                          qdev_get_gpio_in(ccsr->pic, 16 + 27));


    /* DUARTS */
    /* for mpc8540, mpc8544, and P2010 (unmodeled), the DUART reference clock
     * is the CCB clock divided by 16.
     * So baud rate is CCB/(16*divider)
     */
    if (serial_hds[0]) {
        serial_mm_init(&ccsr->iomem, E500_DUART_OFFSET(0), 0,
                       qdev_get_gpio_in(ccsr->pic, 16 + 26),
                       ccsr->ccb_freq / 16u,
                       serial_hds[0], DEVICE_BIG_ENDIAN);
    }

    if (serial_hds[1]) {
        serial_mm_init(&ccsr->iomem, E500_DUART_OFFSET(1), 0,
                       qdev_get_gpio_in(ccsr->pic, 16 + 26),
                       ccsr->ccb_freq / 16u,
                       serial_hds[1], DEVICE_BIG_ENDIAN);
    }

}

static Property e500_ccsr_props[] = {
    DEFINE_PROP_UINT64("base", CCSRState, defbase, 0xff700000),
    DEFINE_PROP_UINT32("ram-size", CCSRState, ram_size, 0),
    DEFINE_PROP_UINT32("porpllsr", CCSRState, porpllsr, 0),
    DEFINE_PROP_UINT32("ccb-freq", CCSRState, ccb_freq, 333333333u),
    /* "mpic-model" aliased from MPIC */
    /* "pci_first_slot"
     * "pci_first_pin_irq" aliased from PCI host bridge
     */
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_e500_ccsr = {
    .name = TYPE_E500_CCSR,
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = e500_ccsr_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT64(base, CCSRState),
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
    dc->realize = e500_ccsr_realize;
    dc->reset = e500_ccsr_reset;
}

static const TypeInfo e500_ccsr_info = {
    .name          = TYPE_E500_CCSR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CCSRState),
    .instance_init = e500_ccsr_init,
    .class_size    = sizeof(SysBusDeviceClass),
    .class_init    = e500_ccsr_class_initfn
};

static void e500_ccsr_register_types(void)
{
    type_register_static(&e500_ccsr_info);
}

type_init(e500_ccsr_register_types)
