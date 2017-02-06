/*
 * MVME3100 board emulation
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
#include "qapi/error.h"
#include "e500.h"
#include "cpu.h"
#include "qemu-common.h"
#include "cpu-qom.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "hw/loader.h"
#include "hw/pci/pci.h"
#include "hw/boards.h"
#include "hw/ppc/ppc.h"
#include "hw/net/fsl_etsec/etsec.h"
#include "sysemu/device_tree.h"
#include "sysemu/qtest.h"
#include "hw/ppc/openpic.h"
#include "qemu/error-report.h"
#include "hw/nvram/eeprom_at24c.h"

/* motload "global environment" variables */
static
const char gev[] =
        /* TODO: somehow snoop in slirp_instances to pick up IP config? */
        "mot-/dev/enet0-cipa=10.0.2.15\0"
        "mot-/dev/enet0-gipa=10.0.2.2\0"
        "mot-/dev/enet0-snma=255.255.255.0\0"
        "mot-/dev/enet0-sipa=10.0.2.2\0"
        /* RTEMS specific names for things motload doesn't have */
        "rtems-dns-server=10.0.2.3\0"
        "rtems-client-name=qemu\0"
        ;

static
void fill_vpd(DeviceState *dev, const char *extra)
{
    uint32_t cnt = 0;
#define WRITE(N, BUF)  \
    do {at24c_eeprom_write(dev, cnt, N, BUF); cnt += N; } while (0)

    WRITE(8, "MOTOROLA");

    /* model name */
    WRITE(16, "\x01\x0eMVME3100-1152");
    /* serial number */
    WRITE(11, "\x03\x09""E0120000");

    WRITE(2, "\x06\x05"); /* (PCI) Bus freq. */
    {
        uint32_t val = cpu_to_be32(66666666);
        WRITE(4, (char *)&val);
        cnt += 1; /* skip one */
    }

    if (nd_table[0].used) {
        WRITE(2, "\x08\x07");
        WRITE(6, (char *)nd_table[0].macaddr.a);
        cnt += 1; /* skip one */
    }
    if (nd_table[1].used) {
        WRITE(2, "\x08\x07");
        WRITE(6, (char *)nd_table[1].macaddr.a);
        cnt += 1; /* skip one */
    }

    WRITE(2, "\xff\x00"); /* End */

    /* MOTLOAD's Global Environment Variables
     * start at offset 0x10f8.
     * This is a set nil terminated strings of the form "name=value"
     * with a zero length string signaling the end.
     */
    cnt = 0x10f8;
    at24c_eeprom_write(dev, cnt, sizeof(gev)-1, gev);
    cnt += sizeof(gev)-1;

    if (extra) {
        char *E = g_strdup(extra);
        char **opts = g_strsplit(E, " ", 0);
        size_t i;

        g_free(E);

        for (i=0; opts[i]; i++) {
            char *opt = g_strstrip(opts[i]);
            size_t olen = strlen(opt);

            if (olen==0)
                continue;

            at24c_eeprom_write(dev, cnt, olen+1, opt);
            cnt += olen+1;
        }

        g_strfreev(opts);
    }

    WRITE(2, "\x00\x00"); /* End */

    /* TODO CRC? (RTEMS doesn't check) */

#undef WRITE
}

static void mvme3100_fixup_devtree(PPCE500Params *params, void *fdt)
{
    (void)params;
    (void)fdt;
}

static
void set_map(CPUPPCState *env, unsigned way,
             target_ulong va, hwaddr pa,
             unsigned size, bool X)
{
    ppcmas_tlb_t *tlb = booke206_get_tlbm(env, 1, 0, way);

    tlb->mas1 = MAS1_VALID | (size << MAS1_TSIZE_SHIFT);
    tlb->mas2 = (va & TARGET_PAGE_MASK) | MAS2_M;
    tlb->mas7_3 = pa & TARGET_PAGE_MASK;
    tlb->mas7_3 |= MAS3_UR | MAS3_UW | MAS3_SR | MAS3_SW;
    if (X) {
        tlb->mas7_3 |= MAS3_UX | MAS3_SX;
    } else {
        tlb->mas2 |= MAS2_I | MAS2_G;
    }
}

static
void remap_tlb_bare(CPUPPCState *env)
{
    /* Default memory map when the CPU resets... depends on who you ask.
     *
     * The MPC8540 ref. manual says only the upper 4KB.
     * The mvme3100 programmers guide says CCSR (1 MB) and
     * ROM (8 MB).
     *
     * We take the more liberal approach.
     */

    /* The mpc8540 mapping */
    /* set_map(env, 0, 0xfffff000, 0xfffff000, 0x02, 1); */

    /* The mvme3100 mapping */
    set_map(env, 0, 0xff800000, 0xff800000, 0x0c, 1);
    set_map(env, 1, 0xffc00000, 0xffc00000, 0x0c, 1);
    set_map(env, 2, 0xff700000, 0xff700000, 0x0a, 0);

    env->tlb_dirty = true;
}

/* runs after ppce500_cpu_reset() */
static void mvme3100_cpu_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUPPCState *env = &cpu->env;

    remap_tlb_bare(&cpu->env);
    env->nip = 0xfffffffc;
}

static
void mvme3100_pci1_set_irq(void *opaque, int irq_num, int level)
{
    qemu_irq *pic = opaque;

    printf("mvme3100 pci1 irq %d %d\n", irq_num, level);
    qemu_set_irq(pic[irq_num], level);
}

/* PCI config from a real mvme3100 as configured by motload
 *
 *  BUS:SLOT:FUN  VENDOR-DEV_ID: COMMAND STATUS BASE_ADDR0 BASE_ADDR1 IRQ_PIN -> IRQ_LINE
 *  0:0x00:0    0x1057-0x0008:  0x0006 0x20B0 0x80000000 0x00000000       0 ->   0 (=0x00)
 *  0:0x11:0    0x10E3-0x0148:  0x0146 0x02B0 0x80100004 0x00000000       1 ->   0 (=0x00)
 *  0:0x12:0    0x10B5-0x6520:  0x0147 0x02B0 0x00000000 0x00000000       0 ->   0 (=0x00)
 *  0:0x13:0    0x10B5-0x6520:  0x0147 0x02B0 0x00000000 0x00000000       0 ->   0 (=0x00)
 *  0:0x14:0    0x8086-0x3200:  0x0145 0x02B0 0x00012001 0x00013001       1 ->   2 (=0x02)
 *  2:0x00:0    0x1033-0x0035:  0x0146 0x0210 0x80300000 0x00000000       1 ->   4 (=0x04)
 *  2:0x00:1    0x1033-0x0035:  0x0146 0x0210 0x80301000 0x00000000       2 ->   5 (=0x05)
 *
 * We don't model:
 * # The PLX bridges for expansion cards (0x10B5-0x6520),
 * # The SATA controller (0x8086-0x3200)
 * # The USB controllers (0x1033-0x0035)
 *
 * (Note that the SATA controller found on newer boards is different)
 */

static void mvme3100_init(MachineState *machine)
{
    /* Config based on system state from MOTLoad, not power on */
    static
    PPCE500Params params = {
        .pci_first_slot = 0,
        .pci_nr_slots = 5,
        .fixup_devtree = mvme3100_fixup_devtree,
        .mpic_version = OPENPIC_MODEL_FSL_MPIC_20,
        .ccsrbar_base = 0xff700000ULL,
        .pci_mmio_base = 0x80000000ULL,
        .pci_mmio_bus_base = 0x80000000ULL,
        .pci_pio_base = 0xE0000000ULL,
        .spin_base = 0xE3000000ULL, /* not used */
        .skip_load = 1,
        .tsec_nic = true,
        /* plat ratio = 5 -> 5:1 CCB:PCI
         * e500 ratio = 4 -> 4:1 e500:CCB
         */
        .porpllsr = 0x0004000a,
        /* Decrementor frequency experimentally determined by guest comparison
         * of RTC w/ NTP time.
         *
         * The RTEMS mvme3100 BSP assumes 41666 ticks per millisecond.
         * It has been experimentally determined that the actual count rate
         * should be a higher (42066) for some reason.
         *
         * Note that this number reflects whatever motload would do before
         * boot. This involves the time base control bits in the HID0
         * register, which are not currently modeled, and may use a board
         * specific clock.
         */
        .decrementor_freq = 42016666,
    };
    DeviceState *dev;
    BusState *i2c;
    PCIBus *pci0, *pci1;
    SysBusDevice *cpld, *ccsr, *pic;
    qemu_irq *pci1_pins = g_malloc_n(4, sizeof(*pci1_pins));

    if (!machine->cpu_model) {
        machine->cpu_model = "e500_v20";
    }

    /* Setup CPU */
    ppce500_init(machine, &params);

    qemu_register_reset(mvme3100_cpu_reset, POWERPC_CPU(first_cpu));

    pic = SYS_BUS_DEVICE(object_resolve_path("/machine/pic", NULL));
    ccsr = SYS_BUS_DEVICE(object_resolve_path("/machine/e500-ccsr", NULL));

    /* Setup mvme3100 specific CPLD device (and CCSR DRAM control) */
    cpld = SYS_BUS_DEVICE(qdev_create(NULL, "mvme3100-cpld"));
    object_property_add_child(qdev_get_machine(), "cpld",
                              OBJECT(cpld), &error_fatal);
    qdev_init_nofail(DEVICE(cpld));

    memory_region_add_subregion(get_system_memory(),
                                0xe2000000,
                                sysbus_mmio_get_region(cpld, 0));

    dev = DEVICE(object_resolve_path("/machine/pci-host", NULL));
    assert(dev);
    pci0 = PCI_BUS(qdev_get_child_bus(dev, "pci.0"));
    assert(pci0);

    /* Add expansion PCI bus (2x PMC sites)
     * "pci-bridge" is not a PLX bridge, but shouldn't matter?
     */
    dev = qdev_create(BUS(pci0), "pci-bridge");

    qdev_prop_set_uint8(dev, "chassis_nr", 1);
    qdev_prop_set_int32(dev, "addr", PCI_DEVFN(0x12, 0));

    qdev_init_nofail(dev);

    pci1 = PCI_BUS(qdev_get_child_bus(dev, "pci.1"));
    assert(pci1);

    pci1_pins[0] = qdev_get_gpio_in(DEVICE(pic), 4);
    pci1_pins[1] = qdev_get_gpio_in(DEVICE(pic), 5);
    pci1_pins[2] = qdev_get_gpio_in(DEVICE(pic), 6);
    pci1_pins[3] = qdev_get_gpio_in(DEVICE(pic), 7);

    pci_bus_irqs(pci1, mvme3100_pci1_set_irq, pci_swizzle_map_irq_fn, pci1_pins, 4);

    /* the actual PLX bridge doesn't emit interrupts */
    pci_set_byte(PCI_DEVICE(dev)->config + PCI_INTERRUPT_PIN, 0);

    /* I2C Controller */
    dev = DEVICE(object_resolve_path("/machine/i2c[0]", NULL));
    assert(dev);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(DEVICE(pic), 16 + 27));
    i2c = qdev_get_child_bus(dev, "bus");
    assert(i2c);

    /* The onboard PCI devices (bus 0) have an arbitary IRQ mapping.
     * The TSI18 is a single function devices which uses all 4 IRQ pins
     * QEMU doesn't support this.
     * so we work around this by doing the routing ourselves
     */
    if (object_class_by_name("tsi148")) {
        PCIDevice *pdev;

        pdev = pci_create_multifunction(pci0, PCI_DEVFN(0x11, 0),
                                        false, "tsi148");
        dev = DEVICE(pdev);

        qdev_connect_gpio_out_named(dev, "IRQ", 0,
                                    qdev_get_gpio_in(DEVICE(pic), 0));
        qdev_connect_gpio_out_named(dev, "IRQ", 1,
                                    qdev_get_gpio_in(DEVICE(pic), 1));
        qdev_connect_gpio_out_named(dev, "IRQ", 2,
                                    qdev_get_gpio_in(DEVICE(pic), 2));
        qdev_connect_gpio_out_named(dev, "IRQ", 3,
                                    qdev_get_gpio_in(DEVICE(pic), 3));

        qdev_init_nofail(dev);
    } else {
        printf("Unable to add tsi148 vme bridge"
               " (qemu built w/o CONFIG_TSI148)\n");
    }

    /* NIC */
    if (nd_table[0].used) {
        qemu_check_nic_model(&nd_table[0], "eTSEC");

        dev = etsec_create(0x24000,
                           sysbus_mmio_get_region(ccsr, 0),
                           &nd_table[0],
                qdev_get_gpio_in(DEVICE(pic), 16 + 13),
                qdev_get_gpio_in(DEVICE(pic), 16 + 14),
                qdev_get_gpio_in(DEVICE(pic), 16 + 18));

    } else if (nd_table[1].used) {
        qemu_check_nic_model(&nd_table[1], "eTSEC");

        dev = etsec_create(0x25000,
                           sysbus_mmio_get_region(ccsr, 0),
                           &nd_table[1],
                sysbus_get_connected_irq(pic, 16 + 19),
                sysbus_get_connected_irq(pic, 16 + 20),
                sysbus_get_connected_irq(pic, 16 + 23));
    }

    /* VPD EEPROM */
    dev = qdev_create(i2c, "at24c-eeprom");
    object_property_add_child(qdev_get_machine(), "vpd", OBJECT(dev),
                              &error_fatal);
    qdev_prop_set_uint8(dev, "address", 0xa8 >> 1);
    qdev_prop_set_uint32(dev, "rom-size", 8192 * 8);
    qdev_init_nofail(dev);

    fill_vpd(dev, machine->kernel_cmdline);

    /* DS1375 RTC */
    dev = qdev_create(i2c, "ds1375");
    object_property_add_child(qdev_get_machine(), "rtc", OBJECT(dev),
                              &error_fatal);
    qdev_prop_set_uint8(dev, "address", 0xd0 >> 1);
    qdev_init_nofail(dev);

    /* root bus is only home to soldered devices, and has a
     * an arbitrary IRQ pin mapping.
     * Don't allow qdev_device_add() to consider it.
     */
    {
        BusState *bpci0 = BUS(pci0);
        BusClass *bcls  = BUS_GET_CLASS(pci0);
        assert(bpci0);

        /* bus 0 is thus declared to be full.
         * as a side-effect, expansion PCI bus limited to 15 devices
         */
        bpci0->max_index = bcls->max_dev = 15;
    }

    /* TODO: unmodeled i2c devices.
     * 0x90 - ds1621 temperature sensor
     * 0xa0 - 256*8 byte DDR SPD (???)
     * 0xa4 - 64k*8 byte eeprom for "user" configuration
     * 0xa6 - 64k*8 byte eeprom for "user" configuration
     * 0xaa -  8k*8 byte eeprom for VPD of rear expansion card
     */

    if (!bios_name) {
        bios_name = "tomload.bin";
    }

    if(!qtest_enabled()) {
        MemoryRegion *rom = g_malloc0(sizeof(*rom));
        char *fullname = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);

        if(!fullname) {
            fprintf(stderr, "qemu: could not find bios file '%s'\n",
                    bios_name);
            exit(1);
        }

        memory_region_init_ram(rom, OBJECT(qdev_get_machine()), "rom",
                               0x800000, &error_fatal);

        memory_region_add_subregion(get_system_memory(),
                                    0xff800000, rom);

        /* BIOS == image in rom (last 8MB of address space.
         * Execution starts with the final word 0xfffffffc
         */
        int bsize = get_image_size(fullname);
        if (bsize != 8 * 1024 * 1024 ||
                -1 == load_image_targphys(fullname,
                                          0xff800000, 8 * 1024 * 1024))
        {
            fprintf(stderr, "qemu: could not load bios file '%s' (%u bytes, requires 8MB)\n",
                    fullname, (unsigned)bsize);
            exit(1);
        }

        memory_region_set_readonly(rom, true);

        g_free(fullname);
    }

    if (machine->kernel_filename &&
            -1 == load_image_targphys(machine->kernel_filename,
                                    0x10000, 0x01000000))
    {
        fprintf(stderr, "qemu: could not load file '%s'\n",
                machine->kernel_filename);
        exit(1);
    }
}


static void ppce500_machine_init(MachineClass *mc)
{
    /* Hack to handle mrf cards with internal sysbus */
    mc->has_dynamic_sysbus = 1;
    mc->desc = "mvme3100-1152";
    mc->init = mvme3100_init;
    mc->max_cpus = 1;
    mc->default_ram_size = 256 * (1 << 20);
}

DEFINE_MACHINE("mvme3100-1152", ppce500_machine_init)
