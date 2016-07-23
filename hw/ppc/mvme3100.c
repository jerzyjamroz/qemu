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
 * mvme3100-1152
 *   677MHz core, 256MB ram, 64MB flash
 * mvme3100-1263
 *   833MHz core, 512MB ram, 128MB flash
 *
 * MOTLoad on mvme3100-1152 says:
 *   MPU-Type             =MPC8540
 *   MPU-Int Clock Speed  =666MHz
 *   MPU-CCB Clock Speed  =333MHz
 *   MPU-DDR Clock Speed  =166MHz
 *   MPU-PCI Clock Speed  =66MHz, PCI, 64-bit
 *   MPU-Int Cache(L2) Enabled, 256KB, L2CTL =A8000300
 *   Reset/Boot Vector    =Flash0
 *   Local Memory Found   =10000000 (&268435456)
 *
 * MOTLoad on mvme3100-1263 says:
 *   MPU-Type             =MPC8540
 *   MPU-Int Clock Speed  =833MHz
 *   MPU-CCB Clock Speed  =333MHz
 *   MPU-DDR Clock Speed  =166MHz
 *   MPU-PCI Clock Speed  =66MHz, PCI, 64-bit
 *   MPU-Int Cache(L2) Enabled, 256KB, L2CTL =A8000300
 *   Reset/Boot Vector    =Flash0
 *   Local Memory Found   =20000000 (&536870912)
 *
 * Clock ratios
 *   CCB/PCI  -> 5/1
 *   core/CCB -> 2/1 (-1152)
 *            -> 5/2 (-1263)
 *
 * The overall memory map is determined by the Local Address Windows.
 * We do not model the LAWs explicitly.
 *
 * MOTLoad configures as follows (a super set of table 1-4)
 *   (MOTLoad RTOS Version 2.0,  PAL Version 1.2 RM04)
 * LAW 0, 7 - disabled
 * LAW 1 - 0x00000000 -> 0x7fffffff - RAM 2G
 * LAW 2 - 0x80000000 -> 0xbfffffff - PCI 1G
 * LAW 3 - 0xc0000000 -> 0xdfffffff - PCI 512MB
 * LAW 4 - 0xe0000000 -> 0xe0ffffff - PCI 16MB
 * gap   - 0xe1000000 -> 0xbfffffff - CCSR @ 0xe1000000
 * LAW 5 - 0xe2000000 -> 0xe2ffffff - LBC 16MB
 * gap   - 0xe3000000 -> 0xefffffff
 * LAW 6 - 0xf0000000 -> 0xffffffff - LBC 256MB
 *
 * And validated against the RTEMS 4.9.6 mvme3100 BSP
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "e500.h"
#include "cpu.h"
#include "qemu-common.h"
#include "cpu-qom.h"
#include "sysemu/sysemu.h"
#include "sysemu/dma.h"
#include "sysemu/block-backend.h"
#include "hw/loader.h"
#include "hw/pci/pci.h"
#include "hw/boards.h"
#include "hw/ppc/ppc.h"
#include "hw/net/fsl_etsec/etsec.h"
#include "sysemu/device_tree.h"
#include "sysemu/qtest.h"
#include "hw/ppc/openpic.h"
#include "qemu/error-report.h"

/* Same as prep.c and other PPC boards */
#define CFG_ADDR 0xf0000510

#define TYPE_MVME3100 MACHINE_TYPE_NAME("mvme3100")
#define MVME3100(obj) OBJECT_CHECK(MVME3100State, (obj), TYPE_MVME3100)
#define MVME3100_GET_CLASS(obj) \
    OBJECT_GET_CLASS(MVME3100Class, (obj), TYPE_MVME3100)
#define MVME3100_CLASS(klass) \
    OBJECT_CLASS_CHECK(MVME3100Class, (klass), TYPE_MVME3100)

#define E500_TSEC_OFFSET(N)     (0x24000 + (N) * 0x1000)

/* Complex Core Bus frequency */
#define CCB_FREQ (333333333u)

typedef struct mvme3100_info {
    const char *desc;
    uint32_t cpu_freq;
    uint32_t porpllsr;
    uint32_t ram_size;
} mvme3100_info;

typedef struct MVME3100Class {
    /*< private >*/
    MachineClass parent_class;
    /*< public >*/

    const mvme3100_info *info;
} MVME3100Class;

typedef struct MVME3100State {
    /*< private >*/
    MachineState parent_obj;
    /*< public >*/

    uint32_t load_address,
             entry_address;

    MemoryRegion ram;
} MVME3100State;


/* motload "global environment" variables */
static
const char *gev[] = {
        /* TODO: somehow snoop in slirp_instances to pick up IP config? */
        "mot-/dev/enet0-cipa=10.0.2.15",
        "mot-/dev/enet0-gipa=10.0.2.2",
        "mot-/dev/enet0-snma=255.255.255.0",
        "mot-/dev/enet0-sipa=10.0.2.2",
        /* RTEMS specific names for things motload doesn't have */
        "rtems-dns-server=10.0.2.3",
        "rtems-client-name=qemu",
        NULL,
};

/* Prepare Motorola Vital Product Data eeprom image.
 * Provided to bootloader for use as a default.
 *
 * Begins with constant "MOTLOAD" followed by variable length records
 * with a two byte header (ID code then body length in bytes).
 *
 * | ID | Length | body .... | repeated until ID=0xFF
 *
 * ID Codes:
 *  1 - Product ID (string)
 *  2 - Assembly # (string)
 *  3 - Serial # (string)
 *  5 - CPU Speed (Hz, 4 byte integer + 1 nil)
 *  6 - Bus Speed (Hz, 4 byte integer + 1 nil)
 *  8 - Ethernet MAC (6 bytes + 1 nil)
 *  9 - CPU type
 *  A - VPD CRC (4 bytes)
 *  B - Flash Config (??)
 *  E - L2 Cache Config (??)
 *  F - VPD Version (4 bytes)
 * 19 - L3 Cache Config (??)
 * FF - End of VPD (size zero)
 *
 * Repeat entries for repeated units.  eg. two ID=0x8 for two NICs
 *
 * MOTLoad uses the same eeprom to hold it's user configuration
 * Global Environment Variable (GEV) list.
 */
typedef struct vpdeeprom {
    char * const base;
    char *cur;
    size_t total;
} vpdeeprom;

static
void append_gev_vpd(vpdeeprom *vpd, const char *str)
{
    const size_t remaining = vpd->total - (vpd->cur - vpd->base),
                 len = strlen(str);

    if ((len == 0 && remaining < 1)
            || (remaining < len + 2))
    {
        fprintf(stderr, "VPD GEV overflow\n");
        return;
    }

    memcpy(vpd->cur, str, len + 1);

    vpd->cur += len + 1;
}

static
void append_vpd(vpdeeprom *vpd, uint8_t id, size_t cnt, const void *val)
{
    const size_t remaining = vpd->total - (vpd->cur - vpd->base);

    /* must have enough space for this entry and final ID=0xff */
    if ((id == 0xff && remaining < 2)
            || (remaining + 4 < cnt || cnt > 255))
    {
        fprintf(stderr, "VPD overflow\n");
        return;
    }

    vpd->cur[0] = id;
    vpd->cur[1] = cnt;
    memcpy(vpd->cur + 2, val, cnt);

    vpd->cur += 2 + cnt;
}

static
void append_string_vpd(vpdeeprom *vpd, uint8_t id, const char *str)
{
    /* include trailing nil */
    append_vpd(vpd, id, strlen(str) + 1, str);
}

static
void append_mac_vpd(vpdeeprom *vpd, uint8_t id, const MACAddr *addr)
{
    char buf[7];
    memcpy(buf, addr->a, 6);
    buf[6] = 0;

    append_vpd(vpd, id, 7, buf);
}

static
void append_u32_vpd(vpdeeprom *vpd, uint8_t id, uint32_t val)
{
    union {
        uint32_t ival;
        char bytes[5]; /* include trailing nil */
    } buf;
    buf.ival = cpu_to_be32(val);
    buf.bytes[4] = 0;

    append_vpd(vpd, id, 5, buf.bytes);
}

static
void build_vpd(const mvme3100_info *info, char *buf, size_t cnt,
               const char *extra)
{
    vpdeeprom vpd = {buf, buf, cnt};
    size_t i;

    memset(buf, 0, cnt);

    strcpy(buf, "MOTOROLA");
    vpd.cur += 8;

    /* Product ID (eg. "MVME3100-1152") */
    append_string_vpd(&vpd, 1, info->desc);

    /* serial number */
    append_string_vpd(&vpd, 3, "E0120000");

    /* CPU Freq. */
    append_u32_vpd(&vpd, 5, info->cpu_freq);

    /* PCI Bus Freq. */
    append_u32_vpd(&vpd, 6, 66666666);

    for (i = 0; i < MAX_NICS; i++) {
        if (nd_table[i].used) {
            append_mac_vpd(&vpd, 8, &nd_table[i].macaddr);
        }
    }

    append_vpd(&vpd, 0xff, 0, NULL);

    if (vpd.cur - vpd.base > 0x10f8) {
        fprintf(stderr, "VPD overflows GEV area.\n");
        return;
    }

    /* MOTLOAD's Global Environment Variables
     * start at offset 0x10f8.
     * This is a set of nil terminated strings of the form "name=value"
     * with a zero length string signaling the end.
     */
    vpd.cur = vpd.base + 0x10f8;

    for (i = 0; gev[i]; i++) {
        append_gev_vpd(&vpd, gev[i]);
    }

    if (extra) {
        char *E = g_strdup(extra);
        char **opts = g_strsplit(E, " ", 0);
        size_t i;

        g_free(E);

        for (i = 0; opts[i]; i++) {
            char *opt = g_strstrip(opts[i]);
            size_t olen = strlen(opt);

            if (olen == 0) {
                continue;
            } else if (!strchr(opt, '=')) {
                fprintf(stderr, "Missing '=' in -append %s\n", extra);
                continue;
            }

            append_gev_vpd(&vpd, opt);
        }

        g_strfreev(opts);
    }

    /* zero length string signals end */
    append_gev_vpd(&vpd, "");
}

static
void set_map(CPUPPCState *env, unsigned way,
             target_ulong va, hwaddr pa,
             unsigned size)
{
    ppcmas_tlb_t *tlb = booke206_get_tlbm(env, 1, 0, way);

    tlb->mas1 = MAS1_VALID | (size << MAS1_TSIZE_SHIFT);
    tlb->mas2 = (va & TARGET_PAGE_MASK) | MAS2_I | MAS2_G;
    tlb->mas7_3 = (pa & TARGET_PAGE_MASK) | MAS3_SR | MAS3_SW | MAS3_SX;
}

static
void remap_tlb_bare(CPUPPCState *env)
{
    /* The MPC8540 ref. manual says only the upper 4KB (ROM)
     * is mapped, but doesn't say exactly how this mapping
     * is setup.  So we arbitrarily decide to use TLB1 entry 0.
     */
    set_map(env, 0, 0xfffff000, 0xfffff000, 0x02);

    env->tlb_dirty = true;
}

static void mvme3100_cpu_reset(void *opaque)
{
    PowerPCCPU *cpu = opaque;
    CPUState *cs = CPU(cpu);
    CPUPPCState *env = &cpu->env;

    cpu_reset(cs);

    /* HID0 clock control functions not modeled.
     * Decrementer always enabled with CCB/8 as reference.
     * HID0[EMCP] and HID0[TBEN] set
     */
    env->spr[SPR_HID0] |= 0x80004000;

    remap_tlb_bare(&cpu->env);
    env->nip = 0xfffffffc;
}

static
void mvme3100_pci1_set_irq(void *opaque, int irq_num, int level)
{
    qemu_irq *pic = opaque;

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
 * The modeled PCI host bridge differs.
 *
 * We model one PCI-PCI bridge (0:0x12:0) but with a different vendor/device
 *
 * We don't model:
 * # The SATA controller GD31244 (0x8086-0x3200)
 * # The USB (OHCI) controller uPD740101 (0x1033-0x0035)
 * # The second PCI-PCI bridge (0:0x13:0) in front of the USB controllers
 *
 * (Note that the SATA controller found on newer boards is different)
 */

static void mvme3100_init(MachineState *machine)
{
    MVME3100State *mvme3100 = MVME3100(machine);
    const mvme3100_info *info;
    DeviceState *dev;
    BusState *i2c;
    PCIBus *pci0, *pci1;
    SysBusDevice *cpld, *pic;
    qemu_irq *pci1_pins = g_malloc_n(4, sizeof(*pci1_pins));
    FWCfgState *fwinfo;
    DriveInfo *drvinfo;
    MemoryRegion *ccsr;

    {
        MVME3100Class *klass = MVME3100_GET_CLASS(machine);
        info = klass->info;
    }

    /* Setup CPU */

    ppce500_init(machine, CCB_FREQ / 8u);

    memory_region_allocate_system_memory(&mvme3100->ram, NULL,
                                         "mvme3100.ram", ram_size);
    memory_region_add_subregion(get_system_memory(), 0, &mvme3100->ram);

    qemu_register_reset(mvme3100_cpu_reset, POWERPC_CPU(first_cpu));

    /* Create CCSR and builtin periphrials */
    dev = qdev_create(NULL, "e500-ccsr");
    object_property_add_child(qdev_get_machine(), "e500-ccsr",
                              OBJECT(dev), NULL);
    qdev_prop_set_uint32(dev, "ccb-freq", CCB_FREQ);
    qdev_prop_set_uint32(dev, "mpic-model", OPENPIC_MODEL_FSL_MPIC_20);
    qdev_prop_set_uint32(dev, "porpllsr", info->porpllsr);
    qdev_prop_set_uint32(dev, "base", 0xff700000ULL);
    qdev_prop_set_uint32(dev, "ram-size", ram_size);
    qdev_prop_set_uint32(dev, "pci_first_slot", 0);
    qdev_prop_set_uint32(dev, "pci_first_pin_irq", 1);
    qdev_init_nofail(dev);

    ccsr = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);

    pic = SYS_BUS_DEVICE(object_resolve_path("/machine/pic", NULL));

    /* Setup mvme3100 specific CPLD device */
    cpld = SYS_BUS_DEVICE(qdev_create(NULL, "mvme3100-cpld"));
    object_property_add_child(qdev_get_machine(), "cpld",
                              OBJECT(cpld), &error_fatal);
    qdev_init_nofail(DEVICE(cpld));

    memory_region_add_subregion(get_system_memory(),
                                0xe2000000,
                                sysbus_mmio_get_region(cpld, 0));

    fwinfo = fw_cfg_init_mem(CFG_ADDR, CFG_ADDR + 2);
    fw_cfg_add_i16(fwinfo, FW_CFG_NB_CPUS, 1);
    fw_cfg_add_i16(fwinfo, FW_CFG_MAX_CPUS, 1);
    fw_cfg_add_i64(fwinfo, FW_CFG_RAM_SIZE, machine->ram_size);

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

    pci_bus_irqs(pci1, mvme3100_pci1_set_irq,
                 pci_swizzle_map_irq_fn, pci1_pins, 4);

    /* the actual PLX bridge doesn't emit interrupts */
    pci_set_byte(PCI_DEVICE(dev)->config + PCI_INTERRUPT_PIN, 0);

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
               " (qemu built w/o TSI148 emulation)\n");
    }

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

    /* I2C Controller */
    dev = DEVICE(object_resolve_path("/machine/i2c[0]", NULL));
    assert(dev);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       qdev_get_gpio_in(DEVICE(pic), 16 + 27));
    i2c = qdev_get_child_bus(dev, "bus");
    assert(i2c);

    /* NIC (2x TSEC and 1x FEC) */
    if (nd_table[0].used) {
        qemu_check_nic_model(&nd_table[0], "eTSEC");

        dev = etsec_create(E500_TSEC_OFFSET(0), ccsr, &nd_table[0],
                qdev_get_gpio_in(DEVICE(pic), 16 + 13),
                qdev_get_gpio_in(DEVICE(pic), 16 + 14),
                qdev_get_gpio_in(DEVICE(pic), 16 + 18));

    } else if (nd_table[1].used) {
        qemu_check_nic_model(&nd_table[1], "eTSEC");

        dev = etsec_create(E500_TSEC_OFFSET(1), ccsr, &nd_table[1],
                qdev_get_gpio_in(DEVICE(pic), 16 + 19),
                qdev_get_gpio_in(DEVICE(pic), 16 + 20),
                qdev_get_gpio_in(DEVICE(pic), 16 + 23));

    } else if (nd_table[2].used) {
        qemu_log_mask(LOG_UNIMP, "FEC (ethernet #3) not modeled\n");
    }

    /* VPD EEPROM */
    dev = qdev_create(i2c, "at24c-eeprom");
    object_property_add_child(qdev_get_machine(), "vpd", OBJECT(dev),
                              &error_fatal);
    qdev_prop_set_uint8(dev, "address", 0xa8 >> 1);
    qdev_prop_set_uint32(dev, "rom-size", 8192 * 8);

    drvinfo = drive_get(IF_PFLASH, 0, 0);
    if (drvinfo) {
        qdev_prop_set_drive(dev, "drive", blk_by_legacy_dinfo(drvinfo),
                            &error_fatal);
    }

    qdev_init_nofail(dev);

    {
        char *buf;

        buf = g_malloc0(8192 * 8);

        build_vpd(info, buf, 8192 * 8, machine->kernel_cmdline);

        fw_cfg_add_file(fwinfo, "tomload/vpd", buf, 8192 * 8);
    }

    /* DS1375 RTC */
    dev = qdev_create(i2c, "ds1375");
    object_property_add_child(qdev_get_machine(), "rtc", OBJECT(dev),
                              &error_fatal);
    qdev_prop_set_uint8(dev, "address", 0xd0 >> 1);
    qdev_init_nofail(dev);
    qdev_connect_gpio_out(dev, 0, qdev_get_gpio_in(DEVICE(pic), 11));

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

    if (!qtest_enabled()) {
        MemoryRegion *rom = g_malloc0(sizeof(*rom));
        char *fullname = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);

        if (!fullname) {
            fprintf(stderr, "qemu: could not find bios file '%s'\n",
                    bios_name);
            exit(1);
        }

        memory_region_init_ram(rom, OBJECT(cpld), "rom",
                               0x800000, &error_fatal);

        memory_region_add_subregion(get_system_memory(),
                                    0xff800000, rom);

        /* BIOS == image in rom (last 8MB of address space).
         * Execution starts with the final word 0xfffffffc
         */
        int bsize = get_image_size(fullname);
        if (bsize != 8 * 1024 * 1024 ||
                -1 == load_image_targphys(fullname,
                                          0xff800000, 8 * 1024 * 1024))
        {
            fprintf(stderr, "qemu: could not load bios file '%s'"
                            " (%u bytes, requires 8MB)\n",
                    fullname, (unsigned)bsize);
            exit(1);
        }

        memory_region_set_readonly(rom, true);

        g_free(fullname);
    }

    {
        hwaddr image_addr = mvme3100->load_address;

        int image_size = load_image_targphys(machine->kernel_filename,
                                             image_addr, 0x01000000);
        if (machine->kernel_filename &&
                -1 == image_size)
        {
            fprintf(stderr, "qemu: could not load file '%s'\n",
                    machine->kernel_filename);
            exit(1);

        } else if (mvme3100->entry_address == 0) {
            mvme3100->entry_address = image_addr;

        } else if (mvme3100->entry_address < image_addr
                  || mvme3100->entry_address >= image_addr + image_size)
        {
            fprintf(stderr, "qemu: entry-address out of range\n");
            exit(1);
        }

        if (machine->kernel_cmdline) {
            fw_cfg_add_i32(fwinfo, FW_CFG_CMDLINE_SIZE,
                           strlen(machine->kernel_cmdline) + 1);
            fw_cfg_add_string(fwinfo, FW_CFG_CMDLINE_DATA,
                              machine->kernel_cmdline);
        }

        fw_cfg_add_i32(fwinfo, FW_CFG_KERNEL_ADDR, image_addr);
        fw_cfg_add_i32(fwinfo, FW_CFG_KERNEL_ENTRY, mvme3100->entry_address);
        fw_cfg_add_i32(fwinfo, FW_CFG_KERNEL_SIZE, image_size);
    }
}

static void mvme3100_inst_init(Object *obj)
{
    MVME3100State *mvme3100 = MVME3100(obj);
    mvme3100->load_address = 0x10000;
    mvme3100->entry_address = 0;
}

static void mvme3100_visit_addr(Object *obj,
                                Visitor *v,
                                const char *name,
                                void *opaque,
                                Error **errp)
{
    MVME3100State *mvme3100 = MVME3100(obj);
    uint32_t *ptr;

    if (strcmp(name, "load-address") == 0) {
        ptr = &mvme3100->load_address;
    } else if (strcmp(name, "entry-address") == 0) {
        ptr = &mvme3100->entry_address;
    } else {
        fprintf(stderr, "logic error: mvme3100 has no prop '%s'\n", name);
        exit(1);
    }

    visit_type_uint32(v, name, ptr, errp);
}

static void ppce500_machine_class_init(ObjectClass *klass, void *raw)
{
    mvme3100_info *info = raw;
    MachineClass *mc = MACHINE_CLASS(klass);
    MVME3100Class *m3c = MVME3100_CLASS(klass);

    m3c->info = info;

    mc->desc = info->desc;
    mc->init = mvme3100_init;
    mc->max_cpus = 1;
    mc->default_ram_size = info->ram_size;
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("mpc8540_v21");

    object_class_property_add(OBJECT_CLASS(mc), "load-address", "uint32",
                              &mvme3100_visit_addr, &mvme3100_visit_addr, NULL,
                              NULL, &error_fatal);
    object_class_property_add(OBJECT_CLASS(mc), "entry-address", "uint32",
                              &mvme3100_visit_addr, &mvme3100_visit_addr, NULL,
                              NULL, &error_fatal);
}

static const TypeInfo mvme3100_type = {
    .abstract = true,
    .name = TYPE_MVME3100,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(MVME3100State),
    .instance_init = mvme3100_inst_init,
    .class_size = sizeof(MVME3100Class),
};

static mvme3100_info mvme3100_1152 = {
    .desc = "MVME3100-1152",
    .cpu_freq = 666666666u,
    /* CCB/PCI  -> 5/1
     * core/CCB -> 2/1
     *
     * plat ratio = 5 -> 5:1 CCB:PCI
     * e500 ratio = 4 -> 4:1 e500:CCB
     */
    .porpllsr = 0x0004000a,
    .ram_size = 256 * (1 << 20),
};

static const TypeInfo mvme3100_1152_type = {
    .name = MACHINE_TYPE_NAME("mvme3100-1152"),
    .parent = TYPE_MVME3100,
    .class_init = ppce500_machine_class_init,
    .class_data = &mvme3100_1152,
};

static mvme3100_info mvme3100_1263 = {
    .desc = "MVME3100-1263",
    .cpu_freq = 833333333u,
    /* CCB/PCI  -> 5/1
     * core/CCB -> 5/2
     */
    .porpllsr = 0x0005000a,
    .ram_size = 512 * (1 << 20),
};

static const TypeInfo mvme3100_1263_type = {
    .name = MACHINE_TYPE_NAME("mvme3100-1263"),
    .parent = TYPE_MVME3100,
    .class_init = ppce500_machine_class_init,
    .class_data = &mvme3100_1263,
};

static void mvme3100_machine_init(void)
{
    type_register_static(&mvme3100_type);
    type_register_static(&mvme3100_1152_type);
    type_register_static(&mvme3100_1263_type);
}

type_init(mvme3100_machine_init)
