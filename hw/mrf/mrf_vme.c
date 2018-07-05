/*
 * QEMU MRF VME EVR/EVG emulation
 *
 * Michael Davisdaver
 * Copyright (c) 2017
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


#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/vme/vme_bus.h"
#include "net/net.h"
#include "net/checksum.h"
#include "hw/loader.h"
#include "sysemu/sysemu.h"
#include "chardev/char-fe.h"
#include "qemu/error-report.h"

#include "mrf_core.h"

//#define MRF_DEBUG

#define TYPE_MRF_VME "mrf-vme"

#ifdef MRF_DEBUG
#define	DBGOUT(fmt, ...) info_report(TYPE_MRF_VME ": %s() " fmt, __func__, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#endif

#define ERR(mask, fmt, ...)  qemu_log_mask(mask, TYPE_MRF_VME ": " fmt "\n", ## __VA_ARGS__)

#define NELEM(N) (sizeof(N)/sizeof((N)[0]))

#define MRF_VME(obj) OBJECT_CHECK(MRFVMEState, (obj), TYPE_MRF_VME)
#define MRF_VME_CLASS(klass) OBJECT_CLASS_CHECK(MRFVMEClass, klass, TYPE_MRF_VME)
#define MRF_VME_GET_CLASS(obj) OBJECT_GET_CLASS(MRFVMEClass, obj, TYPE_MRF_VME)

/* MRF specific CSR registers to configure VME interrupt signaling */
#define  UCSR_IRQ_LEVEL                0x7fb03
#define  UCSR_IRQ_VECTOR               0x7fb07

typedef struct MRFVMEState {
    VMEDevice parent_obj;

    CharBackend chr; /* event link */

    MemoryRegion cr, fn[3];
    uint8_t rom[0x80000>>2];

    BusState *sbus;
    MRFCore *core;

    uint8_t fwversion;

} MRFVMEState;

typedef struct {
    const char *core;
    const char *desc;
    uint32_t vendor_id;
    uint32_t board_id;
    uint32_t revision;
    uint8_t mrf_type;
} mrf_vme_info;

typedef struct MRFVMEClass {
    VMEDeviceClass parent_obj;

    const mrf_vme_info *info;
} MRFVMEClass;

static Property mrfvme_properties[] = {
    DEFINE_PROP_UINT8("version", MRFVMEState, fwversion, 6),
    DEFINE_PROP_CHR("chardev", MRFVMEState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static
void handle_vme_irq(void *opaque, int n, int level)
{
    MRFVMEState *d = opaque;
    uint8_t irqlevel  = d->rom[UCSR_IRQ_LEVEL>>2];

    DBGOUT("Assert IRQ%u = %d", irqlevel, level);

    if(irqlevel)
        vme_set_irq(VME(d), irqlevel, !!level);
}

static
bool handle_vme_iack(struct VMEDevice *dev, uint8_t lvl, uint32_t *pvect)
{
    MRFVMEState *d = MRF_VME(dev);
    uint8_t irqvector  = d->rom[UCSR_IRQ_VECTOR>>2];

    *pvect = irqvector;

    DBGOUT("IACK%u with 0x%"PRIx32, lvl, *pvect);
    return true;
}

static void mrf_csr_update(MRFVMEState *d)
{
    VMEDevice *dev = VME(d);
    unsigned i;

    for(i=0; i<3; i++) {
        /* base address 0xffffff00
         * modifier     0x000000fc  (shift >>2)
         */
        uint8_t amod = d->rom[(VME_CSR_FN_ADER(i)>>2)+3]>>2;
        uint32_t base = d->rom[(VME_CSR_FN_ADER(i)>>2)+0];
        base = (base<<8) | d->rom[(VME_CSR_FN_ADER(i)>>2)+1];
        base = (base<<8) | d->rom[(VME_CSR_FN_ADER(i)>>2)+2];
        base <<= 8;

        vme_del_region(dev, &d->fn[i]);

        if(amod) {
            DBGOUT("Map FN[%u] to %02x @%08x", i, amod, (unsigned)base);
            vme_add_region(VME(d), amod, base, &d->fn[i]);
        }
    }

    {
        /* Make sure we are asserting the correct interrupt, if any. */
        unsigned i;
        const uint8_t irqlevel  = dev->irq_status ? d->rom[0x03fb07>>2] : 0;
        for(i=1; i<=7; i++) {
            DBGOUT("Assert IRQ%u = %d", i, i==irqlevel);

            vme_set_irq(dev, i, i==irqlevel);
        }
    }
}

/* First 512 bytes of CR from an actual card
0000: 00000066 00000000 00000000 0000001f
0010: 00000081 00000081 00000002 00000043
0020: 00000052 00000000 0000000e 000000b2
0030: 00000045 00000052 00000046 000000e6
0040: 00000000 00000000 00000000 00000000
0050: 00000000 00000000 00000000 00000000
0060: 00000000 00000000 00000000 00000000
0070: 00000000 00000000 00000000 00000001
0080: 00000000 00000000 00000000 00000000
0090: 00000000 00000000 00000000 00000000
00a0: 00000000 00000000 00000000 00000000
00b0: 00000003 000000fb 00000007 000000ff
00c0: 000000fb 00000007 00000013 000000fb
00d0: 00000007 00000027 000000fb 00000007
00e0: 00000004 00000000 00000004 00000000
00f0: 00000000 000000fe 00000000 00000081
0100: 00000084 00000084 00000084 00000084
0110: 00000084 00000084 00000084 00000084
0120: 00000000 00000000 00000000 00000000
 ...
01f0: 00000000 00000000 00000000 00000000
*/

static uint64_t
mrf_csr_read(void *opaque, hwaddr rawaddr, unsigned size)
{
    /* VME64 CSR/CR addressing is weird.
     * Byte registers are given dword spacing, including multi-byte registers.
     * eg. a 4 byte register like ADER0 (address decoder 0)
     * is accessed through 0x7ff63, 0x7ff67, 0x7ff6b, and 0x7ff63f
     * with the first being the MSB.
     *
     * So we require that the two least bits of the address be set
     * then shift down to ignore them.
     */
    uint64_t ret;
    MRFVMEState *d = opaque;
    uint32_t addr = rawaddr;

    if((addr&3)!=3)
        return 0;
    addr>>=2;

    ret = d->rom[addr];

    DBGOUT("CSR read "TARGET_FMT_plx" -> %02x", rawaddr, (unsigned)ret);
    return ret;
}

static void
mrf_csr_write(void *opaque, hwaddr rawaddr, uint64_t val,
                 unsigned size)
{
    uint32_t addr = rawaddr;
    MRFVMEState *d = opaque;

    DBGOUT("CSR write "TARGET_FMT_plx" <- %02x", rawaddr, (unsigned)val);

    if((addr&3)!=3)
        return;

    switch(addr) {
    case VME_CSR_FN_ADER(0):
    case VME_CSR_FN_ADER(0)+4:
    case VME_CSR_FN_ADER(0)+8:
    case VME_CSR_FN_ADER(0)+12:
    case VME_CSR_FN_ADER(1):
    case VME_CSR_FN_ADER(1)+4:
    case VME_CSR_FN_ADER(1)+8:
    case VME_CSR_FN_ADER(1)+12:
    case VME_CSR_FN_ADER(2):
    case VME_CSR_FN_ADER(2)+4:
    case VME_CSR_FN_ADER(2)+8:
    case VME_CSR_FN_ADER(2)+12:
    case UCSR_IRQ_VECTOR: /* MRF IRQ vector */
        break;
    case UCSR_IRQ_LEVEL: /* MRF IRQ level */
        val &= 7;
        break;
    default:
        ERR(LOG_UNIMP|LOG_GUEST_ERROR, "Write to unsupported CSR/CR 0x%06x",(unsigned)addr);
        return;
    }

    d->rom[addr>>2] = val;
    mrf_csr_update(d);
}

static const MemoryRegionOps mrf_csr_ops = {
    .read = mrf_csr_read,
    .write = mrf_csr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void mrf_vme_realize(struct VMEDevice *dev, Error **errp)
{
    MRFVMEState *d = MRF_VME(dev);
    MRFVMEClass *k = MRF_VME_GET_CLASS(dev);
    DeviceState *coredev;

    Chardev *chr = qemu_chr_fe_get_driver(&d->chr);

    assert(k->info);

    d->sbus = qbus_create(TYPE_MRF_BUS, DEVICE(dev), "local");

    coredev = qdev_create(d->sbus, k->info->core);
    d->core = MRF_CORE(coredev);

    object_property_add_child(OBJECT(dev), "core", OBJECT(coredev), &error_abort);

    DBGOUT("core is %s", object_get_canonical_path(OBJECT(coredev)));

    qdev_prop_set_uint8(coredev, "mrf-type", k->info->mrf_type);
    qdev_prop_set_uint8(coredev, "version", d->fwversion);

    if(chr) {
        /* transfer the chardev to the core device */
        qemu_chr_fe_deinit(&d->chr, false);
        qdev_prop_set_chr(DEVICE(coredev), "chardev", chr);
    }

    qdev_prop_set_uint8(DEVICE(coredev), "bridge-type", 2);
    qdev_init_nofail(DEVICE(coredev));

    memory_region_init_io(&d->cr, OBJECT(dev), &mrf_csr_ops, d, "mrf-csr", 0x80000);

    /* We can map core registers into 3 different regions.
     * each has a fixed size, so not registers will be accessible
     */
    memory_region_init_alias(&d->fn[0], OBJECT(dev), "mrf-core16",
            &d->core->core, 0, 0x800);
    memory_region_init_alias(&d->fn[1], OBJECT(dev), "mrf-core24",
            &d->core->core, 0, 0x10000);
    memory_region_init_alias(&d->fn[2], OBJECT(dev), "mrf-core32",
            &d->core->core, 0, 0x40000);

    if(dev->slot) {
        vme_add_region(dev, VME_AM_CRCSR, dev->slot<<19, &d->cr);
    } else {
        ERR(LOG_GUEST_ERROR, "No slot defined, CSR/CR not mapped.  No so useful...");
    }

    dev->iack = &handle_vme_iack;

    qdev_init_gpio_in_named(DEVICE(dev), &handle_vme_irq, "RAWIRQ", 1);
    qdev_connect_gpio_out(DEVICE(coredev), 0, qdev_get_gpio_in_named(DEVICE(dev), "RAWIRQ", 0));
}

static void mrf_vme_reset(DeviceState *dev)
{
    MRFVMEState *d = MRF_VME(dev);
    MRFVMEClass *c = MRF_VME_GET_CLASS(d);

    memset(d->rom, 0, sizeof(d->rom));

    d->rom[VME_CR_ASCII_C>>2] = 'C';
    d->rom[VME_CR_ASCII_R>>2] = 'R';
    d->rom[VME_CR_SPACE_ID>>2] = VME_CR_SPACE_ID_VME64x;
    d->rom[VME_CR_IRQ_CAP>>2] = 0xfe; /* all 7 lines */

    d->rom[(VME_CR_ROM_LENGTH>>2)+2] = 0x1f;

    /* Vendor ID    : 0x000eb2 */
    d->rom[(VME_CR_IEEE_OUI>>2)+0] = c->info->vendor_id>>16;
    d->rom[(VME_CR_IEEE_OUI>>2)+1] = c->info->vendor_id>>8;
    d->rom[(VME_CR_IEEE_OUI>>2)+2] = c->info->vendor_id>>0;

    /* Board ID     : 0x455246e6 */
    d->rom[(VME_CR_BOARD_ID>>2)+0] = c->info->board_id>>24;
    d->rom[(VME_CR_BOARD_ID>>2)+1] = c->info->board_id>>16;
    d->rom[(VME_CR_BOARD_ID>>2)+2] = c->info->board_id>>8;
    d->rom[(VME_CR_BOARD_ID>>2)+3] = c->info->board_id>>0;

    /* UCSR offset  : 0x7fb03
     * MRF violates spec. by storing in LSB
     */
    d->rom[(VME_CR_BEG_UCSR>>2)+0] = 0x03;
    d->rom[(VME_CR_BEG_UCSR>>2)+1] = 0xfb;
    d->rom[(VME_CR_BEG_UCSR>>2)+2] = 0x07;

    /* MSB of CSR address */
    d->rom[VME_CSR_BAR>>2] = VME(dev)->slot<<3;

    /* TODO: track SYSFAIL and BERR */
    d->rom[VME_CSR_BIT_SET>>2] = VME_CSR_BIT_MODULE_ENA;

    mrf_csr_update(d);
}

static void mrf_vme_class_init(ObjectClass *klass, void *data)
{
    mrf_vme_info *info = data;
    DeviceClass *dc = DEVICE_CLASS(klass);
    VMEDeviceClass *k = VME_CLASS(klass);
    MRFVMEClass *mk = MRF_VME_CLASS(klass);

    assert(info);

    mk->info = info;

    k->realize = mrf_vme_realize;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->desc = info->desc;
    dc->reset = mrf_vme_reset;
    dc->props = mrfvme_properties;
}

static const TypeInfo mrf_vme_base_info = {
    .name          = TYPE_MRF_VME,
    .parent        = TYPE_VME,
    .instance_size = sizeof(MRFVMEState),
    .class_size    = sizeof(MRFVMEClass),
    .abstract      = true,
};

static mrf_vme_info vme_evr_230_info = {
    .core = "mrf-evr",
    .desc = "Micro Research VME-EVR-230",
    .vendor_id = 0x000eb2,
    .board_id = 0x455246e6,
    .revision = 0,
    .mrf_type = 0x12,
};

static const TypeInfo mrf_evr_230_info = {
    .name          = "vme-evr-230",
    .parent        = TYPE_MRF_VME,
    .class_init    = mrf_vme_class_init,
    .class_data    = &vme_evr_230_info,
};

static mrf_vme_info vme_evg_230_info = {
    .core = "mrf-evg",
    .desc = "Micro Research VME-EVG-230",
    .vendor_id = 0x000eb2,
    .board_id = 0x454700e6,
    .revision = 0,
    .mrf_type = 0x22,
};

static const TypeInfo mrf_evg_230_info = {
    .name          = "vme-evg-230",
    .parent        = TYPE_MRF_VME,
    .class_init    = mrf_vme_class_init,
    .class_data    = &vme_evg_230_info,
};

static mrf_vme_info vme_evm_300_info = {
    .core = "mrf-evm",
    .desc = "Micro Research VME-EVM-300",
    .vendor_id = 0x000eb2,
    .board_id = 0x4547012c,
    .revision = 0,
    .mrf_type = 0x28,
};

static const TypeInfo mrf_evm_300_info = {
    .name          = "vme-evm-300",
    .parent        = TYPE_MRF_VME,
    .class_init    = mrf_vme_class_init,
    .class_data    = &vme_evm_300_info,
};

static void mrf_vme_register_types(void) {
    type_register_static(&mrf_vme_base_info);
    type_register_static(&mrf_evr_230_info);
    type_register_static(&mrf_evg_230_info);
    type_register_static(&mrf_evm_300_info);
}

type_init(mrf_vme_register_types)
