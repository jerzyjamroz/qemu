/*
 * Tundra TSI148 PCI to VME bus bridge emulation
 *
 * Copyright (c) 2016 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
/* The TSI148 is an odd-ball PCI device in several ways.
 *
 * Can use all 4 PCI IRQ pins.
 * Decodes variable size address ranges outside the usual BARs.
 * Can talk to itself through VME (PCI -> VME -> internal registers)
 *
 * The "outbound" (PCI -> VME) windows are modeled.
 * The "inbound" (VME -> PCI) windows are not.
 *
 * Handling of VME interrupts and Bus error are implemented.
 *
 * There are also two scatter/gather DMA engines capable of
 * arbitrary transfers between PCI and VME buses (including
 * PCI<->PCI and VME<->VME).
 *
 * Oh, and it does automatic byte order swapping on all reads/writes
 * including DMA.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/registerfields.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/vme/vme_bus.h"
#include "qemu/error-report.h"

#define TYPE_TSI148 "tsi148"

#define TSI148(obj) OBJECT_CHECK(TSI148State, (obj), TYPE_TSI148)

/* #define TSI148_DEBUG 1 */

#ifdef TSI148_DEBUG
#define DBGOUT(fmt, ...) info_report(TYPE_TSI148 ": " fmt, ## __VA_ARGS__)
#else
#define DBGOUT(fmt, ...) do {} while (0)
#endif

#define LOG(mask, fmt, ...) qemu_log_mask(mask, fmt, ##__VA_ARGS__)

#define CRG_SIZE 0x1000

#define NUM_OUT 8

#define TSI_PAGE_MASK (0xffff)

/* Ok, there are a lot of registers here, which can be
 * accessed from PCI and/or VME address spaces in many and varied ways.
 *
 * Device Control Registers Group (CRG) can be accessed with PCI BAR 0,
 * the CRG VME Window (A12, A24, A32, or A64), and in VME CR/CSR range
 *
 * Groups with CRG offset ranges
 *   PCFS - 0x0000 -> 0x00ff Device PCI config registers
 *   LCSR - 0x0100 -> 0x05c7 Out/In windows, interrupts, DMA engine
 *   GCSR - 0x0600 -> 0x061f
 *   CR/CSR 0x0ff4 -> 0x0fff set/clear and base address
 *
 * We track offsets in the CRG unless otherwise noted.
 *
 * Some unimplemented registers are not listed,
 * and some listed registers are unimplemented.
 */

/* Output windows (PCI -> VME) */

/* Output window PCI start */
#define R_OTSAU(N) (0x100 + (N) * 0x20)
#define R_OTSAL(N) (0x104 + (N) * 0x20)
/* Output window PCI End */
#define R_OTEAU(N) (0x108 + (N) * 0x20)
#define R_OTEAL(N) (0x10c + (N) * 0x20)
/* Output window VME Offset */
#define R_OTOFU(N) (0x110 + (N) * 0x20)
#define R_OTOFL(N) (0x114 + (N) * 0x20)
/* Output window 2eSST broadcast select */
#define R_OTBS(N)  (0x118 + (N) * 0x20)
/* Output window attributes */
#define R_OTAT(N)  (0x11c + (N) * 0x20)

FIELD(OTAT, ENA, 31, 1)
/* memory prefetch, and 2eSST not modeled */
FIELD(OTAT, TM, 8, 3)
FIELD(OTAT, DBW, 6, 2)
FIELD(OTAT, SUP, 5, 1)
FIELD(OTAT, PGM, 4, 1)
FIELD(OTAT, AM, 0, 4)

/* VME IRQ acknowledge. (read width selects IACK width)  */
#define R_VIACK(N)    (0x204 + (N) * 0x04)

/* general control/status */
#define R_VMCTRL (0x234)
FIELD(VMCTRL, VSA, 27, 1)
FIELD(VMCTRL, VS, 26, 1)
#define R_VCTRL  (0x238)
#define R_VSTAT  (0x23c)

#define R_VMEFL  (0x250)

/* VME exception address */
#define R_VEAU   (0x260)
#define R_VEAL   (0x264)
/* VME exception attributes */
#define R_VEAT   (0x268)
#define R_VEAT_VES   (1u << 31)
#define R_VEAT_VEOF  (1u << 30)
#define R_VEAT_VESCL (1u << 29)
#define R_VEAT_VERR  (1u << 19)
#define R_VEAT_WRITE  (1u << 17)

/* Inbound windows (VME -> PCI) */

/* Inbound window PCI start */
#define R_ITSAU(N) (0x300 + (N) * 0x20)
#define R_ITSAL(N) (0x304 + (N) * 0x20)
/* Inbound window PCI End */
#define R_ITEAU(N) (0x308 + (N) * 0x20)
#define R_ITEAL(N) (0x30c + (N) * 0x20)
/* Inbound window VME Offset */
#define R_ITOFU(N) (0x310 + (N) * 0x20)
#define R_ITOFL(N) (0x314 + (N) * 0x20)
/* Inbound window attributes */
#define R_ITAT(N)  (0x318 + (N) * 0x20)

/* GCSR VME base (VME -> internal registers) */
#define R_GBAU   (0x400)
#define R_GBAL   (0x404)
#define R_GCSRAT (0x408)

/* CRG VME base (VME -> internal registers) */
#define R_CBAU   (0x40c)
#define R_CBAL   (0x410)
#define R_CRGAT  (0x414)

FIELD(CRGAT, ENA, 7, 1)
FIELD(CRGAT, AS, 4, 3)
FIELD(CRGAT, SUPR, 3, 1)
FIELD(CRGAT, NPRIV, 2, 1)
FIELD(CRGAT, PGM, 1, 1)
FIELD(CRGAT, DATA, 0, 1)

/* CR/CSR offset (VME -> internal registers) */
#define R_CROU   (0x418)
#define R_CROL   (0x41c)
#define R_CRAT   (0x420)

/* Interrupt controller */
#define R_VICR     (0x440)
/* Enable */
#define R_INTEN    (0x448)
/* Out Enable */
#define R_INTEO    (0x44c)
/* Status */
#define R_INTS     (0x450)
/* Clear */
#define R_INTC     (0x454)

#define R_CBAR     (0xffc)

/* mask applies to INTEN, INTEO, INTS, and INTC */
#define R_INT_DMA1  (1u << 25)
#define R_INT_DMA0  (1u << 24)
#define R_INT_PERR  (1u << 13)
#define R_INT_VERR  (1u << 12)
#define R_INT_VIE   (1u << 11)
#define R_INT_IACK  (1u << 10)
/* N in [1,7] */
#define R_INT_IRQ(N) (1u << (N))

/* Mapping for DMA, LM, and MB. 2 bits per source */
#define R_INTM1    (0x458)
#define R_INTM1_DMA1_S  (18)
#define R_INTM1_DMA0_S  (16)
/* Mapping for VME IRQs */
#define R_INTM2    (0x45c)
#define R_INTM2_PERR_S (26)
#define R_INTM2_VERR_S (24)
#define R_INTM2_VIE_S  (22)
#define R_INTM2_IACK_S (20)
#define R_INTM2_IRQ_S(N) ((N) * 2u)

/* DMA engines */

/* Control */
#define R_DCTL(N)  (0x500 + (N) * 0x80)
/* Status */
#define R_DSTA(N)  (0x504 + (N) * 0x80)
/* Current Source */
#define R_DCSAU(N) (0x508 + (N) * 0x80)
#define R_DCSAL(N) (0x50c + (N) * 0x80)
/* Current Dest */
#define R_DCDAU(N) (0x510 + (N) * 0x80)
#define R_DCDAL(N) (0x514 + (N) * 0x80)
/* Current Link */
#define R_DCLAU(N) (0x518 + (N) * 0x80)
#define R_DCLAL(N) (0x51c + (N) * 0x80)
/* Source */
#define R_DSAU(N)  (0x520 + (N) * 0x80)
#define R_DSAL(N)  (0x524 + (N) * 0x80)
/* Dest */
#define R_DDAU(N)  (0x528 + (N) * 0x80)
#define R_DDAL(N)  (0x52c + (N) * 0x80)
/* Source attrib */
#define R_DSAT(N)  (0x530 + (N) * 0x80)
/* Dest attrib */
#define R_DDAT(N)  (0x534 + (N) * 0x80)
/* Next Link */
#define R_DNLAU(N) (0x538 + (N) * 0x80)
#define R_DNLAL(N) (0x53c + (N) * 0x80)
/* Count */
#define R_DCNT(N)  (0x540 + (N) * 0x80)
/* Dest broadcast */
#define R_DDBS(N)  (0x544 + (N) * 0x80)

#define R_GCTRL    (0x604)

typedef struct TSI148State TSI148State;

typedef struct {
    TSI148State *tsi;
    unsigned int idx;
    MemoryRegion region;
} OutRegion;

struct TSI148State {
    PCIDevice parent_obj;

    uint8_t slot; /* slot to which this bus controller is attached */

    VMEBus vbus;
    VMEDevice vdev;

    MemoryRegion crg, /* full CRG region */
                 crg_pci, /* PCFS PCI config space from CRG */
                 crg_iack, /* overlay on CRG for VIACK */
                 crg_crcsr; /* CRG viewed from VME CR/CSR */
    MemoryRegion crgwin; /* CRG viewed from VME not CR/CSR */

    qemu_irq irq[4];

    uint32_t reg[CRG_SIZE >> 2]; /* CRG registers */

    bool enable;
    bool lreset;
    OutRegion outbound[NUM_OUT];
};

#define REG(ADDR) reg[(ADDR) >> 2]

#define BUILD64(U, L) (((uint64_t)U) << 32u | (L))

static inline
uint64_t tsi_read64(TSI148State *tsi, uint32_t addru)
{
    uint64_t val = tsi->REG(addru);
    val <<= 32;
    val |= tsi->REG(addru + 4u);
    return val;
}

/* PCI config space for this device may be mapped into a VME space */

static
uint64_t tsi148_conf_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIDevice *pci = opaque;
    uint64_t val;
    assert(addr < PCI_CONFIG_SPACE_SIZE);
    val = pci->config_read(pci, addr, size);
    DBGOUT("read config "TARGET_FMT_plx" -> %08x", addr, (unsigned)val);
    return val;
}

static
void tsi148_conf_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    PCIDevice *pci = opaque;
    DBGOUT("write config "TARGET_FMT_plx" <- %08x", addr, (unsigned)val);
    pci->config_write(pci, addr, val, size);
}

static const MemoryRegionOps crg_pci_ops = { /* PCFS */
    .read = tsi148_conf_read,
    .write = tsi148_conf_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

/* Call after updating IRQ status or configuration
 */
static
void update_irq(TSI148State *tsi)
{
    unsigned i;
    uint64_t map = tsi_read64(tsi, R_INTM1);
    uint32_t ena = tsi->REG(R_INTEN),
             out = tsi->REG(R_INTEO),
             sts = tsi->REG(R_INTS),
             act = ena & out & sts && tsi->enable;
    char dbglvl[5];
    bool level[4] = {0, 0, 0, 0};

    /* bit masks or ena, out, and sts are the same.
     * bit mask for map is twice the size (2 bits for each
     * IRQ source) in the same order.
     */
    for (i = 0; i < 32; i++, act >>= 1, map >>= 2) {
        if (act & 1) {
            level[map & 3] = 1;
        }
    }

    for (i = 0; i < 4; i++) {
        dbglvl[i] = level[i] ? 'X' : '_';
        qemu_set_irq(tsi->irq[i], level[i]);
    }

    dbglvl[4] = '\0';
    (void)dbglvl;
    DBGOUT("IRQ ACT %08x ENA %08x OUT %08x STS %08x %s",
           (unsigned)act, (unsigned)ena, (unsigned)out, (unsigned)sts, dbglvl);
}

/* VME Interrupt pin handler */
static
void vme_irq(void *opaque, int n, int level)
{
    TSI148State *tsi = opaque;
    uint32_t mask = R_INT_IRQ(++n);

    DBGOUT("VME%u %sassert", n,
           level ? "" : "de");

    assert(n >= 1 && n <= 7);

    if (level) {
        tsi->REG(R_INTS) |= mask;
    } else {
        tsi->REG(R_INTS) &= ~mask;
    }
    update_irq(tsi);
}

/* VME Bus Error handler.  Called by VMEBridge for failed read/write */
static
void tsi148_berr(VMEBus *vbus, void *opaque)
{
    TSI148State *tsi = opaque;
    uint32_t status = R_VEAT_VES | R_VEAT_VERR;

    DBGOUT("BERR %02x @ %08x",
           (unsigned)tsi->vbus.berr.amod,
           (unsigned)tsi->vbus.berr.addr);

    if (tsi->REG(R_VEAT) & R_VEAT_VES) {
        DBGOUT("BERR overflow");
        tsi->REG(R_VEAT) |= R_VEAT_VEOF; /* overflow */
        return;
    }

    status |= tsi->vbus.berr.write ? R_VEAT_WRITE : 0;

    /* TODO: LWORD, DS0, DS1 */

    status |= tsi->vbus.berr.amod << 8;

    tsi->REG(R_VEAU) = tsi->vbus.berr.addr >> 32;
    tsi->REG(R_VEAL) = tsi->vbus.berr.addr;
    tsi->REG(R_VEAT) = status;

    tsi->REG(R_INTS) |= R_INT_VERR;
    update_irq(tsi);
}

/* calculate address modifier from consitituents */
static
bool translate_amod(uint8_t *out, uint8_t amod,
                    bool sup, bool pgm, uint8_t mode)
{
    uint8_t ret;
    switch (amod) {
    case 0:
        ret = VME_AM_MSK_A16;
        break;
    case 1:
        ret = VME_AM_MSK_A24;
        break;
    case 2:
        ret = VME_AM_MSK_A32;
        break;
    case 4:
        /* ret = VME_AM_MSK_A64; break; */
        LOG(LOG_UNIMP, "64-bit AS not implemented\n");
        return false;
    case 5:
        *out = VME_AM_CRCSR;
        return true;
    default:
        LOG(LOG_GUEST_ERROR, "Invalid AS 0x%x\n", amod);
        return false;
    }

    switch (mode) {
    case 0:
        ret |= pgm ? VME_AM_MSK_PRG : VME_AM_MSK_DAT;
        break;
    case 1:
        ret |= VME_AM_MSK_BLT;
        break;
    case 2:
        ret |= VME_AM_MSK_MBLT;
        break;
    default:
        LOG(LOG_GUEST_ERROR, "Invalid data width 0x%x\n", mode);
        return false;
    }

    ret |= sup ? VME_AM_BIT_SUP : 0;

    *out = ret;
    return true;
}

/* Call after changes to output window N config */
static
void out_window_setup(TSI148State *tsi, unsigned N)
{
    MemoryRegion *pcibus = pci_address_space(PCI_DEVICE(tsi)), *vmespace;
    OutRegion *OR = &tsi->outbound[N];
    uint64_t pci_start = tsi_read64(tsi, R_OTSAU(N)),
             pci_end   = tsi_read64(tsi, R_OTEAU(N)),
             vme_off   = tsi_read64(tsi, R_OTOFU(N)),
             vme_start;
    const uint32_t otat = tsi->REG(R_OTAT(N));
    bool enabled = !!(tsi->enable && FIELD_EX32(otat, OTAT, ENA)),
         changed = enabled ^ OR->region.enabled;
    uint8_t amod;

    if (memory_region_is_mapped(&OR->region)) {
        memory_region_del_subregion(pcibus, &OR->region);
        object_unparent(OBJECT(&OR->region));
    }

    if (!enabled) {
        if (changed) {
            DBGOUT("Window %u disable", N);
        }
        return;
    }

    if (pci_start & TSI_PAGE_MASK) {
        LOG(LOG_GUEST_ERROR, "Window %u start %08x not aligned\n",
            N, (unsigned)pci_start);
        /* round up to next page */
        pci_start = tsi->REG(R_OTSAL(N)) = (pci_start | TSI_PAGE_MASK) + 1;
    }
    if ((pci_end & TSI_PAGE_MASK) != TSI_PAGE_MASK) {
        LOG(LOG_GUEST_ERROR, "Window %u end %08x not aligned\n",
            N, (unsigned)pci_end);
        /* round down to page start */
        pci_end = tsi->REG(R_OTEAL(N)) = pci_end & ~TSI_PAGE_MASK;
    }
    if (vme_off & TSI_PAGE_MASK) {
        LOG(LOG_GUEST_ERROR, "Window %u offset %08x not aligned\n",
            N, (unsigned)vme_off);
        /* round down */
        vme_off = tsi->REG(R_OTOFL(N)) = vme_off & ~TSI_PAGE_MASK;
    }

    /* HW comparison is a range test [pci_start, pci_end] inclusive
     * on address bits 63:16.
     */
    pci_start &= ~TSI_PAGE_MASK;
    pci_end   |=  TSI_PAGE_MASK;
    vme_off   &= ~TSI_PAGE_MASK;

    vme_start = pci_start + vme_off;

    if (pci_end < pci_start) {
        pci_end = pci_start;
        if (enabled) {
            LOG(LOG_UNIMP | LOG_GUEST_ERROR,
                "Outbound window wrap around not implemented.  Treating as zero length\n");
        }
    }

    if (!translate_amod(&amod,
                        FIELD_EX32(otat, OTAT, AM),
                        FIELD_EX32(otat, OTAT, SUP),
                        FIELD_EX32(otat, OTAT, PGM),
                        FIELD_EX32(otat, OTAT, TM)))
    {
        return;
    }

    vmespace = vme_bus_get_region(&tsi->vbus, amod);
    if (!vmespace) {
        LOG(LOG_UNIMP, "Unimplemented address space\n");
        return;
    }

    gchar *wname = g_strdup_printf("tsi-out%u", N);

    memory_region_init_alias(&OR->region, OBJECT(tsi),
                             wname ? wname : "tsi-outX-X",
                             vmespace, vme_start, pci_end - pci_start + 1);
    memory_region_set_enabled(&OR->region, enabled);
    memory_region_add_subregion(pcibus, pci_start, &OR->region);

    g_free(wname);

    if (changed) {
        DBGOUT("Window %u enable start=%08x end=%08x offset=%08x amod=%s",
               N, (unsigned)pci_start, (unsigned)pci_end, (unsigned)vme_start,
               vmespace->name);
        /* mtree_info(fprintf, stdout, true); */
    }
}

/* The CRG window can appear simultaniously in an number
 * of different VME address spaces.  We only support
 * mapping to one at a time.
 * This means only one of SUPR or NPRIV can be set.
 * Same for PGM or DATA.
 */
static
void crg_window(TSI148State *tsi)
{
    MemoryRegion *reg;
    uint8_t amod = 0;
    const uint32_t crgat = tsi->REG(R_CRGAT);
    hwaddr off = tsi->REG(R_CBAU);

    off <<= 32;
    off |= tsi->REG(R_CBAL);

    if (memory_region_is_mapped(&tsi->crgwin)) {
        memory_region_del_subregion(tsi->crgwin.container, &tsi->crgwin);
    }

    /* enabled if ENA set, and one of SUPR/NPRIV and one of PGM/DATA */
    if (!FIELD_EX32(crgat, CRGAT, ENA) || (crgat & 0xc) == 0
        || (crgat & 0x3) == 0) {
        return;
    }
    if (crgat & 0xc) {
        LOG(LOG_UNIMP, "Map CRG to both sup and non-priv AMs not implemented."
                       "  SUPR only.\n");
    }
    if (crgat & 0x3) {
        LOG(LOG_UNIMP, "Map CRG to both DAT and PGM not implemented."
                       "  PGM only.\n");
    }

    if (!translate_amod(&amod, FIELD_EX32(crgat, CRGAT, AS),
                        FIELD_EX32(crgat, CRGAT, SUPR),
                        FIELD_EX32(crgat, CRGAT, PGM), 0)) {
        LOG(LOG_GUEST_ERROR, "CRG window attribute, incorrect "
                             "space %08x\n", (unsigned)crgat);
        return;
    }

    reg = vme_bus_get_region(&tsi->vbus, amod);
    if (!reg) {
        LOG(LOG_GUEST_ERROR | LOG_UNIMP,
            "CRG window attribute, incorrect "
            "space %08x (may be unimplemented\n", (unsigned)crgat);
        return;
    }

    memory_region_set_enabled(&tsi->crgwin, tsi->enable);

    memory_region_add_subregion(reg, off, &tsi->crgwin);
    DBGOUT("CRG window enabled %s # %08x", reg->name, (unsigned)off);
}

struct dma_desc {
    uint64_t src, dst, pnext;
    uint32_t sattr, dattr, cnt, bcast;
};
typedef struct dma_desc dma_desc;

static
bool dma_read_desc(TSI148State *tsi, dma_desc *desc, uint64_t link)
{
    uint32_t raw[10];
    bool ret;

    if (desc->pnext & 6) {
        LOG(LOG_GUEST_ERROR, "Link pointer not aligned "
            TARGET_FMT_plx "\n", link);
        return false;
    }

    ret = dma_memory_rw(pci_get_address_space(&tsi->parent_obj),
                        link, raw, sizeof(raw),
                        DMA_DIRECTION_TO_DEVICE);

    if (ret) {
        LOG(LOG_GUEST_ERROR, "Failed to read link descriptor from "
            TARGET_FMT_plx "\n", link);
        return false;
    }

    desc->src    = ((uint64_t)be32_to_cpu(raw[0])) << 32;
    desc->src   |= be32_to_cpu(raw[1]);
    desc->dst    = ((uint64_t)be32_to_cpu(raw[2])) << 32;
    desc->dst   |= be32_to_cpu(raw[3]);
    desc->sattr  = be32_to_cpu(raw[4]);
    desc->dattr  = be32_to_cpu(raw[5]);
    desc->pnext  = ((uint64_t)be32_to_cpu(raw[6])) << 32;
    desc->pnext |= be32_to_cpu(raw[7]);
    desc->cnt    = be32_to_cpu(raw[8]);
    desc->bcast  = be32_to_cpu(raw[5]);

    DBGOUT(" Next Desc src %016llx (%08x) dst %016llx (%08x) cnt %08x"
           " next=%016llx",
           (unsigned long long)desc->dst, (unsigned)desc->dattr,
           (unsigned long long)desc->src, (unsigned)desc->sattr,
           (unsigned)desc->cnt,
           (unsigned long long)desc->pnext);

    return true;
}

static
void dma_write_desc(TSI148State *tsi, unsigned eng, uint64_t link,
                    const dma_desc *desc)
{
    DBGOUT(" Current Desc src %016llx (%08x) dst %016llx (%08x) cnt %08x"
           " next=%016llx",
           (unsigned long long)desc->src, (unsigned)desc->sattr,
           (unsigned long long)desc->dst, (unsigned)desc->dattr,
           (unsigned)desc->cnt,
           (unsigned long long)desc->pnext);
    tsi->REG(R_DCSAU(eng)) = desc->src >> 32;
    tsi->REG(R_DCSAL(eng)) = desc->src;
    tsi->REG(R_DCDAU(eng)) = desc->dst >> 32;
    tsi->REG(R_DCDAL(eng)) = desc->dst;
    tsi->REG(R_DCLAU(eng)) = link >> 32;
    tsi->REG(R_DCLAL(eng)) = link;
    tsi->REG(R_DSAT(eng))  = desc->sattr;
    tsi->REG(R_DDAT(eng))  = desc->dattr;
    tsi->REG(R_DNLAU(eng)) = desc->pnext >> 32;
    tsi->REG(R_DNLAL(eng)) = desc->pnext;
    tsi->REG(R_DCNT(eng))  = desc->cnt;
    tsi->REG(R_DDBS(eng))  = desc->bcast;
}

static
AddressSpace *dma_get_vme_space(TSI148State *tsi, uint32_t attr)
{
    uint8_t amod = 0xff;

    /* TODO: data width?  Might just work since everything is Native order */

    if (!translate_amod(&amod, extract32(attr, 0, 4),
                        extract32(attr, 5, 1),
                        extract32(attr, 4, 1),
                        extract32(attr, 8, 3)))
    {
        return NULL;
    }

    return vme_bus_get_space(&tsi->vbus, amod);
}

static
bool dma_do_desc(TSI148State *tsi, dma_desc *desc)
{
    char buf[128];
    AddressSpace *asrc, *adst;

    if ((desc->sattr | desc->dattr) & (1 << 29)) {
        LOG(LOG_UNIMP, "DMA data pattern mode not implemented\n");
        return false;
    }

    if (desc->sattr & (1 << 28)) {
        asrc = dma_get_vme_space(tsi, desc->sattr);
    } else {
        asrc = pci_get_address_space(&tsi->parent_obj);
    }
    if (desc->dattr & (1 << 28)) {
        adst = dma_get_vme_space(tsi, desc->dattr);
    } else {
        adst = pci_get_address_space(&tsi->parent_obj);
    }

    if (!asrc || !adst) {
        return false;
    }

    DBGOUT("  EXEC %s -> %s", asrc->name, adst->name);

    while (desc->cnt) {
        dma_addr_t len = sizeof(buf);
        if (len > desc->cnt) {
            len = desc->cnt;
        }
        DBGOUT("  %x -> %x (count %u)", (unsigned)desc->src,
               (unsigned)desc->dst, (unsigned)desc->cnt);
        if (dma_memory_rw(asrc,
                          desc->src, buf, sizeof(buf),
                          DMA_DIRECTION_TO_DEVICE))
        {
            return false;
        }
        if (dma_memory_rw(adst,
                          desc->dst, buf, sizeof(buf),
                          DMA_DIRECTION_FROM_DEVICE))
        {
            return false;
        }
        desc->cnt -= len;
    }

    return true;
}

static
void do_dma(TSI148State *tsi, unsigned eng)
{
    uint32_t ctrl = tsi->REG(R_DCTL(eng));
    dma_desc D;
    D.pnext = 0;

    DBGOUT("DMA%c Start", '0' + eng);
    tsi->REG(R_DSTA(eng)) = 1 << 24; /* BUSY */

    if (ctrl & (1 << 23)) { /* Direct DMA */
        D.src = BUILD64(tsi->REG(R_DSAU(eng)), tsi->REG(R_DSAL(eng)));
        D.dst = BUILD64(tsi->REG(R_DDAU(eng)), tsi->REG(R_DDAL(eng)));
        D.sattr = tsi->REG(R_DSAT(eng));
        D.dattr = tsi->REG(R_DDAT(eng));
        D.cnt = tsi->REG(R_DCNT(eng));
        D.pnext = 1; /* first and also last */
        dma_write_desc(tsi, eng, 0, &D);

    } else { /* Linked list DMA */
        uint64_t link = BUILD64(tsi->REG(R_DNLAU(eng)), tsi->REG(R_DNLAL(eng)));
        if (!dma_read_desc(tsi, &D, link)) {
            goto fail;
        }
        dma_write_desc(tsi, eng, link, &D);
    }

    while (true) {
        uint64_t link;
        if (!dma_do_desc(tsi, &D)) {
            goto fail;
        }
        if (D.pnext & 1) {
            break;
        }
        DBGOUT("oops");
        break;
        link = D.pnext;
        if (!dma_read_desc(tsi, &D, D.pnext)) {
            goto fail;
        }
        dma_write_desc(tsi, eng, link, &D);
    }

    tsi->REG(R_DSTA(eng)) = 1 << 25; /* DONE */
    tsi->REG(R_INTS) |= 1 << (24 + eng);
    update_irq(tsi);
    DBGOUT("DMA%c Done", '0' + eng);
    return;
fail:
    /* TODO: indicate error source? */
    tsi->REG(R_DSTA(eng)) = 1 << 28; /* ERR */
    tsi->REG(R_INTS) |= 1 << (24 + eng);
    update_irq(tsi);
    DBGOUT("DMA%c Fail", '0' + eng);
}

static
uint64_t tsi148_crg_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t offset = addr;
    TSI148State *tsi = opaque;
    uint32_t val = tsi->REG(addr);

    assert(addr >= PCI_CONFIG_SPACE_SIZE);

    switch (offset) {
    case 0 ... 0xff:
        /* This should have hit the PCI config region */
        break;
    case 0x100 ... 0x200: /* outbound window config */
        break;
    case 0x204 ... 0x220: /* VIACKx */
        /* should hit IACK region */
        break;
    case R_VMCTRL: /* VME Master Control */
    case R_VCTRL:
    case R_VSTAT:
    case R_VEAT:
    case 0x300 ... 0x3ff: /* inbound window config */
    case R_CBAU:
    case R_CBAL:
    case R_CRGAT:
    case R_CRAT:
    case R_VICR:
    case R_INTEN:
    case R_INTEO:
        break;
    case R_INTS:
        val &= tsi->REG(R_INTEN);
        break;
    case R_INTC:
    case R_INTM1:
    case R_INTM2:
        break;
    case 0x500 ... 0x544: /* DMA engine 0 */
    case 0x580 ... 0x5c4: /* DMA engine 1 */
        break;
    case R_GCTRL: /* Control/Status */
        val = 0x01 | (tsi->slot << 8); /* TODO: GAP */
        val |= (1 << 28); /* SCONS */
        val |= tsi->enable ? (1 << 27) : 0;
        val |= tsi->vdev.sysfail ? (1 << 30) : 0;
        val |= tsi->lreset ? (1 << 31) : 0;
        break;
    case 0xff4: /* CSRBCR */
        break;
    case 0xff8: /* CSRBSR */
        /* both Set and Clear read the current state */
        val = tsi->REG(0xff4);
        break;
    case R_CBAR: /* CSRBAR */
        break;
    default:
        LOG(LOG_UNIMP, "Un implemented register %08x\n", (unsigned)offset);
    }

    DBGOUT("read CRG "TARGET_FMT_plx" -> %08x", addr, (unsigned)val);
    return val;
}

static
void tsi148_crg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    uint32_t offset = addr;
    TSI148State *tsi = opaque;

    DBGOUT("write CRG %08x <- %08x", (unsigned)offset, (unsigned)val);

    assert(addr >= PCI_CONFIG_SPACE_SIZE);

    switch (offset) {
    case 0 ... 0xff:
        /* This should have hit the PCI config region */
        break;
    case 0x100 ... 0x200: /* Outbound windows */
        tsi->REG(offset) = val;
        out_window_setup(tsi, (offset - 0x100) / 0x20);
        break;
    case 0x204 ... 0x220: /* VIACKx */
        /* should hit IACK region */
        break;
    case R_VMCTRL: /* VMCTRL VME Master Control */
        val &= 0x511771f;
        if (FIELD_EX32(val, VMCTRL, VS)) {
            val |= R_VMCTRL_VS_MASK; /* immediate ack. of VME Stop */
            LOG(LOG_UNIMP, "VME Stop not implemented");
        }
        tsi->REG(offset) = val;
        break;
    case R_VCTRL: /* VCTRL */
        val &= 0x8f109fcf;
        tsi->REG(offset) = val;
        break;
    case R_VSTAT: /* VSTAT */
        val &= (1 << 14);
        val |= tsi->slot;
        val |= val & (1 << 14) ? 1 << 11 : 0;
        tsi->REG(offset) = val;
        break;
    case R_VEAT: /* VEAT */
        if (val & 0x20000000) {
            tsi->REG(offset) &= ~0xc0000000; /* clear VES and VEOF */
        }
        break;
    case 0x300 ... 0x3ff:
        LOG(LOG_UNIMP, "Inbound windows not implemented\n");
        break;
    case R_CBAU: /* CBAU */
    case R_CBAL: /* CBAL */
        tsi->REG(offset) = val;
        crg_window(tsi);
        break;
    case R_CRGAT: /* CRGAT */
        val &= 0xff;
        tsi->REG(offset) = val;
        crg_window(tsi);
        break;
    case R_CRAT: /* CRAT */
        memory_region_set_enabled(&tsi->crg_crcsr, !!(val & 0x80));
        break;
    case R_VICR: /* VICR */
        if (val) {
            LOG(LOG_UNIMP, "VICR not implemented\n");
        }
        break;
    case R_INTEN: /* INTEN */
    case R_INTEO: /* INTEO */
        val &= 0x01ff3ffe;
        tsi->REG(offset) = val;
        update_irq(tsi);
        break;
    case R_INTS: /* INTS */
        break; /* R/O */
    case R_INTC: /* INTC */
        tsi->REG(R_INTS) &= ~val;
        update_irq(tsi);
        break;
    case R_INTM1: /* INTM1 */
        val &= 0x000fffff;
        tsi->REG(offset) = val;
        update_irq(tsi);
        break;
    case R_INTM2: /* INTM2 */
        val &= 0x0fffffff;
        tsi->REG(offset) = val;
        update_irq(tsi);
        break;
    case R_DCTL(0): /* DCTLx */
    case R_DCTL(1):
        val &= 0x2837777;
        tsi->REG(offset) = val & ~(1 << 25);
        if ((val & (1 << 25)) && tsi->enable) {
            do_dma(tsi, !!(offset & 0x80));
        }
        break;
    case R_DSTA(0): /* DSTAx */
    case R_DSTA(1):
        break; /* R/O */
    case 0x508 ... 0x544: /* DMA engine 0 */
    case 0x588 ... 0x5c4: /* DMA engine 1 */
        tsi->REG(offset) = val;
        break;
    case R_GCTRL: /* Control/Status */
        tsi->lreset = !!(val & (1 << 31)); /* Local Reset */
        if (val & (1 << 31)) { /* Local Reset */
            /* TODO: what does this reset? */
            LOG(LOG_UNIMP, "Local reset not implemented\n");
        }
        tsi->vdev.sysfail = !!(val & (1 << 30)); /* SYSFAIL Enable */
        tsi->enable = !!(val & (1 << 27)); /* Master Enable */
        /*TODO: Notify IOMMU changes */

        update_irq(tsi);
        break;
    case 0xff4: /* CSRBCR */
        val &= 0xd8;
        tsi->REG(offset) &= ~val;
        break;
    case 0xff8: /* CSRBSR */
        val &= 0xd8;
        tsi->REG(0xff4) |= val;
        if (val) {
            LOG(LOG_UNIMP, "CSR control bits unimplemented\n");
        }
        break;
    case R_CBAR: /* CSRBAR */
        if (val != tsi->REG(offset)) {
            LOG(LOG_UNIMP, "Changing CR/CSR BAR not implemented\n");
        }
    default:
        LOG(LOG_UNIMP, "Un implemented register %08x\n", (unsigned)offset);
    }
}

static const MemoryRegionOps crg_ops = {
    .read = tsi148_crg_read,
    .write = tsi148_crg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static
uint64_t tsi148_iack_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t offset = addr, val = 0;
    TSI148State *tsi = opaque;
    unsigned level = (offset >> 2) + 1;

    assert(level >= 1 && level <= 7);

    if (size > 1 || (offset & 0x3) != 3) {
        LOG(LOG_UNIMP, "iack "TARGET_FMT_plx" read%u not implemented."
                       "  Treat as read8\n",
            addr, size * 8);
        size = 1;
    }

    DBGOUT("BEGIN IACK %u", level);

    if (!vme_bus_get_iack(&tsi->vbus, level, &val)) {
        val = 0xffffffff;
        /* TODO: bus error */
    }

    DBGOUT("END IACK %u -> 0x%x", level, (unsigned)val);

    tsi->REG(R_INTS) |= 1 << 10; /* IACK */
    update_irq(tsi);

    return val;
}

static
void tsi148_iack_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    LOG(LOG_GUEST_ERROR, "Can't write to VIACK\n");
}

static const MemoryRegionOps iack_ops = {
    .read = tsi148_iack_read,
    .write = tsi148_iack_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void tsi148_realize(PCIDevice *pci_dev, Error **errp)
{
    Object *obj = OBJECT(pci_dev);
    TSI148State *tsi = TSI148(pci_dev);
    unsigned i;

    DBGOUT("realize() slot=%u", tsi->slot);

    pci_dev->config[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    qdev_prop_set_uint8(&tsi->vdev.qdev, "slot", tsi->slot);

    /* Initialize our VME Bus */
    object_property_set_bool(OBJECT(&tsi->vbus), true, "realized", errp);
    /* Ourself on the VME Bus */
    qdev_init_nofail(DEVICE(&tsi->vdev));

    /* the main region for CRG registers.  All 32-bit access */
    memory_region_init_io(&tsi->crg, obj, &crg_ops, tsi,
                          "tsi148-crg", CRG_SIZE);
    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &tsi->crg);

    /* overlay on top of CRG for the VIACK registers to enable different
     * access widths.
     */
    memory_region_init_io(&tsi->crg_iack, obj, &iack_ops, tsi,
                          "tsi148-crg-iack", 0x220 - 0x204);
    memory_region_add_subregion_overlap(&tsi->crg, 0x204, &tsi->crg_iack, 1);

    /* overlay on top of CRG for PCI config access */
    memory_region_init_io(&tsi->crg_pci, obj, &crg_pci_ops, pci_dev,
                          "tsi148-pci", PCI_CONFIG_SPACE_SIZE);
    memory_region_add_subregion_overlap(&tsi->crg, 0, &tsi->crg_pci, 1);

    /* CRG registers accessible from VME A16,24,32,64 */
    memory_region_init_alias(&tsi->crgwin, obj, "tsi148-crg-win", &tsi->crg,
                             0, CRG_SIZE);
    /* initially un-mapped */

    /* CRG registers accessible from VME CR/CSR */
    memory_region_init_alias(&tsi->crg_crcsr, obj, "tsi148-crg-image",
                             &tsi->crg, 0, CRG_SIZE);
    /* initially un-mapped */

    for (i = 0; i < NUM_OUT; i++) {
        tsi->outbound[i].idx = i;
        tsi->outbound[i].tsi = tsi;
    }

    if (qdev_get_gpio_out_connector(DEVICE(pci_dev), "IRQ", 0) == NULL) {
        /* The TSI 148 is one of the few PCI devices which can make use of all
         * 4 of the IRQ pins available to it.
         * If the board setup codes doesn't handle this then by default we
         * connect only INTA as QEMU's API for PCI devices doesn't seem to
         * support devices using more than one pin.
         */
        qdev_connect_gpio_out_named(DEVICE(pci_dev), "IRQ", 0,
                                    pci_allocate_irq(&tsi->parent_obj));
    }
}

static void tsi148_reset(DeviceState *dev)
{
    TSI148State *tsi = TSI148(dev);
    unsigned i;

    tsi->enable = 0;

    memset(tsi->reg, 0, sizeof(tsi->reg));

    tsi->REG(R_VMCTRL) = 3; /* VME Master Control, default request level 3 */
    tsi->REG(R_VSTAT)  = 0x5900 | tsi->slot; /* TODO GAP */
    tsi->REG(0xff4)    = 0x10; /* Board Fail */
    tsi->REG(R_CBAR)   = tsi->slot << 3;

    /* unmap and disable movable MemoryRegions */

    vme_del_region(&tsi->vdev, &tsi->crg_crcsr);

    if (memory_region_is_mapped(&tsi->crgwin)) {
        memory_region_del_subregion(tsi->crgwin.container, &tsi->crgwin);
    }
    memory_region_set_enabled(&tsi->crgwin, 0);

    for (i = 0; i < NUM_OUT; i++) {
        out_window_setup(tsi, i);
    }

    /* The initial state of the CR/CSR mapping is defined by HW configuration.
     * We enable it if a slot number is known.
     */
    if (tsi->slot) {
        tsi->REG(R_CRAT) = 0x80; /* CR/CSR enable */
        /* set CR/CSR enable in CRAT.
         * Our control registers are also accessible from the VME bus.
         * In fact they can by access through an outbound window to the CR/CSR
         * space. RTEMS uses this image at the end of VME irq unhandlers to
         * flush posted VME writes (eg. for RORA to prevent latching spurious
         * interrupts)
         */
        /* TODO: allow override of slot # */
        vme_add_region(&tsi->vdev, VME_AM_CRCSR,
                       tsi->vdev.slot * 0x80000 + 0x7f000, &tsi->crg_crcsr);
    }

    memory_region_set_enabled(&tsi->crg_crcsr, !!tsi->slot);
}

static
void tsi148_init(Object *obj)
{
    unsigned n;
    TSI148State *tsi = TSI148(obj);
    DeviceState *dev = &tsi->parent_obj.qdev;
    qemu_irq *ivme = qemu_allocate_irqs(vme_irq, tsi, NUM_VME_IRQ);


    vme_bus_init(&tsi->vbus, sizeof(tsi->vbus), dev);
    object_initialize(&tsi->vdev, sizeof(tsi->vdev), TYPE_VME);
    qdev_set_parent_bus(&tsi->vdev.qdev, &tsi->vbus.qbus);

    object_property_add_child(&tsi->vbus.qbus.obj, "vme-scon",
                              &tsi->vdev.qdev.parent_obj, &error_fatal);

    tsi->vbus.berr.cb = &tsi148_berr;
    tsi->vbus.berr.cb_arg = tsi;

    for (n = 0; n < 7; n++) {
        qdev_connect_gpio_out_named(dev, "vme-irq", n, ivme[n]);
    }

    qdev_init_gpio_out_named(dev, tsi->irq, "IRQ", 4);
}

static Property tsi_props[] = {
    DEFINE_PROP_UINT8("slot", TSI148State, slot, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static
void tsi148_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *pci = PCI_DEVICE_CLASS(klass);

    pci->realize = tsi148_realize;
    pci->vendor_id = 0x10e3;
    pci->device_id = 0x0148;
    pci->subsystem_vendor_id = 0x10e3;
    pci->subsystem_id = 0;
    pci->class_id = PCI_CLASS_BRIDGE_OTHER;
    pci->revision = 1;

    set_bit(DEVICE_CATEGORY_BRIDGE, dc->categories);
    dc->desc = "Tsi148 VME bus bridge";
    dc->reset = tsi148_reset;
    dc->props = tsi_props;
}

static const TypeInfo tsi148_type_info = {
    .name = TYPE_TSI148,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(TSI148State),
    .class_size = sizeof(PCIDeviceClass),
    .class_init = tsi148_class_init,
    .instance_init = tsi148_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

static
void tsi148_register_types(void)
{
    type_register_static(&tsi148_type_info);
}

type_init(tsi148_register_types)
