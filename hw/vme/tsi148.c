/*
 * TSI148 VME bus bridge emulation
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
#include "hw/pci/pci.h"
#include "hw/vme/vme_bus.h"

#define TYPE_TSI148 "tsi148"

#define TSI148(obj) OBJECT_CHECK(TSI148State, (obj), TYPE_TSI148)

/* #define TSI148_DEBUG 1 */

#ifdef TSI148_DEBUG
#define	DBGOUT(fmt, ...) printf(TYPE_TSI148": " fmt, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#endif

#define LOG(mask, fmt, ...) qemu_log_mask(mask, fmt, ##__VA_ARGS__)

#define CRG_SIZE 0x1000

#define NUM_OUT 8

typedef struct {
    PCIDevice parent_obj;

    uint8_t slot; /* slot to which this bus controller is attached */

    VMEBus vbus;
    VMEDevice vdev;

    MemoryRegion crg, crg_pci, crcsr;
    MemoryRegion crgwin, *crgwin_parent;

    qemu_irq irq[4];

    uint32_t reg[CRG_SIZE>>2]; /* CRG registers */

    bool enable;
    bool lreset;
    unsigned outbound_active;
    MemoryRegion outbound[NUM_OUT];
} TSI148State;

static
uint64_t tsi148_conf_read(void *opaque, hwaddr addr, unsigned size)
{
    PCIDevice *pci = opaque;
    uint64_t val;
    assert(addr<PCI_CONFIG_SPACE_SIZE);
    val = pci->config_read(pci, addr, size);
    DBGOUT("read config "TARGET_FMT_plx" -> %08x\n", addr, (unsigned)val);
    return val;
}

static
void tsi148_conf_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    PCIDevice *pci = opaque;
    DBGOUT("write config "TARGET_FMT_plx" <- %08x\n", addr, (unsigned)val);
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

/* The TSI148 has 4 IRQ lines to which the various
 * internal IRQ sources can be mapped.
 */
static
void update_irq(TSI148State *tsi)
{
    unsigned i;
    uint64_t map = tsi->reg[0x458>>2];
    uint32_t ena = tsi->reg[0x448>>2],
             out = tsi->reg[0x44c>>2],
             sts = tsi->reg[0x450>>2],
             act = ena & out & sts && tsi->enable;
    bool level[4] = {0,0,0,0};

    map<<=32;
    map |= tsi->reg[0x45c>>2];

    DBGOUT("IRQ ACT %08x ENA %08x OUT %08x STS %08x ",
           (unsigned)act, (unsigned)ena, (unsigned)out, (unsigned)sts);

    for(i=0; i<32; i++, act>>=1, map>>=2) {
        if(act&1) {
            level[map&3] = 1;
        }
    }

    for(i=0; i<4; i++) {
#ifdef TSI148_DEBUG
        printf("%c", level[i] ? 'X' : '_');
#endif
        qemu_set_irq(tsi->irq[i], level[i]);
    }
#ifdef TSI148_DEBUG
    printf("\n");
#endif
}

static
void vme_irq(void *opaque, int n, int level)
{
    TSI148State *tsi = opaque;
    uint32_t mask = 1<<(n+1);

    DBGOUT("VME%u %sassert\n", n+1,
           level ? "" : "de");

    assert(n<7);

    if(level)
        tsi->reg[0x450>>2] |= mask;
    else
        tsi->reg[0x450>>2] &= ~mask; /* todo: deassert w/o iack? */
    update_irq(tsi);
}

static
void tsi148_berr(VMEBus *vbus, void *opaque)
{
    TSI148State *tsi = opaque;
    uint32_t status = (1<<31)|(1<<19); /* VES and BERR */

    if (tsi->reg[0x268>>2]&(1<<31)) {
        DBGOUT("BERR overflow\n");
        tsi->reg[0x268>>2] |= 1<<30; /* overflow */
        return;
    }

    DBGOUT("BERR\n");

    status |= tsi->vbus.berr.write ? 1<<17 : 0;

    /* TODO: LWORD, DS0, DS1 */

    status |= tsi->vbus.berr.amod<<8;

    tsi->reg[0x260>>2] = tsi->vbus.berr.addr>>32;
    tsi->reg[0x264>>2] = tsi->vbus.berr.addr;
    tsi->reg[0x268>>2] = status;

    tsi->reg[0x450>>2] |= 1<<12;
    update_irq(tsi);
}

static
bool translate_amod(uint8_t *out, uint8_t amod, bool sup, bool pgm, uint8_t mode)
{
    uint8_t ret;
    switch (amod) {
    case 0: ret = VME_AM_MSK_A16; break;
    case 1: ret = VME_AM_MSK_A24; break;
    case 2: ret = VME_AM_MSK_A32; break;
    /* case 4: ret = VME_AM_MSK_A64; break; */
    case 5:
        *out = VME_AM_CRCSR;
        return true;
    default:
        return false;
    }

    switch (mode) {
    case 0:
        ret |= pgm ? VME_AM_MSK_PRG : VME_AM_MSK_DAT;
        break;
    case 1: ret |= VME_AM_MSK_BLT; break;
    case 2: ret |= VME_AM_MSK_MBLT; break;
    default:
        return false;
    }

    ret |= sup ? VME_AM_BIT_SUP : 0;

    *out = ret;
    return true;
}

#define CRG_OUT(N, OFF) tsi->reg[(0x100+(N)*0x20+(OFF))>>2]

static
void tsi_update_out(TSI148State *tsi, unsigned N)
{
    /* Use the Bus MR for outbound (host -> device) */
    MemoryRegion *breg = pci_address_space(&tsi->parent_obj),
            *areg, *alias = &tsi->outbound[N];
    uint64_t pci_start = CRG_OUT(N, 0x00),
             pci_end   = CRG_OUT(N, 0x08),
             vme_off   = CRG_OUT(N, 0x10),
             pci_size;
    uint32_t ctrl = CRG_OUT(N, 0x1c);
    uint8_t amod;

    pci_start <<= 32;
    pci_end <<= 32;
    vme_off <<= 32;

    pci_start |= CRG_OUT(N, 0x04);
    pci_end |= CRG_OUT(N, 0x0c);
    vme_off |= CRG_OUT(N, 0x14);

    vme_off += pci_start;

    if(tsi->outbound_active&(1<<N)) {
        memory_region_del_subregion(breg, alias);
        tsi->outbound_active &= ~(1<<N);
        assert(alias->parent_obj.ref==0);
    }

    DBGOUT("CTRL  %08x\n", (unsigned)ctrl);

    if(!(ctrl&(1<<31))) {
        return; /* disabled */
    }

    if(pci_end<pci_start) {
        pci_size = 0;
    } else {
        /* valid region is [start, end] inclusive */
        pci_size = pci_end - pci_start + 1;
    }

    DBGOUT("START %16llx\n", (unsigned long long)pci_start);
    DBGOUT("END   %16llx\n", (unsigned long long)pci_end);
    DBGOUT("VME   %16llx\n", (unsigned long long)vme_off);

    if(pci_size==0) {
        LOG(LOG_GUEST_ERROR, "Outbound windown %u would have negative size\n", N);
        return; /* disabled */
    }

    if (!translate_amod(&amod, ctrl&0xf, !!(ctrl&(1<<5)), !!(ctrl&(1<<4)),
                extract32(ctrl, 8, 3)))
    {
        LOG(LOG_UNIMP, "Unimplemented address modifier\n");
        return;
    }

    areg = vme_bus_get_region(&tsi->vbus, amod);

    if(!areg) {
        LOG(LOG_UNIMP, "Unsupported address modifier %02x\n", amod);
        return;
    }

    memory_region_init_alias(alias,&tsi->parent_obj.qdev.parent_obj,
                             "vme", areg, vme_off, pci_size);
    memory_region_set_enabled(alias, tsi->enable);

    memory_region_add_subregion(breg, pci_start, alias);

    tsi->outbound_active |= 1<<N;

    DBGOUT("outbound %u active on %02x %p (under %s)\n", N, amod, areg, breg->name);
}

static
void crg_window(TSI148State *tsi)
{
    MemoryRegion *reg;
    uint8_t amod = 0;
    uint32_t attrs = tsi->reg[0x414>>2];
    hwaddr off = tsi->reg[0x40c>>2];

    off<<=32;
    off |= tsi->reg[0x410>>2];

    if (tsi->crgwin_parent) {
        memory_region_del_subregion(tsi->crgwin_parent, &tsi->crgwin);
        tsi->crgwin_parent = NULL;
    }

    if (!(attrs&(1<<7))) {
        return;
    }

    /* TODO, can respond to both sup and prg */
    if (!translate_amod(&amod, extract32(attrs, 4, 3),
                        extract32(attrs, 3, 1),
                        extract32(attrs, 1, 1), 0)) {
        LOG(LOG_GUEST_ERROR, "CRG window attribute, incorrect "
                             "space %08x\n", (unsigned)attrs);
        return;
    }

    switch((attrs>>4)&7) {
    case 0: /* A16 */
        if (attrs&(1<<3)) {
            amod = VME_AM_A16_SUP;
        } else if (attrs&(1<<2)) {
            amod = VME_AM_A16_USR;
        }
        break;
    case 1: /* A24 */
        /* TODO */
        break;
    case 2: /* A32 */
        switch(attrs&0xf) {
        case 0b1010: amod = VME_AM_A32_SUP_PRG; break;
        case 0b1001: amod = VME_AM_A32_SUP_DAT; break;
        case 0b0110: amod = VME_AM_A32_USR_PRG; break;
        case 0b0101: amod = VME_AM_A32_USR_DAT; break;
        }
        break;
    case 4: /* A64 */
        /* TODO */
        break;
    }

    if (amod==0) {
        LOG(LOG_GUEST_ERROR, "CRG window attribute, incorrect "
                             "space %08x\n", (unsigned)attrs);
        return;
    }

    reg = vme_bus_get_region(&tsi->vbus, amod);
    if (!reg) {
        LOG(LOG_GUEST_ERROR|LOG_UNIMP,
            "CRG window attribute, incorrect "
            "space %08x (may be unimplemeented\n", (unsigned)attrs);
        return;
    }

    memory_region_add_subregion(reg, off, &tsi->crgwin);
    tsi->crgwin_parent = reg;
    DBGOUT("CRG window enabled\n");
}

#define DMA_REG(N, OFF) tsi->reg[((OFF)+(N)*0x80)>>2]

#define DMA_GET64(N, OFF) ( ((uint64_t)DMA_REG(N,OFF))<<32 | \
    ((uint64_t)DMA_REG(N,OFF+4)) )

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

    if(desc->pnext&6) {
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

    desc->src = ((uint64_t)be32_to_cpu(raw[0]))<<32;
    desc->src|= be32_to_cpu(raw[1]);
    desc->dst = ((uint64_t)be32_to_cpu(raw[2]))<<32;
    desc->dst|= be32_to_cpu(raw[3]);
    desc->sattr = be32_to_cpu(raw[4]);
    desc->dattr = be32_to_cpu(raw[5]);
    desc->pnext = ((uint64_t)be32_to_cpu(raw[6]))<<32;
    desc->pnext|= be32_to_cpu(raw[7]);
    desc->cnt   = be32_to_cpu(raw[8]);
    desc->bcast = be32_to_cpu(raw[5]);

    return true;
}

static
void dma_write_desc(TSI148State *tsi, unsigned eng, uint64_t link,
                    const dma_desc *desc)
{
    DMA_REG(eng, 0x508) = desc->src>>32;
    DMA_REG(eng, 0x50c) = desc->src;
    DMA_REG(eng, 0x510) = desc->dst>>32;
    DMA_REG(eng, 0x514) = desc->dst;
    DMA_REG(eng, 0x518) = link>>32;
    DMA_REG(eng, 0x51c) = link;
    DMA_REG(eng, 0x530) = desc->sattr;
    DMA_REG(eng, 0x534) = desc->dattr;
    DMA_REG(eng, 0x538) = desc->pnext>>32;
    DMA_REG(eng, 0x53c) = desc->pnext;
    DMA_REG(eng, 0x540) = desc->cnt;
    DMA_REG(eng, 0x544) = desc->bcast;
}

static
AddressSpace* dma_get_vme_space(TSI148State *tsi, uint32_t attr)
{
    uint8_t amod = 0xff;

    if (!translate_amod(&amod, attr&0xf, !!(attr&(1<<5)), !!(attr&(1<<4)),
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

    if (desc->sattr&(1<<28)) {
        asrc = dma_get_vme_space(tsi, desc->sattr);
    } else {
        asrc = pci_get_address_space(&tsi->parent_obj);
    }
    if (desc->dattr&(1<<28)) {
        adst = dma_get_vme_space(tsi, desc->dattr);
    } else {
        adst = pci_get_address_space(&tsi->parent_obj);
    }

    if (!asrc || !adst) {
        return false;
    }

    while (desc->cnt) {
        dma_addr_t len = sizeof(buf);
        if (len>desc->cnt) {
            len = desc->cnt;
        }
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
        desc->cnt -= sizeof(buf);
    }

    return true;
}

static
void do_dma(TSI148State *tsi, unsigned eng)
{
    uint32_t ctrl = DMA_REG(eng, 0x500);
    dma_desc D;

    DMA_REG(eng, 0x504) = 1<<24; /* BUSY */

    if (ctrl&(1<<23)) { /* Direct DMA */
        D.src = DMA_GET64(eng, 0x508);
        D.dst = DMA_GET64(eng, 0x510);
        D.sattr = DMA_REG(eng, 0x530);
        D.dattr = DMA_REG(eng, 0x534);
        D.cnt = DMA_REG(eng, 0x540);
        D.pnext = 1; /* first and also last */
        dma_write_desc(tsi, eng, 0, &D);

    } else { /* Linked list DMA */
        uint64_t link = DMA_GET64(eng, 0x538);
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
        if (D.pnext&1) {
            goto fail;
        }
        link = D.pnext;
        if (!dma_read_desc(tsi, &D, D.pnext)) {
            goto fail;
        }
        dma_write_desc(tsi, eng, link, &D);
    }

    DMA_REG(eng, 0x504) = 1<<25; /* DONE */
    tsi->reg[0x450>>2] |= 1<<(24+eng);
    update_irq(tsi);
    return;
fail:
    /* TODO: indicate error source? */
    DMA_REG(eng, 0x504) = 1<<28; /* ERR */
    tsi->reg[0x450>>2] |= 1<<(24+eng);
    update_irq(tsi);
}

static
uint64_t tsi148_crg_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t offset = addr;
    TSI148State *tsi = opaque;
    uint32_t val = tsi->reg[addr>>2];

    assert(addr>=PCI_CONFIG_SPACE_SIZE);

    switch(offset) {
    case 0 ... 0xff:
        /* This should have hit the PCI window */
        break;
    case 0x100 ... 0x200: /* outbound window config */
        break;
    case 0x204 ... 0x220: /* VIACKx */
        /* TODO: support 16/32-bit IACK? */
        if(!vme_bus_get_iack(&tsi->vbus, (offset-0x200)>>2, &val))
            val = 0xffffffff;
        val <<= 8*(offset&3);
        val >>= 8*(4-size);
        break;
    case 0x234: /* VME Master Control */
    case 0x238: /* VCTRL */
    case 0x23c: /* VSTAT */
    case 0x268: /* VEAT */
    case 0x300 ... 0x3ff: /* inbound window config */
    case 0x40c: /* CBAU */
    case 0x410: /* CBAL */
    case 0x414: /* CRGAT */
    case 0x420: /* CRAT */
    case 0x440: /* VICR */
    case 0x448: /* INTEN */
    case 0x44c: /* INTEO */
        break;
    case 0x450: /* INTS */
        val &= tsi->reg[0x448>>2];
        break;
    case 0x454: /* INTC */
    case 0x458: /* INTM1 */
    case 0x45C: /* INTM2 */
        break;
    case 0x500: /* DCTLx */
    case 0x580:
    case 0x504: /* DSTAx */
    case 0x584:
    case 0x508: /* DCSAUx */
    case 0x588:
    case 0x50c: /* DCSALx */
    case 0x58c:
    case 0x510: /* DCDAUx */
    case 0x590:
    case 0x514: /* DCDALx */
    case 0x594:
    case 0x518: /* DCLAUx */
    case 0x598:
    case 0x51c: /* DCLALx */
    case 0x59c:
    case 0x520: /* DDSAUx */
    case 0x5a0:
    case 0x524: /* DDSALx */
    case 0x5a4:
    case 0x528: /* DDDAUx */
    case 0x5a8:
    case 0x52c: /* DDDALx */
    case 0x5ac:
    case 0x530: /* DSATx */
    case 0x5b0:
    case 0x534: /* DDATx */
    case 0x5b4:
    case 0x538: /* DNLAUx */
    case 0x5b8:
    case 0x53c: /* DNLALx */
    case 0x5bc:
    case 0x540: /* DCNTx */
    case 0x5c0:
    case 0x544: /* DDBSx */
    case 0x5c4:
        break;
    case 0x604: /* Control/Status */
        val = 0x01 | (tsi->slot<<8); /* TODO: GAP */
        val |= (1<<28); /* SCONS */
        val |= tsi->enable ? (1<<27) : 0;
        val |= tsi->vdev.sysfail ? (1<<30) : 0;
        val |= tsi->lreset ? (1<<31) : 0;
        break;
    case 0xff4: /* CSRBCR */
        break;
    case 0xff8: /* CSRBSR */
        val = tsi->reg[0xff4>>2]; /* both Set and Clear read the current state */
        break;
    case 0xffc: /* CSRBAR */
        break;
    default:
        LOG(LOG_UNIMP, "Un implemented register %08x\n", (unsigned)offset);
    }

    DBGOUT("read CRG "TARGET_FMT_plx" -> %08x\n", addr, (unsigned)val);
    return val;
}

static
void tsi148_crg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    uint32_t offset = addr;
    TSI148State *tsi = opaque;
    unsigned N;

    DBGOUT("write CRG %08x <- %08x\n", (unsigned)offset, (unsigned)val);

    assert(addr>=PCI_CONFIG_SPACE_SIZE);

    switch(offset) {
    case 0 ... 0xff:
        break;
    case 0x100 ... 0x200: /* Outbound windows */
        N = (offset-0x100)/0x20; /* window # */
        tsi->reg[offset>>2] = val;
        tsi_update_out(tsi, N);
        break;
    case 0x234: /* VMCTRL VME Master Control */
        val &= 0x511771f;
        val |= val&(1<<26) ? 1<<27 : 0; /* immediate ack. of VS */
        tsi->reg[offset>>2] = val;
        break;
    case 0x238: /* VCTRL */
        val &= 0x8f109fcf;
        tsi->reg[offset>>2] = val;
        break;
    case 0x23c: /* VSTAT */
        val &= (1<<14);
        val |= tsi->slot;
        val |= val&(1<<14) ? 1<<11 : 0;
        tsi->reg[offset>>2] = val;
        break;
    case 0x268: /* VEAT */
        if(val&0x20000000) {
            tsi->reg[offset>>2] &= ~0xc0000000; /* clear VES and VEOF */
        }
        break;
    case 0x300 ... 0x3ff:
        LOG(LOG_UNIMP, "Inbound windows not implemented\n");
        break;
    case 0x40c: /* CBAU */
    case 0x410: /* CBAL */
        tsi->reg[offset>>2] = val;
        crg_window(tsi);
        break;
    case 0x414: /* CRGAT */
        val &= 0xff;
        tsi->reg[offset>>2] = val;
        crg_window(tsi);
        break;
    case 0x420: /* CRAT */
        memory_region_set_enabled(&tsi->crcsr, !!(val&0x80));
        break;
    case 0x440: /* VICR */
        //tsi->reg[offset>>2] = val;
        if(val)
            LOG(LOG_UNIMP, "VICR not implemented\n");
        break;
    case 0x448: /* INTEN */
    case 0x44c: /* INTEO */
        val &= 0x01ff3ffe;
        tsi->reg[offset>>2] = val;
        update_irq(tsi);
        break;
    case 0x450: /* INTS */
        break; /* R/O */
    case 0x454: /* INTC */
        tsi->reg[offset>>2] &= ~val;
        break;
    case 0x458: /* INTM1 */
        val &= 0x000fffff;
        tsi->reg[offset>>2] = val;
        update_irq(tsi);
        break;
    case 0x45C: /* INTM2 */
        val &= 0x0fffffff;
        tsi->reg[offset>>2] = val;
        update_irq(tsi);
        break;
    case 0x500: /* DCTLx */
    case 0x580:
        val &= 0x2837777;
        if ((val&(1<<25)) && tsi->enable) {
            do_dma(tsi, !!(offset&0x80));
            val &= !(1<<25);
        }
        tsi->reg[offset>>2] = val;
        break;
    case 0x504: /* DSTAx */
    case 0x584:
        break; /* R/O */
    case 0x508: /* DCSAUx */
    case 0x588:
    case 0x50c: /* DCSALx */
    case 0x58c:
    case 0x510: /* DCDAUx */
    case 0x590:
    case 0x514: /* DCDALx */
    case 0x594:
    case 0x518: /* DCLAUx */
    case 0x598:
    case 0x51c: /* DCLALx */
    case 0x59c:
    case 0x520: /* DDSAUx */
    case 0x5a0:
    case 0x524: /* DDSALx */
    case 0x5a4:
    case 0x528: /* DDDAUx */
    case 0x5a8:
    case 0x52c: /* DDDALx */
    case 0x5ac:
    case 0x530: /* DSATx */
    case 0x5b0:
    case 0x534: /* DDATx */
    case 0x5b4:
    case 0x538: /* DNLAUx */
    case 0x5b8:
    case 0x53c: /* DNLALx */
    case 0x5bc:
    case 0x540: /* DCNTx */
    case 0x5c0:
    case 0x544: /* DDBSx */
    case 0x5c4:
        tsi->reg[offset>>2] = val;
        break;
    case 0x604: /* Control/Status */
        tsi->lreset = !!(val&(1<<31)); /* Local Reset */
        if (val&(1<<31)) { /* Local Reset */
            /* TODO: what does this reset? */
            LOG(LOG_UNIMP, "Local reset not implemented\n");
        }
        tsi->vdev.sysfail = !!(val&(1<<30)); /* SYSFAIL Enable */
        tsi->enable = !!(val&(1<<27)); /* Master Enable */
        for(N=0; N<NUM_OUT; N++) {
            if (tsi->outbound_active&(1<<N)) {
                memory_region_set_enabled(&tsi->outbound[N], tsi->enable);
            }
        }
        update_irq(tsi);
        break;
    case 0xff4: /* CSRBCR */
        val &= 0xd8;
        tsi->reg[offset>>2] &= ~val;
        break;
    case 0xff8: /* CSRBSR */
        val &= 0xd8;
        tsi->reg[0xff4>>2] |= val;
        if(val)
            LOG(LOG_UNIMP, "CSR control bits unimplemented\n");
        break;
    case 0xffc: /* CSRBAR */
        if(val!=tsi->reg[offset>>2])
            LOG(LOG_UNIMP, "Changing CR/CSR BAR not implemented\n");
    default:
        LOG(LOG_UNIMP, "Un implemented register %08x\n", (unsigned)offset);
    }
}

static const MemoryRegionOps crg_ops = {
    .read = tsi148_crg_read,
    .write = tsi148_crg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
    }
};

static void tsi148_realize(PCIDevice *pci_dev, Error **errp)
{
    Object *obj = &pci_dev->qdev.parent_obj;
    TSI148State *tsi = TSI148(pci_dev);

    DBGOUT("realize()\n");

    pci_dev->config[PCI_INTERRUPT_PIN] = 1; /* interrupt pin A */

    qdev_prop_set_uint8(&tsi->vdev.qdev, "slot", tsi->slot);

    object_property_set_bool(&tsi->vbus.qbus.obj, true, "realized", errp);
    qdev_init_nofail(&tsi->vdev.qdev);

    pci_dev->config[PCI_CACHE_LINE_SIZE] = 1; /* INTA */

    memory_region_init_io(&tsi->crg, obj, &crg_ops, tsi,
                          "tsi148-crg", CRG_SIZE);

    memory_region_init_io(&tsi->crg_pci, obj, &crg_pci_ops, pci_dev,
                          "tsi148-pci", PCI_CONFIG_SPACE_SIZE);

    memory_region_init_alias(&tsi->crgwin, obj, "tsi148-crg-win", &tsi->crg,
                             0, CRG_SIZE);

    pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &tsi->crg);

    memory_region_add_subregion(&tsi->crg, 0, &tsi->crg_pci);

    memory_region_init_alias(&tsi->crcsr, obj, "tsi148-crg-image", &tsi->crg,
                             0, CRG_SIZE);

    if(qdev_get_gpio_out_connector(DEVICE(pci_dev), "IRQ", 0)==NULL) {
        /* The TSI 148 is one of the few PCI devices which can make use of all 4
         * of the IRQ pins available to it.
         * The board setup codes doesn't handle this then by default we connect
         * only INTA as QEMU's API for PCI devices doesn't seem to support devices
         * using more than one pin.
         */
        qdev_connect_gpio_out_named(DEVICE(pci_dev), "IRQ", 0, pci_allocate_irq(&tsi->parent_obj));
    }
}

static void tsi148_reset(DeviceState *dev)
{
    TSI148State *tsi = TSI148(dev);

    tsi->reg[0x234>>2] = 3; /* VME Master Control */
    tsi->reg[0x23c>>2] = 0x5900 | tsi->slot; /* TODO GAP */
    tsi->reg[0xff4>>2] = 0x10; /* Board Fail */
    tsi->reg[0xffc>>2] = tsi->slot<<3;
    /* The initial state of the CR/CSR mapping is defined by HW configuration.
     * We enable it if a slot number is known.
     */
    if(tsi->slot) {
        tsi->reg[0x420] = 0x80; /* CR/CSR enable */
        // set CR/CSR enable in CRAT
        /* Our control registers are also accessible from the VME bus.
         * In fact they can by access through an outbound window to the CR/CSR space.
         * RTEMS uses this image at the end of VME irq unhandlers to flush
         * posted VME writes (eg. for RORA to prevent latching spurious interrupts)
         */
        vme_add_region(&tsi->vdev, VME_AM_CRCSR, tsi->vdev.slot*0x80000+0x7f000, &tsi->crcsr);
    }
}

static
void tsi148_init(Object *obj)
{
    unsigned n;
    TSI148State *tsi = TSI148(obj);
    DeviceState *dev = &tsi->parent_obj.qdev;
    qemu_irq *ivme = qemu_allocate_irqs(vme_irq, tsi, 7);


    vme_bus_init(&tsi->vbus, sizeof(tsi->vbus), dev);
    object_initialize(&tsi->vdev, sizeof(tsi->vdev), TYPE_VME);
    qdev_set_parent_bus(&tsi->vdev.qdev, &tsi->vbus.qbus);

    object_property_add_child(&tsi->vbus.qbus.obj, "vme-scon",
                              &tsi->vdev.qdev.parent_obj, &error_fatal);

    tsi->vbus.berr.cb = &tsi148_berr;
    tsi->vbus.berr.cb_arg = tsi;

    for (n=0; n<7; n++) {
        qdev_connect_gpio_out_named(dev,
                                    "vme-irq", n,
                                    ivme[n]);
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
};

static
void tsi148_register_types(void)
{
    type_register_static(&tsi148_type_info);
}

type_init(tsi148_register_types)
