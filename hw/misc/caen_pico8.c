
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "hw/pci/msi.h"
#include "sysemu/dma.h"

#if 0
#define	DBGOUT(fmt, ...) fprintf(stderr, "pico8: " fmt, ## __VA_ARGS__)
#define	DBGOUTx(fmt, ...) fprintf(stderr, fmt, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#define	DBGOUTx(fmt, ...) do {} while (0)
#endif


#define TYPE_CAEN_PICO8 "amc-pico-8"

#define PICO8(obj) OBJECT_CHECK(PicoState, (obj), TYPE_CAEN_PICO8)

/* request and response fifo depths */
#define NDMA (1<<11)

#define REGSIZE 0x40000

#define PICO_DDR_PAGES 16
#define PICO_DDR_PAGE (16*1024*1024)

typedef struct {
    uint32_t base;
    uint32_t count;
    uint32_t cmd; /* unused in response fifo */
} dma_fifo_ent_t;

typedef struct {
    dma_fifo_ent_t entries[NDMA];
    unsigned in, out;
} dma_fifo_t;

typedef struct {
    PCIDevice parent_obj;

    AddressSpace mram_as;
    MemoryRegion mmio, mram, mram_access;

    uint32_t regval[REGSIZE>>2];

    uint32_t irq_mask, irq_status;
    unsigned dma_ena:1;

    dma_fifo_t cmd, resp;

    int64_t start_time, next_time;

    bool frib; /* simulate frib specific logic */
    QEMUTimer sumtimer;
} PicoState;

static
unsigned fifo_used(const dma_fifo_t *fifo)
{
    /* maxcount==4
     * in==out -> 0 (empty)
     * in==2, out==1 -> 1
     * in==3, out==1 -> 2
     * in==0, out==1 -> 3 (4-1+0)
     *
     * in==3, out==3 -> 0
     * in==0, out==3 -> 1 (4-3+0)
     * in==1, out==3 -> 2 (4-3+1)
     * in==2, out==3 -> 4 (4-3+2)
     */
    if(fifo->in>=fifo->out)
        return fifo->in-fifo->out;
    else
        return NDMA-fifo->out+fifo->in;
}

static inline
unsigned fifo_empty(const dma_fifo_t *fifo)
{
    return fifo->in==fifo->out;
}

static inline
unsigned fifo_full(const dma_fifo_t *fifo)
{
    unsigned next = (fifo->in+1)%NDMA;
    return next==fifo->out;
}

static inline
void fifo_reset(dma_fifo_t *fifo)
{
    fifo->in = fifo->out = 0u;
}

static
void pico_update_irq(PicoState *pico)
{
    uint32_t irq = pico->irq_status;
    bool prev = !!irq;

    if(!fifo_empty(&pico->resp))
        irq |= 1; /* DMA Done */
    if(pico->regval[0x30004>>2] & (1<<17))
        irq |= 2; /* summary record ready */

    /* not sure how IRQ while masked behaves */
    irq &= pico->irq_mask;

    if(msi_enabled(&pico->parent_obj)) {
        /* send MSI on rising edge of !!irq_status */
        if(!!irq && !prev) {
            msi_notify(&pico->parent_obj, 0);
            DBGOUT("-> MSI\n");
        }
    }

    /* update IRQ pin */
    if(!!irq ^ !prev)
        DBGOUT("IRQ %c\n", pico->irq_status ? '!' : '_');
    pico->irq_status =   irq;
    pci_set_irq(&pico->parent_obj, !!irq);
}

static void pico8_dma_reset(PicoState *pico)
{
    pico->dma_ena = 0; /* enabled cleared? */

    pico->cmd.in = pico->cmd.out = 0;
    pico->resp.in = pico->resp.out = 0;
}

static
void pico_run_dma(PicoState *pico)
{
    /* a static pattern for all 8 channels repeating every 4 samples */
    static const float buf[] = { 0.1,  1.1,  2.1,  3.1,  4.1,  5.1,  6.1,  7.1,
                                 0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,
                                -0.1, -1.1, -2.1, -3.1, -4.1, -5.1, -6.1, -7.1,
                                 0.0,  1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0};
    AddressSpace * const pcispace = pci_get_address_space(&pico->parent_obj);
    bool do_irq = 0;

    if(!pico->dma_ena)
        return;

    /* no sure what real hardware does when response fifo is full, we stall */
    while(!fifo_empty(&pico->cmd) && !fifo_full(&pico->resp)) {
        int fail = 0;
        dma_fifo_ent_t *cmd = &pico->cmd.entries[pico->cmd.out],
                       *resp= &pico->resp.entries[pico->resp.in];

        do_irq |= !!(cmd->cmd&0x08000000);

        DBGOUT("pico_run_dma %08x count %08x do_irq %u resp_in %u resp_out %u\n",
               (unsigned)cmd->base, (unsigned)cmd->count,
               do_irq, pico->resp.in, pico->resp.out);

        /* copy in static pattern */
        for(;cmd->count>=sizeof(buf); cmd->base+=sizeof(buf), cmd->count-=sizeof(buf)) {
            fail |= dma_memory_write(pcispace, cmd->base, buf, sizeof(buf));
        }
        if(cmd->count) {
            fail |= dma_memory_write(pcispace, cmd->base, buf, cmd->count);
        }

        if(fail) {
            qemu_log_mask(LOG_GUEST_ERROR, "pico8: DMA fails: %x\n", fail);
        }

        /* add to response fifo */
        resp->base = cmd->base;
        resp->count = cmd->count;
        /* resp->cmd is unused */

        /* clear processed fifo entry */
        cmd->base = cmd->count = 0;

        pico->cmd.out = (pico->cmd.out+1)%NDMA;
        pico->resp.in = (pico->resp.in+1)%NDMA;
    }

    if(do_irq) {
        pico_update_irq(pico);
    }
}

static inline
uint32_t f2i(float v)
{
    union {
        uint32_t i;
        float f;
    } pun;
    pun.f = v;
    return pun.i;
}

static uint64_t
pico8_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    PicoState *pico = opaque;
    uint32_t ret = pico->regval[addr>>2];

    DBGOUT("pico8 read "TARGET_FMT_plx"\n", addr);

    switch(addr) {
    case 0 ... 0x18: /* control registers */
        break;
    case 0x20 ... 0x3c: /* raw sample registers */
        ret = 1e3 * (addr-0x20)/4;
        break;
    case 0x40 ... 0x5c: /* cooked sample registers */
        ret = 1e-6 * (addr-0x40)/4;
        break;
    case 0x78:
    case 0x7c:
        break;
    case 0x100 ... 0x17c: /* scaling factors */
        break;
    /* DMA engine */
    case 0x10000: /* DMA status */
        /* number of entries in response fifo */
        ret = fifo_used(&pico->resp)<<16;
        ret|= fifo_used(&pico->cmd)<<4;
        /* ret[0] always zero since we model DMA as instantaneous */
        break;
    case 0x10004: /* DMA Control */
        ret = pico->dma_ena ? 0x100 : 0;
        break;
    case 0x10008: /* DMA Command */
        ret = 0;
        break;
    case 0x1000C: /* DMA Command Start */
    case 0x10010: /* DMA Command Count */
        break;
    case 0x10014: /* DMA Response Count */
        return pico->resp.entries[pico->resp.out].count;
    case 0x10018: /* DMA Response Start */
        return pico->resp.entries[pico->resp.out].base;
    case 0x20000: /* Mux */
        break;
    /* User */
    case 0x30000 ... 0x33fff:
        if(pico->frib) {
            switch(addr) {
            case 0x30000:
                ret = 0x71C02DD5;
                break;
            case 0x30004 ... 0x3000c:
                break;
            case 0x30040:
                ret = 0xb105; /* FRIB FW sub-version */
                break;
            case 0x30044:
            case 0x30048:
                break;
            case 0x3004c:
            {
                uint64_t ellapsed = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT)-pico->start_time; // [ns]
                ellapsed /= 10000000u; // [10 ms]
                ellapsed &= 0xffff;
                /* all attempts are successes :) */
                ret = ellapsed | (ellapsed<<16);
            }
                break;
            case 0x30050: /* current time, sec */
                ret = qemu_clock_get_ns(QEMU_CLOCK_HOST)/NANOSECONDS_PER_SECOND; // [sec]
                break;
            case 0x30054: /* current time, sub-sec (ticks of 10.0625 MHz clock) */
                ret = qemu_clock_get_ns(QEMU_CLOCK_HOST)%NANOSECONDS_PER_SECOND; // [ns]
                ret *= 10.0625e6/1e9; // [ns] * [cycle/ns] -> [cycles]
                break;
            case 0x30058 ... 0x30064:
            case 0x300c0 ... 0x300ec:
            case 0x30100 ... 0x301ac:
                break;
            default:
                qemu_log_mask(LOG_GUEST_ERROR, "pico8: read from undefined FRIB register "TARGET_FMT_plx"\n", addr);
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "pico8: read from undefined USER register "TARGET_FMT_plx"\n", addr);
        }
        break;
    /* DDR Page select */
    case 0x3c000:
        ret = 0xdd527ACE;
        break;
    case 0x3c004:
        break;
    /* Interrupt controller */
    case 0x40000:
        ret = 0x157C5721;
        break;
    case 0x40004:
        ret  = !!pico->irq_status;
        ret |= msi_enabled(&pico->parent_obj) ? 1<<8 : 0;
        break;
    case 0x40008:
        break;
    case 0x4000c:
        ret = pico->irq_status;
        break;
    case 0x40014:
        ret = pico->irq_mask;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "pico8: read from undefined register "TARGET_FMT_plx"\n", addr);
        ret = 0xdeadbeef;
    }

    DBGOUT("read %08x -> %08x\n", (unsigned)addr, (unsigned)ret);

    return ret;
}

static void
pico8_mmio_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    PicoState *pico = opaque;

    DBGOUT("pico8 write %08x <- %08x\n", (unsigned)addr, (unsigned)val);

    /* fall through to store in regval */

    switch(addr) {
    case 0x0:
        val &= 0xf;
        break;
    case 0x4:
        val &= 0x0707;
        break;
    case 0x8:
        val &= 0x7ff;
        break;
    case 0xc:
        val &= 0x3ff;
        break;
    case 0x10:
        val &= 0x0703;
        break;
    case 0x14:
    case 0x18:
    case 0x100 ... 0x17c: /* scaling factors */
        break;
    case 0x10004: /* DMA Control */
        pico->dma_ena = !!(val&0x100);
        if(val&1) { /* Reset */
            pico8_dma_reset(pico);
        } else {
            pico_run_dma(pico);
        }
        break;
    case 0x10008: /* DMA Command */
        val &= 0x88000000;
        if(val&0x80000000) {
            if(fifo_full(&pico->cmd)) {
                qemu_log_mask(LOG_GUEST_ERROR, "DMA command queue overflow.  Ignoring request.\n");
            } else {
                dma_fifo_ent_t *cmd = &pico->cmd.entries[pico->cmd.in];
                cmd->cmd   = val;
                cmd->base  = pico->regval[0x1000C>>2];
                cmd->count = pico->regval[0x10010>>2];
                pico->cmd.in = (pico->cmd.in+1)%NDMA;
                pico_run_dma(pico);
            }
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "DMA command write w/o GO.\n");
        }
        break;
    case 0x1000C: /* DMA Command Start Address */
    case 0x10010: /* DMA Command Count */
        break;
    case 0x10014: /* DMA Response Count */
        if(!fifo_empty(&pico->resp)) {
            pico->resp.out = (pico->resp.out+1)%NDMA;
            /* in case DMA is stalled due to full response fifo */
            pico_run_dma(pico);
        } else {
            qemu_log_mask(LOG_GUEST_ERROR, "pico8: response pop when fifo empty!!!\n");
        }
        val = 0;
        break;
    case 0x20000: /* Mux */
        val &= 3;
        break;
    case 0x3c004: /* DDR page select */
        val &= 0xf;
        break;
    case 0x40010:
        pico->irq_status &= ~val;
        pico_update_irq(pico);
        return;
    case 0x40014:
        pico->irq_mask = val;
        pico_update_irq(pico);
        break;
    case 0x30000 ... 0x33fff:
        if(pico->frib) {
            switch(addr) {
            case 0x30004:
                if(val & (1<<16)) {
                    /* ACK */
                    pico->regval[addr>>2] &= ~(1<<17);
                }
                if(val & (1<<18)) {
                    /* clear missed ACK */
                    if(pico->regval[addr>>2] & (1<<18))
                        DBGOUT("Reset Missed ACK\n");
                    pico->regval[addr>>2] &= ~(1<<18);
                }
                pico_update_irq(pico);
                return;
            case 0x30044:
                DBGOUT("is in AMC slot=%u\n", extract32(val, 16, 4));
                if(val&0x800) {
                    /* FRIB logic reset */
                    pico->regval[addr>>2] = 0;
                }
                val &= 0xff0ff000;
                break;
            default:
                qemu_log_mask(LOG_GUEST_ERROR, "pico8: write to undefined FRIB register "TARGET_FMT_plx"\n", addr);
            }
            break;
        }
        /* fall through */
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "pico8: write to undefined register "TARGET_FMT_plx"\n", addr);
        return;
    }

    pico->regval[addr>>2] = val;
}

static const MemoryRegionOps mmio_ops = {
    .read = pico8_mmio_read,
    .write = pico8_mmio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void
pico8_ddr_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    uint32_t temp;
    MemTxResult ok;
    PicoState *pico = opaque;
    unsigned align = addr&3;
    uint32_t mask = (1ull << (size*8))-1;

    DBGOUT("pico8 ddr write%u %08x <- %08x | ", size, (unsigned)addr, (unsigned)val);

    /* pico8 doesn't correctly handle unaligned access across word boundaries.
     * Each operation accesses only a single RAM word.
     */
    addr -= align;
    addr += PICO_DDR_PAGE*(pico->regval[0x3c004>>2]&0xf); // page select

    DBGOUTx("%08x ->", (unsigned)addr);

    temp = address_space_ldl_le(&pico->mram_as, addr, MEMTXATTRS_UNSPECIFIED, &ok);

    DBGOUTx("%08x ->", (unsigned)temp);

    if(ok!=MEMTX_OK) {
        DBGOUTx(" X\n");
        qemu_log_mask(LOG_GUEST_ERROR, "pico8 DDR write/read error %08x\n", (unsigned)addr);
        return;
    }

    temp = ror32(temp, align*8)&~mask;

    DBGOUTx("%08x ->", (unsigned)temp);

    temp |= val&mask;

    DBGOUTx("%08x ->", (unsigned)temp);

    temp = rol32(temp, align*8);

    DBGOUTx("%08x\n", (unsigned)temp);

    address_space_stl_le(&pico->mram_as, addr, temp, MEMTXATTRS_UNSPECIFIED, &ok);

    if(ok!=MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "pico8 DDR write error %08x\n", (unsigned)addr);
        return;
    }
}

static uint64_t
pico8_ddr_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t ret;
    MemTxResult ok;
    unsigned align = addr&3;
    PicoState *pico = opaque;

    DBGOUT("pico8 ddr read%u %08x | ", size, (unsigned)addr);

    addr -= align;
    addr += PICO_DDR_PAGE*(pico->regval[0x3c004>>2]&0xf); // page select
    DBGOUTx("%08x ->", (unsigned)addr);

    ret = address_space_ldl_le(&pico->mram_as, addr, MEMTXATTRS_UNSPECIFIED, &ok);

    if(ok!=MEMTX_OK) {
        DBGOUTx(" X\n");
        qemu_log_mask(LOG_GUEST_ERROR, "pico8 DDR read error %08x\n", (unsigned)addr);
        return 0xffffffff;
    }
    DBGOUTx("%08x ->", (unsigned)ret);

    ret = ror32(ret, align*8);
    DBGOUTx("%08x\n", (unsigned)ret);

    return ret;
}

static const MemoryRegionOps ddr_ops = {
    .read = pico8_ddr_read,
    .write = pico8_ddr_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
    },
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
        .unaligned = true,
    },
};

static
void pico_frib_update_sum_record(void *opaque)
{
    unsigned ch;
    PicoState *pico = opaque;
    const uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_HOST),
                  vnow = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT);

    pico->next_time += NANOSECONDS_PER_SECOND;
    if(pico->next_time<=vnow) {
        fprintf(stderr, "Oops, pico8 record update is lagging...\n");
        pico->next_time = vnow + NANOSECONDS_PER_SECOND;
    }
    timer_mod(&pico->sumtimer, pico->next_time);

    if(pico->regval[0x30004>>2] & (1<<17)) {
        if(!(pico->regval[0x30004>>2] & (1<<18)))
            DBGOUT("Missed ACK\n");
        pico->regval[0x30004>>2] |= 1<<18; // missed ACK
        return;
    }
    pico->regval[0x30004>>2] |= 1<<17; // waiting for ACK

    pico->regval[0x30100>>2] = now/NANOSECONDS_PER_SECOND; // [sec]
    pico->regval[0x30104>>2] = now%NANOSECONDS_PER_SECOND; // [ns]
    pico->regval[0x30104>>2] *= 80.5e6/1e9; // [ns] * [cycles/ns] -> [cycles]

    /* Time on */
    pico->regval[0x30108>>2] = 1e-3 * 80.5e6;

    for(ch=0; ch<8; ch++) {
        unsigned base = (0x30110 + ch*0x14)>>2;

        pico->regval[base+0] = f2i(ch*1e-9); // total charge (sum)
        pico->regval[base+1] = f2i(ch*1e-9-5e-10); // min
        pico->regval[base+2] = f2i(ch*1e-9+5e-10); // max
        pico->regval[base+3] = f2i(2e-10); // std
        pico->regval[base+4] = f2i(1e-11); // offset?
    }

    pico_update_irq(pico);
}

static void caen_pico8_reset(DeviceState *dev)
{
    PicoState *pico = PICO8(dev);

    pico->start_time = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL_RT);
    pico->next_time = pico->start_time+NANOSECONDS_PER_SECOND;

    memset(&pico->regval, 0, sizeof(pico->regval));

    pico->regval[0x78>>2] = 0x001000a; /* version */
    pico->regval[0x7c>>2] = 1471472803u; /* build time (posix sec) */

    pico->irq_mask = pico->irq_status = 0;
    pico_update_irq(pico);

    pico8_dma_reset(pico);

    if(pico->frib) {
        timer_mod_ns(&pico->sumtimer, pico->next_time);
    }
}

static void caen_pico8_realize(PCIDevice *dev, Error **errp)
{
    PicoState *pico = PICO8(dev);
    uint8_t *conf;
    int pos;
    /* 07:00.0 Signal processing controller: Xilinx Corporation Default PCIe endpoint ID
     *         Subsystem: Device cae2:71c0
     *         Control: I/O+ Mem+ BusMaster+ SpecCycle- MemWINV- VGASnoop- ParErr- Stepping- SERR- FastB2B- DisINTx+
     *         Status: Cap+ 66MHz- UDF- FastB2B- ParErr- DEVSEL=fast >TAbort- <TAbort- <MAbort- >SERR- <PERR- INTx-
     *         Latency: 0, Cache Line Size: 64 bytes
     *         Interrupt: pin A routed to IRQ 69
     */

    dev->config[PCI_INTERRUPT_PIN] = 1; /* INTA */

    dev->config[PCI_CACHE_LINE_SIZE] = 0x10; /* 64 bytes */

    if (pci_is_express(dev)) {
        /* Capabilities: [60] Express (v1) Endpoint, MSI 00
         *         DevCap: MaxPayload 512 bytes, PhantFunc 1, Latency L0s unlimited, L1 unlimited
         *                 ExtTag+ AttnBtn- AttnInd- PwrInd- RBE+ FLReset-
         *         DevCtl: Report errors: Correctable- Non-Fatal- Fatal- Unsupported-
         *                 RlxdOrd+ ExtTag- PhantFunc- AuxPwr- NoSnoop+
         *                 MaxPayload 128 bytes, MaxReadReq 512 bytes
         *         DevSta: CorrErr- UncorrErr- FatalErr- UnsuppReq- AuxPwr- TransPend-
         *         LnkCap: Port #0, Speed 2.5GT/s, Width x4, ASPM L0s, Latency L0 unlimited, L1 unlimited
         *                 ClockPM- Surprise- LLActRep- BwNot-
         *         LnkCtl: ASPM Disabled; RCB 64 bytes Disabled- Retrain- CommClk-
         *                 ExtSynch- ClockPM- AutWidDis- BWInt- AutBWInt-
         *         LnkSta: Speed 2.5GT/s, Width x4, TrErr- Train- SlotClk+ DLActive- BWMgmt- ABWMgmt-
         */
        pcie_endpoint_cap_init(dev, 0x60);
    } else {
        fprintf(stderr, "Warning: pico8: PCIe device connected to PCI (not express) bus.\n");
    }
    /*  Capabilities: [48] MSI: Enable+ Count=1/1 Maskable- 64bit+
     *          Address: 00000000fee004b8  Data: 0000
     */
    msi_init(dev, 0x48, pico->frib ? 32 : 1, true, false, errp);

    if (pci_is_express(dev)) {
        pos = PCI_CONFIG_SPACE_SIZE;
        // Capabilities: [100 v1] Device Serial Number 00-00-00-01-01-00-0a-35
        pcie_add_capability(dev, PCI_EXT_CAP_ID_DSN, 1,
                            pos,
                            PCI_EXT_CAP_DSN_SIZEOF);

        conf = dev->config + pos + 4;

        /* S/N 00-00-00-01-01-00-0a-30 */
        pci_set_quad(conf, 0x0000000101000a30ull);
    }

    /* Region 0: Memory at 91000000 (32-bit, non-prefetchable) [size=2M]
     * Region 2: Memory at 90000000 (32-bit, non-prefetchable) [size=16M]
     * Expansion ROM at 91b00000 [disabled] [size=1M]
     */

    memory_region_init_io(&pico->mmio, OBJECT(dev), &mmio_ops, pico,
                          "pico8-reg", 2*1024*1024);

    /* on board DDR ram */
    memory_region_init_ram(&pico->mram, OBJECT(dev), "pico8-dram", PICO_DDR_PAGES*PICO_DDR_PAGE, &error_fatal);
    address_space_init(&pico->mram_as, &pico->mram, "pico8-ddr-as");

    /* BAR 2 provides access to one RAM page */
    memory_region_init_io(&pico->mram_access, OBJECT(dev), &ddr_ops, pico,
                          "pico8-dram-page", PICO_DDR_PAGE);
    /* ROM not modeled */

    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &pico->mmio);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &pico->mram_access);

    if(pico->frib) {
        timer_init_ns(&pico->sumtimer, QEMU_CLOCK_VIRTUAL_RT, &pico_frib_update_sum_record, pico);
    }
}

static
void caen_pico8_unrealize(PCIDevice *dev)
{
    PicoState *pico = PICO8(dev);
    if(pico->frib) {
        timer_del(&pico->sumtimer);
    }
}

static Property pico8_props[] = {
    DEFINE_PROP_BOOL("frib", PicoState, frib, false),
    DEFINE_PROP_END_OF_LIST()
};

static void caen_pico8_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->vendor_id = PCI_VENDOR_ID_XILINX;
    k->device_id = 0x0007;
    k->subsystem_vendor_id = 0xCAE2;
    k->subsystem_id = 0x71C0;
    k->class_id = 0x1180;
    k->is_express = true;
    k->revision = 0;
    k->realize = &caen_pico8_realize;
    k->exit = &caen_pico8_unrealize;
    dc->reset = &caen_pico8_reset;
    dc->props = pico8_props;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo caen_pico8_type = {
    .name = TYPE_CAEN_PICO8,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PicoState),
    .class_size = sizeof(PCIDeviceClass),
    .class_init = caen_pico8_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { },
    },
};

static void caen_pico8_register(void)
{
    type_register_static(&caen_pico8_type);
}

type_init(caen_pico8_register)
