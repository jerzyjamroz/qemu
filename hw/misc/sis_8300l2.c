
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "sysemu/dma.h"

#define TYPE_SIS8300L2 "sis8300l2"

#if 0
#define DO_DEBUG
#define	DBGOUT(fmt, ...) fprintf(stderr, TYPE_SIS8300L2 ": " fmt, ## __VA_ARGS__)
#else
#define	DBGOUT(fmt, ...) do {} while (0)
#endif

#define LOG(mask, fmt, ...) qemu_log_mask(mask, TYPE_SIS8300L2 ": " fmt, ##__VA_ARGS__)

#define NCHAN 10

#define NSAMPPERBLK 16

typedef struct {
    PCIDevice parent_obj;

    MemoryRegion mmio;

    uint32_t reg[0x500];

    uint8_t *memory;
    size_t memlen; /* actual allocated length */
} SISState;

#define SIS8300L2(obj) OBJECT_CHECK(SISState, (obj), TYPE_SIS8300L2)

#ifdef DO_DEBUG
// Mux A feeds mux C and D
static const char* muxa[4] = {"RTMCLK_CLK2", "TCLKB", "TCLKA", "Crystal"};
// Mux B feeds mux C and E
static const char* muxb[4] = {"RTMCLK_CLK2", "TCLKB", "TCLKA", "Crystal"};
// Mux C feeds multiplier
static const char* muxc[4] = {"Harlink", "SMA", "MUXA", "MUXB"};
// Mux D feeds AD9510 #2
static const char* muxd[4] = {"MUXA", "MULT", "SMA", "Harlink"};
// Mux E feeds AD9510 #1
static const char* muxe[4] = {"MUXB", "MULT", "SMA", "Harlink"};

static const char* adcmd[4] = {"NOOP", "R/W", "Func", "Rsrv"};
static const char* sicmd[4] = {"R/W", "Rst", "Dec", "Inc"};
#endif

static
void sis_update_irq(SISState *state)
{
    uint32_t ena = state->reg[0x880>>2],
             act = state->reg[0x884>>2];
    unsigned irq = !!(ena&act);

    DBGOUT("IRQ ena %08x act %08x irq %s\n",
           (unsigned)ena, (unsigned)act,
           irq?"ON":"OFF");

    pci_set_irq(&state->parent_obj, irq);
}

static
void sis_do_dma_read(SISState *state)
{
    uint32_t dstl = state->reg[0x800>>2],
             dsth = state->reg[0x804>>2],
             src  = state->reg[0x808>>2],
             cnt  = state->reg[0x80c>>2];
    dma_addr_t dst = ((uint64_t)dsth<<32)|dstl;

    if(src>state->memlen) {
        qemu_log_mask(LOG_GUEST_ERROR, TYPE_SIS8300L2": DMA starts past end of memory! %08x %08x\n",
                      (unsigned)src, (unsigned)state->memlen);

    } else if(src+cnt>state->memlen) {
        qemu_log_mask(LOG_GUEST_ERROR, TYPE_SIS8300L2": DMA ends past end of memory! %08x %08x\n",
                      (unsigned)(src+cnt), (unsigned)state->memlen);

    } else if(cnt>0) {
        DBGOUT("DMA Read to "TARGET_FMT_plx" from %08x count %u\n",
               dst, (unsigned)src, (unsigned)cnt);
        if(pci_dma_write(&state->parent_obj,
                      dst, &state->memory[src], cnt)) {
            qemu_log_mask(LOG_GUEST_ERROR, TYPE_SIS8300L2": DMA write fails\n");
        }
        state->reg[0x808>>2] += cnt; /* auto-advance the source address */

    } else {
        DBGOUT(" DMA length 0\n");
    }

    state->reg[0x884>>2] |= (1<<0); /* DMA Read Done */
    sis_update_irq(state);
}

static
uint16_t sis_gen_sample(unsigned int chan, uint32_t samp)
{
    return 0x7fff+0x100*(chan-5)+samp%100;
}

static
void sis_acquire(SISState *state)
{
    unsigned int ch;
    unsigned int bswap = state->reg[0x814>>2]&1; /* 0 - LSB, 1 - MSB */
    uint32_t sampctrl = state->reg[0x44>>2],
             nsamp = (state->reg[0x4a8>>2]+1)<<4; /* # samples */

    DBGOUT("Acquire %u samples\n", (unsigned)nsamp);

    for(ch=0; ch<NCHAN; ch++) {
        unsigned n;
        const uint32_t start = state->reg[(0x480+4*ch)>>2]*NSAMPPERBLK; /* first sample */
        const size_t needsize = 2*(start+nsamp); /* bytes */
        uint16_t *next;

        if(sampctrl&(1<<ch)) {
            DBGOUT("chan %u disabled\n", ch);
            continue;
        }
        DBGOUT("chan %u starts at %08x\n", ch, (unsigned)2*start);

        if(start+nsamp>0x3fffffff) {
            qemu_log_mask(LOG_GUEST_ERROR, TYPE_SIS8300L2": Can't acquire past end of memory!\n");
            continue;

        } else if(needsize>state->memlen) {
            uint8_t *newptr;
            DBGOUT("Reallocate memory for 0x%08x bytes\n", (unsigned)needsize);
            newptr = realloc(state->memory, needsize);
            if(!newptr) {
                hw_error("SIS could not allocate %u bytes for backing store\n",
                         (unsigned)needsize);
            }
            state->memory = newptr;
            state->memlen = needsize;
        }

        next = (uint16_t*)&state->memory[2*start];

        for(n=0; n<nsamp; n++, next++) {
            uint16_t samp = sis_gen_sample(ch, n);
            /* we're cheating here since byte swap should happen during DMA (read)
             * not during store.
             * Assume guest isn't going to change this bit between acquire start
             * and DMA start.
             */
            if(bswap)
                samp = __bswap_16(samp);
            *next = samp;
        }

        state->reg[(0x480+4*ch)>>2] += nsamp/NSAMPPERBLK;
    }

    state->reg[0x40>>2] = 0x80; /* clear arm and busy */
    state->reg[0x884>>2] |= (1<<14); /* DAQ Done */
    sis_update_irq(state);

    if(state->reg[0x858>>2]&1) {
        DBGOUT("Chain DAQ to DMA\n");
        sis_do_dma_read(state);
    }
}

static uint64_t
sis_read(void *opaque, hwaddr addr, unsigned size)
{
    SISState *state = opaque;
    uint64_t ret = state->reg[addr>>2];

    if(addr!=0x880 && addr!=0x884) /* exclude IRQ enable and status */
        DBGOUT("read "TARGET_FMT_plx" -> %08x\n", addr, (unsigned)ret);

    if(addr==0x104 || addr==0x120) {
        /* Clear busy on first read after write.
         * First read shows busy, second read shows done
         */
        state->reg[addr>>2] &= ~0x80000000;
    }

    return ret;
}

static void
sis_write(void *opaque, hwaddr addr, uint64_t val,
                 unsigned size)
{
    SISState *state = opaque;

    DBGOUT("write "TARGET_FMT_plx" <- %08x\n", addr, (unsigned)val);

    switch(addr) {
    case 0x10: /* User CSR */
        val &= 0x00030003;
        state->reg[addr>>2] &= ~(val>>16);
        state->reg[addr>>2] |= val&0xffff;
        return;
    case 0x40: /* ADC Acq. CSR */
        if(val&4)
            state->reg[addr>>2] = 0x80; /* clear arm and busy */
        if(val&2)
            state->reg[addr>>2] = 0x82; /* arm */
        if(val&1) {
            state->reg[addr>>2] = 0x91; /* busy */
            sis_acquire(state);
        }
        return;
    case 0x44: /* ADC Sample Control */
        break; /* just store */
    case 0x100: /* Clock Dist. Mux. Control */
        val &= 0x00000f3f;
    {
        unsigned temp=val;
        DBGOUT(" MUXA %s (%u)\n", muxa[temp&3], temp&3);
        temp>>=2;
        DBGOUT(" MUXB %s (%u)\n", muxb[temp&3], temp&3);
        temp>>=2;
        DBGOUT(" MUXC %s (%u)\n", muxc[temp&3], temp&3);
        temp>>=4;
        DBGOUT(" MUXD %s (%u)\n", muxd[temp&3], temp&3);
        temp>>=2;
        DBGOUT(" MUXE %s (%u)\n", muxe[temp&3], temp&3);
    }
        break; /* just store */
    case 0x104: /* Clock Dist. AD9510 SPI */
        DBGOUT("AD9510 #%c %s addr=%03x data=%02x\n",
               (val&(1<<24))?'2':'1',
               adcmd[val>>30], (unsigned)((val>>8)&0xfff),
                (unsigned)(val&0xff));
        if(state->reg[addr>>2]&0x80000000) {
            LOG(LOG_GUEST_ERROR, "AD9510 Start OP w/o busy test\n");
        }
        state->reg[addr>>2] = 0xa00000fe; /* TODO */
        return;
    case 0x108: /* Clock Mult. SI5326 SPI */
        DBGOUT("SI5326 %s addr=%03x data=%02x\n",
               sicmd[val>>30], (unsigned)((val>>8)&0xff),
                (unsigned)(val&0xff));
        state->reg[addr>>2] = 0; /* TODO */
        return;
    case 0x10c: /* Clock Synth. SI5338A Ctrl */
        DBGOUT("SI5338A%s%s%s%s%s%s %08x\n",
               (val&(1<<13))?" R":"",
               (val&(1<<12))?" W":"",
               (val&(1<<11))?" STOP":"",
               (val&(1<<10))?" REP":"",
               (val&(1<<9))?" START":"",
               (val&(1<<8))?" ACK":"",
               (unsigned)(val&0xff)
               );
        state->reg[addr>>2] = 0; /* TODO */
        return;
    case 0x120: /* ADC SPI */
        DBGOUT("ADC %c %u %03x <- %02x%s\n",
               (val&(1<<23))?'R':'W',
               (unsigned)((val>>24)&0x7),
               (unsigned)((val>>8)&0xfff),
               (unsigned)(val&0xff),
               (val&(1<<31))?" Synch":""
               );
        if(state->reg[addr>>2]&0x80000000) {
            LOG(LOG_GUEST_ERROR, "ADC Start OP w/o busy test\n");
        }
        state->reg[addr>>2] = 0x800000fe; /* TODO */
        return;
    case 0x400 ... 0x424: /* Trigger Setup */
    case 0x440 ... 0x464: /* Trigger Threshold */
    case 0x480 ... 0x4a4: /* Memory Sample Start Address */
    case 0x4a8: /* Sample Length */
    case 0x4ac: /* Ring buffer delay */
    case 0x800 ... 0x80c: /* DMA Read addrs and count */
        break; /* just store */
    case 0x810: /* DMA Read Control */
        if(val&1) {
            sis_do_dma_read(state);
        }
        break;
    case 0x814: /* DMA Read Endian */
    case 0x840 ... 0x84C: /* DMA Write addrs and count */
        break; /* just store */
    case 0x850: /* DMA Write control */
        if(val&1) {
            qemu_log_mask(LOG_UNIMP, "DMA Write unimplemented\n");
            state->reg[0x884>>2] |= (1<<1); /* DMA Write Done */
            sis_update_irq(state);
        }
        break;
    case 0x858: /* DAQ to DMA chain */
        break; /* just store */
    case 0x880: /* IRQ Enable */
        val &= 0xc003c003;
        state->reg[addr>>2] &= ~(val>>16);
        state->reg[addr>>2] |= val&0xffff;
        sis_update_irq(state);
        return;
    case 0x884: /* IRQ Status */
        return; /* R/O */
    case 0x888: /* IRQ Clear (Ack.) */
        val &= 0xc003;
        state->reg[0x884>>2] &= ~val;
        sis_update_irq(state);
        return;
    case 0x88c: /* IRQ Refresh */
        qemu_log_mask(LOG_UNIMP, "IRQ Refresh is a no-op?\n");
        return;
    /* 0x8c0 - Undocumented DDR2_ACCESS_CONTROL */
    case 0x1000 ... 0x13fc: /* User registers */
        break; /* just store */
    default:
        qemu_log_mask(LOG_UNIMP, "Register "TARGET_FMT_plx" not implemented\n", addr);
        return;
    }

    state->reg[addr>>2] = val;
}

static const MemoryRegionOps mmio_ops = {
    .read = sis_read,
    .write = sis_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void sis_reset(DeviceState *dev)
{
    SISState *state = SIS8300L2(dev);

    state->reg[0x00>>2] = 0x83021809; /* ID */
    state->reg[0x04>>2] = 0x45; /* S/N */
    /* FW Options.
     *  Real L2 reports 0xC1090152:
     *   DUAL_OPTICAL_INTERFACE_EN         - 0x40
     *   QUAD_PORT12_13_14_15_INTERFACE_EN - 0x10
     *   RINGBUFFER_DELAY_EN               - 0x02
     * And some bits documented as reserved.
     * We advertise only the documented bits.
     * And we implement TRIGGER_BLOCK_EN (0x01) since
     * it appears to exist even when un-advertised :P
     */
    state->reg[0x14>>2] = 0x52;
    state->reg[0x40>>2] = 0x80; /* ADC Acq. CSR */
    state->reg[0x104>>2] = 0x20000000; /* Clock Dist. AD9510 SPI */
}

static void sis_realize(PCIDevice *dev, Error **errp)
{
    SISState *state = SIS8300L2(dev);

    dev->config[PCI_INTERRUPT_PIN] = 1; /* INTA */
    dev->config[PCI_CACHE_LINE_SIZE] = 0x10; /* 64 bytes */

    if (pci_is_express(dev)) {
        pcie_endpoint_cap_init(dev, 0x60);
    }

    memory_region_init_io(&state->mmio, &dev->qdev.parent_obj, &mmio_ops, state,
                          TYPE_SIS8300L2, 16*1024);
    assert(sizeof(state->reg)<=16*1024);

    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &state->mmio);
}

static void sis_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->vendor_id = 0x1796;
    k->device_id = 0x0019;
    k->subsystem_vendor_id = 0x1796;
    k->subsystem_id = 0x0019;
    k->class_id = 0xff00;
    k->is_express = true;
    k->revision = 0;
    k->realize = &sis_realize;
    dc->reset = &sis_reset;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo sis8300l2_type = {
    .name = TYPE_SIS8300L2,
    .parent = TYPE_PCI_DEVICE,
    .instance_size = sizeof(SISState),
    .class_size = sizeof(PCIDeviceClass),
    .class_init = sis_class_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_PCIE_DEVICE },
        { },
    },
};

static void sis_register(void)
{
    type_register_static(&sis8300l2_type);
}

type_init(sis_register)
