/*
 * VME Bus infrastructure
 *
 * Copyright (c) 2015 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
#ifndef VME_BUS_H
#define VME_BUS_H


#define TYPE_VME_BUS "vme-bus"
#define VME_BUS(i) OBJECT_CHECK(VMEBus, i, TYPE_VME_BUS)
#define VME_BUS_CLASS(k) OBJECT_CLASS_CHECK(VMEBusClass, k, TYPE_VME_BUS)

#define TYPE_VME "vme"
#define VME(i) OBJECT_CHECK(VMEDevice, i, TYPE_VME)
#define VME_CLASS(k) OBJECT_CLASS_CHECK(VMEDeviceClass, k, TYPE_VME)
#define VME_GET_CLASS(i) \
     OBJECT_GET_CLASS(VMEDeviceClass, (i), TYPE_VME)

/* Max. # of VME cards in a full size crate.
 * numbered 1 through NUM_VME.
 */
#define NUM_VME 21

/* # of IRQ lines */
#define NUM_VME_IRQ 7

/*VME address modifiers (AMs)
 * 0bxxxxx___ - Address width/operation type
 * 0b_____x__ - 1 - Supervisor, 0 - non privlaged
 * 0b______xx - cycle type (MBLT, DAT, PRG, BLT) for != A16 or CSR
 *
 * We consider that an address space is uniquely
 * identified by the 6 high bits.
 * Slaves which ignore the supervisor bit should
 * add themselves to both address spaces
 */

#define VME_AM_BIT_SUP 0x4

#define VME_AM_MSK_BLT 0x3
#define VME_AM_MSK_PRG 0x2
#define VME_AM_MSK_DAT 0x1
#define VME_AM_MSK_MBLT 0x0

#define VME_AM_MSK_OPER 0xf8
#define VME_AM_MSK_A16 0x28
#define VME_AM_MSK_A24 0x38
#define VME_AM_MSK_A32 0x08

#define VME_AM_MSK_SPACE (VME_AM_MSK_OPER | VME_AM_BIT_SUP)

#define VME_AM_CRCSR 0x2f

#define VME_AM_A16_SUP 0x2d
#define VME_AM_A16_USR 0x29

#define VME_AM_CRCSR 0x2f

#define VME_AM_A24_SUP_BLT 0x3f
#define VME_AM_A24_SUP_PRG 0x3e
#define VME_AM_A24_SUP_DAT 0x3d
#define VME_AM_A24_SUP_MBLT 0x3c
#define VME_AM_A24_USR_BLT 0x3b
#define VME_AM_A24_USR_PRG 0x3a
#define VME_AM_A24_USR_DAT 0x39
#define VME_AM_A24_USR_MBLT 0x38

#define VME_AM_A32_SUP_BLT 0xf
#define VME_AM_A32_SUP_PRG 0xe
#define VME_AM_A32_SUP_DAT 0xd
#define VME_AM_A32_SUP_MBLT 0xc
#define VME_AM_A32_USR_BLT 0xb
#define VME_AM_A32_USR_PRG 0xa
#define VME_AM_A32_USR_DAT 0x9
#define VME_AM_A32_USR_MBLT 0x8

/* VME64 CSR/CR standard registers */

/* TODO: how is checksum calculated? */
#define  VME_CR_ROM_CHECKSUM           0x0003
#define  VME_CR_ROM_LENGTH             0x0007

#define  VME_CR_DATA_ACCESS_WIDTH      0x0013
#define  VME_CSR_DATA_ACCESS_WIDTH     0x0017

/* Effectively identify complience with VME64 (1) or VME64x (2) */
#define  VME_CR_SPACE_ID               0x001B
#  define VME_CR_SPACE_ID_VME64 1
#  define VME_CR_SPACE_ID_VME64x 2

/* ID test registers.  Must read 'C' and 'R' */
#define  VME_CR_ASCII_C                0x001F
#define  VME_CR_ASCII_R                0x0023

#define  VME_CR_IEEE_OUI               0x0027
#define  VME_CR_IEEE_OUI_BYTES         3
#define  VME_CR_BOARD_ID               0x0033
#define  VME_CR_BOARD_ID_BYTES         4
#define  VME_CR_REVISION_ID            0x0043
#define  VME_CR_REVISION_ID_BYTES      4
#define  VME_CR_ASCII_STRING           0x0053
#define  VME_CR_PROGRAM_ID             0x007F

/* VME64x CSR/CR standard registers */

/* Offset of manufacturer-defined CR space */
#define  VME_CR_BEG_UCR                0x0083
#define  VME_CR_END_UCR                0x008F
#define  VME_CR_BEG_UCSR_BYTES         3

/* Offset of Configuration RAM (CRAM) space */
#define  VME_CR_BEG_CRAM               0x009B
#define  VME_CR_END_CRAM               0x00A7

/* Offset of manufacturer-defined CSR space */
#define  VME_CR_BEG_UCSR               0x00B3
#define  VME_CR_END_UCSR               0x00BF

/* Offset of board serial number */
#define  VME_CR_BEG_SN                 0x00CB
#define  VME_CR_END_SN                 0x00DF

#define  VME_CR_SLAVE_CHAR             0x00E3
#define  VME_CR_UD_SLAVE_CHAR          0x00E7

#define  VME_CR_MASTER_CHAR            0x00EB
#define  VME_CR_UD_MASTER_CHAR         0x00EF

/* Interrupt levels board can respond to (handle) */
#define  VME_CR_IRQ_HANDLER_CAP        0x00F3
/* Interrupt levels board can assert.
 * Bit # is IRQ line # (aka bit 0 not used)
 */
#define  VME_CR_IRQ_CAP                0x00F7

#define  VME_CR_CRAM_WIDTH             0x00FF

/* Data Access Width Parameter (DAWPR) regs. N = 0 -> 7 */
#define  VME_CR_FN_DAWPR(N)  (0x0103 + (N) * 0x04)
#define  VME_CR_DAWPR_BYTES  1

/* Address Mode Capability (AMCAP) registers.  N = 0 -> 7 */
#define  VME_CR_FN_AMCAP(N)  (0x0123 + (N) * 0x20)
#define  VME_CR_AMCAP_BYTES  8

/* Extended Address Mode Cap (XAMCAP) registers.  N = 0 -> 7 */
#define  VME_CR_FN_XAMCAP(N) (0x0223 + (N) * 0x80)
#define  VME_CR_XAMCAP_BYTES 32

/* Address Decoder Mask (ADEM) registers.  N = 0 -> 7 */
#define  VME_CR_FN_ADEM(N)   (0x0623 + (N) * 0x10)
#define  VME_CR_ADEM_BYTES   4

/* Master Data Access Width Parameter */
#define  VME_CR_MASTER_DAWPR           0x06AF
/* Master Address Mode Capabilities   (8 entries) */
#define  VME_CR_MASTER_AMCAP           0x06B3
/* Master Extended Address Mode Capabilities (8 entries) */
#define  VME_CR_MASTER_XAMCAP          0x06D3

/* Size in bytes of CR space */
#define  VME_CR_SIZE                          0x0750
/* actual number of accessible bytes */
#define  VME_CR_BYTES                   (VME_CR_SIZE >> 2)

/* VME64 required CSR registers */

/* Base Address Register (MSB of our CR/CSR address) */
#define  VME_CSR_BAR                0x7ffff
#define  VME_CSR_BIT_SET            0x7fffb
#define  VME_CSR_BIT_CLEAR          0x7fff7
/* set when card in reset */
#  define  VME_CSR_BIT_RESET_MODE           0x80
/* set while card asserts SYSFAIL */
#  define  VME_CSR_BIT_SYSFAIL_ENA          0x40
#  define  VME_CSR_BIT_MODULE_FAIL          0x20
#  define  VME_CSR_BIT_MODULE_ENA           0x10
/* set after card has asserted BERR */
#  define  VME_CSR_BIT_BERR                 0x08
#  define  VME_CSR_BIT_CRAM_OWNED           0x04

/* VME64x required CSR registers */

/* Configuration RAM Owner Flag Register (0 = not owned) */
#define  VME_CSR_CRAM_OWNER         0x7fff3
/* user defined bit mask */
#define  VME_CSR_UD_BIT_SET         0x7ffef
#define  VME_CSR_UD_BIT_CLEAR       0x7ffeb

/* Function N Address Decoder Compare Register. N = 0 -> 7 */
#define  VME_CSR_FN_ADER(N) (0x7ff63 + (N) * 0x10)
#define  VME_CSR_ADER_BYTES 4


struct VMEBus;
struct VMEDevice;

typedef struct {
    DeviceClass parent_class;

    void (*realize)(struct VMEDevice *dev, Error **errp);
    void (*unrealize)(struct VMEDevice *dev, Error **errp);
} VMEDeviceClass;

typedef struct VMEDevice {
    DeviceState qdev;

    /* Slot # in VME crate.
     * 1 is first slot.
     * 0 is unknown slot
     */
    uint8_t slot;

    struct VMEBus *bus;

    /* Called during an IACK cycle to determine
     * if this device is asserting an interrupt,
     * and which vector code it asserts.
     * ROAK device should deassert the IRQ
     * in this callback
     */
    bool (*iack)(struct VMEDevice *dev, uint8_t lvl, uint32_t *pvect);
    uint8_t irq_status; /* bit mask of asserted IRQ lines */

    bool sysfail; /* is this device asserting sysfail? */
} VMEDevice;

typedef struct {
    AddressSpace space;
    MemoryRegion root;
    struct VMEBus *bus;
    uint8_t amod;
} VMEAddressSpace;

typedef struct {
    BusClass parent_class;
} VMEBusClass;

typedef struct VMEBus {
    BusState qbus;

    VMEAddressSpace *spaces[256];

    VMEDevice *devices[NUM_VME + 1];

    qemu_irq irq[NUM_VME_IRQ];

    struct berr_t {
        uint8_t amod;
        hwaddr addr;
        unsigned size;
        bool write;
        void (*cb)(struct VMEBus*, void *arg);
        void *cb_arg;
    } berr;
} VMEBus;

void vme_bus_init(VMEBus *pbus, size_t bsize, DeviceState *parent);

MemoryRegion *vme_bus_get_region(VMEBus *pbus, uint8_t amod);
AddressSpace *vme_bus_get_space(VMEBus *pbus, uint8_t amod);

/* Perform IACK cycle.  Useful to bus bridge */
bool vme_bus_get_iack(VMEBus *pbus, uint8_t level, uint32_t *pvect);

/* Expose a memory region from this device. */
void vme_add_region(VMEDevice *dev, uint8_t amod, hwaddr base,
                    MemoryRegion *reg);
/* Remove a region, if mapped */
void vme_del_region(VMEDevice *dev, MemoryRegion *reg);

/* Assert/deassert IRQ level
 * level in range [1, 7]
 */
void vme_set_irq(VMEDevice *dev, unsigned irqlevel, unsigned val);

#endif /* VME_BUS_H */
