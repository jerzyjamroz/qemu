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

#include <hw/hw.h>

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

bool vme_bus_get_iack(VMEBus *pbus, uint8_t level, uint32_t *pvect);

void vme_add_region(VMEDevice *dev, uint8_t amod, hwaddr base,
                    MemoryRegion *reg);
void vme_del_region(VMEDevice *dev, uint8_t amod, MemoryRegion *reg);

void vme_set_irq(VMEDevice *dev, unsigned irqlevel, unsigned val);

#endif /* VME_BUS_H */
