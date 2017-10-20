#ifndef PPCE500_H
#define PPCE500_H

#include "hw/boards.h"

typedef struct PPCE500Params {
    int pci_first_slot;
    int pci_nr_slots;

    /* required -- must at least add toplevel board compatible */
    void (*fixup_devtree)(struct PPCE500Params *params, void *fdt);

    int mpic_version;
    uint32_t mpc_model; /* eg. 8540, or zero for other OEMs */
    bool has_platform_bus;
    hwaddr platform_bus_base;
    hwaddr platform_bus_size;
    int platform_bus_first_irq;
    int platform_bus_num_irqs;
    hwaddr ccsrbar_base;
    hwaddr pci_pio_base;
    hwaddr pci_mmio_base;
    hwaddr pci_mmio_bus_base;
    hwaddr spin_base;
    uint32_t porpllsr; /* value of PORPLLSR register */
    uint32_t decrementor_freq; /* in Hz */
    bool skip_load;
    bool tsec_nic;
} PPCE500Params;

void ppce500_init(MachineState *machine, PPCE500Params *params);

hwaddr booke206_page_size_to_tlb(uint64_t size);

#endif
