/*
 * Testing of Tundra TSI148 PCI to VME bus bridge emulation
 *
 * Copyright (c) 2016 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
#include <stdio.h>

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/libqos.h"
#include "libqos/pci.h"
#include "libqos/pci-pc.h"

#define assert_equal(A, B) g_assert_cmphex((A), ==, (B))

static QPCIBus *pcibus;
static QPCIDevice *pcidev;
static QPCIBar bar0;
static uint64_t vme16, ram;

static uint8_t bar0readb(uint32_t offset)
{
    uint8_t ival;

    qpci_memread(pcidev, bar0, offset, &ival, sizeof(ival));
    return ival;
}

static uint16_t bar0readw(uint32_t offset)
{
    uint16_t ival;

    qpci_memread(pcidev, bar0, offset, &ival, sizeof(ival));
    return ival;
}

static uint32_t bar0readl(uint32_t offset)
{
    uint32_t ival;

    qpci_memread(pcidev, bar0, offset, &ival, sizeof(ival));
    return ival;
}

static void bar0writel(uint32_t offset, uint32_t val)
{
    qpci_memwrite(pcidev, bar0, offset, &val, sizeof(val));
}

static void save_fn(QPCIDevice *dev, int devfn, void *data)
{
    QPCIDevice **pdev = (QPCIDevice **) data;

    *pdev = dev;
}

static
void test_init(void)
{
    uint64_t bar0size = 0;

    /* assume RAM is at the bottom of the address space.
     * assume it is accessble from PCI w/o transformation.
     */
    ram = 0x10000;

    pcibus = qpci_init_pc(NULL);
    g_assert(pcibus != NULL);

    qpci_device_foreach(pcibus, 0x10e3, 0x0148, save_fn, &pcidev);

    g_assert(pcidev != NULL);

    bar0 = qpci_iomap(pcidev, 0, &bar0size);
    assert_equal(bar0size, 0x1000);

    qpci_device_enable(pcidev);

    assert_equal(bar0readl(0x234), 3);

    bar0writel(0x604, 1 << 27); /* master enable */

    /* The TSI148 responds to addresses programmed through non-standard
     * registers (not BARs) to allow for variable sizes.
     * Assume there is some space left in the PCI host bridge window
     * after where BAR0 is mapped.
     */
    vme16 = bar0.addr + 0x10000;

    /* map the A16 USR DATA address space */
    /* PCI window Start */
    bar0writel(0x100, vme16 >> 32);
    bar0writel(0x104, vme16);
    /* PCI window End (inclusive) */
    bar0writel(0x108, (vme16 + 0xffff) >> 32);
    bar0writel(0x10c, vme16 + 0xffff);
    /* VME offset.  actual address is PCI addr + this offset */
    bar0writel(0x110, -vme16 >> 32);
    bar0writel(0x114, -vme16);

    /* Enable map, disable prefetch, single cycle, D16, no-SUP, no-PGM, A16 */
    bar0writel(0x11c, 0x80040000);

    /* Enable VERR, IACK, and VME* IRQs
     * Must also set bit in INTEO to actually signal IRQ
     */
    bar0writel(0x448, 0x14fe);
}

static
void test_pci_config(void)
{
    printf("VVV %s VVV\n", __func__);
    test_init();
    /* PCI config access through CRG/BAR0 */
    assert_equal(bar0readw(0), 0x10e3); /* vendor */
    assert_equal(bar0readw(2), 0x0148); /* device */
    printf("^^^ %s ^^^\n", __func__);
}

static
void test_mmio(void)
{
    printf("VVV %s VVV\n", __func__);
    test_init();

    /* The TSI148 does implicit byte order swap so that BE
     * registers always appear in native order
     */
    assert_equal(readl(vme16 + 0x1000), 0x12345678);
    assert_equal(readw(vme16 + 0x1000), 0x1234);
    assert_equal(readw(vme16 + 0x1002), 0x5678);
    assert_equal(readb(vme16 + 0x1000), 0x12);
    assert_equal(readb(vme16 + 0x1001), 0x34);
    assert_equal(readb(vme16 + 0x1002), 0x56);
    assert_equal(readb(vme16 + 0x1003), 0x78);

    /* vme-test device records the width of writes to offset 0x8 */
    writel(vme16 + 0x1008, 0);
    assert_equal(readb(vme16 + 0x1008), 4);
    writew(vme16 + 0x1008, 0);
    assert_equal(readb(vme16 + 0x1008), 2);
    writeb(vme16 + 0x1008, 0);
    assert_equal(readb(vme16 + 0x1008), 1);

    printf("^^^ %s ^^^\n", __func__);
}

static
void test_berr(void)
{
    printf("VVV %s VVV\n", __func__);
    test_init();

    assert_equal(bar0readl(0x450), 0);

    assert_equal(bar0readl(0x260), 0);
    assert_equal(bar0readl(0x264), 0);
    assert_equal(bar0readl(0x268), 0);

    /* vme-test only responds to [0x1000, 0x101ff].
     * VME data lines float high when undriven,
     * so read of unused address is defined.
     */
    assert_equal(readb(vme16 + 0x1200), 0xff);

    /* 0x80000000 - VES
     * 0x00080000 - BERR
     * 0x0000ff00 - AMOD (VME_AM_A16_USR)
     * 0x000000ff - not modeled
     */
    assert_equal(bar0readl(0x268), 0x80082900);

    /* the address of the failed read */
    assert_equal(bar0readl(0x260), 0);
    assert_equal(bar0readl(0x264), 0x1200);

    /* BERR IRQ set, but not enabled */
    assert_equal(bar0readl(0x450), 0x1000);

    bar0writel(0x454, 0x1000); /* Clear */

    assert_equal(bar0readl(0x450), 0);

    printf("^^^ %s ^^^\n", __func__);
}

static
void test_vme_irq(void)
{
    printf("VVV %s VVV\n", __func__);
    test_init();

    assert_equal(bar0readl(0x450), 0);

    /* vector 0x42 */
    writel(vme16 + 0x1010, 0x42);
    /* level 3 */
    writel(vme16 + 0x1014, 0x3);
    /* control, IRQ + ROAK */
    writel(vme16 + 0x101c, 0x3);

    /* level 3 active */
    assert_equal(bar0readl(0x450), 1u << 3);

    /* IACK on level 3. ROAK clears */
    assert_equal(bar0readb(0x20c), 0x42);

    /* IACK active */
    assert_equal(bar0readl(0x450), 0x0400);

    /* clear IACK IRQ */
    bar0writel(0x454, 0x400);

    assert_equal(bar0readl(0x450), 0);

    printf("^^^ %s ^^^\n", __func__);
}

static
void test_dma_direct_ram2ram(void)
{
    printf("VVV %s VVV\n", __func__);
    test_init();

    assert_equal(bar0readl(0x450), 0);

    writel(ram + 0x00, 0x01020304);
    writel(ram + 0x04, 0x05060708);
    writel(ram + 0x08, 0x090a0b0c);
    writel(ram + 0x0c, 0x0e0f0102);
    writel(ram + 0x10, 0x00000000);
    writel(ram + 0x14, 0x00000000);
    writel(ram + 0x18, 0x00000000);
    writel(ram + 0x1c, 0x00000000);

    /* source PCI address */
    bar0writel(0x520, 0);
    bar0writel(0x524, ram + 0x0);
    bar0writel(0x530, 0);
    /* dest PCI addr */
    bar0writel(0x528, 0);
    bar0writel(0x52c, ram + 0x10);
    bar0writel(0x534, 0);
    /* count */
    bar0writel(0x540, 4 * 4);

    assert_equal(bar0readl(0x504), 0);

    /* direct GO */
    bar0writel(0x500, (1 << 25) | (1 << 23));

    /* done? */
    assert_equal(bar0readl(0x504), 1 << 25);

    assert_equal(readl(ram + 0x10), 0x01020304);
    assert_equal(readl(ram + 0x14), 0x05060708);
    assert_equal(readl(ram + 0x18), 0x090a0b0c);
    assert_equal(readl(ram + 0x1c), 0x0e0f0102);

    assert_equal(bar0readl(0x450), 0);

    printf("^^^ %s ^^^\n", __func__);
}

static
void test_dma_direct_ram2vme(void)
{
    printf("VVV %s VVV\n", __func__);
    test_init();

    assert_equal(bar0readl(0x450), 0);

    writel(ram + 0x00, 0x01020304);
    writel(ram + 0x04, 0x05060708);
    writel(ram + 0x08, 0x090a0b0c);
    writel(ram + 0x0c, 0x0e0f0102);
    writel(vme16 + 0x1100, 0x00000000);
    writel(vme16 + 0x1104, 0x00000000);
    writel(vme16 + 0x1108, 0x00000000);
    writel(vme16 + 0x110c, 0x00000000);

    assert_equal(bar0readl(0x450), 0);

    /* source PCI address */
    bar0writel(0x520, 0);
    bar0writel(0x524, ram + 0x0);
    bar0writel(0x530, 0);
    /* dest VME A16 D32 SCT USR DATA */
    bar0writel(0x528, 0);
    bar0writel(0x52c, 0x1100);
    bar0writel(0x534, (1 << 28) | (1 << 6));
    /* count */
    bar0writel(0x540, 4 * 4);

    /* still done from previous test */
    assert_equal(bar0readl(0x504), 1 << 25);

    /* direct GO */
    bar0writel(0x500, (1 << 25) | (1 << 23));

    /* done? */
    assert_equal(bar0readl(0x504), 1 << 25);

    assert_equal(readl(vme16 + 0x1100), 0x01020304);
    assert_equal(readl(vme16 + 0x1104), 0x05060708);
    assert_equal(readl(vme16 + 0x1108), 0x090a0b0c);
    assert_equal(readl(vme16 + 0x110c), 0x0e0f0102);

    assert_equal(bar0readl(0x450), 0);

    printf("^^^ %s ^^^\n", __func__);
}

int main(int argc, char *argv[])
{
    int ret;
    g_test_init(&argc, &argv, NULL);

    qtest_start("-device tsi148,id=vbridge "
                "-device vme-test,base=0x1000 ");

    /* TODO: intercept named GPIO */

    qtest_add_func("/tsi148/config", test_pci_config);
    qtest_add_func("/tsi148/mmio", test_mmio);
    qtest_add_func("/tsi148/berr", test_berr);
    qtest_add_func("/tsi148/vmeirq", test_vme_irq);
    qtest_add_func("/tsi148/dma_direct_ram2ram", test_dma_direct_ram2ram);
    qtest_add_func("/tsi148/dma_direct_ram2vme", test_dma_direct_ram2vme);

    ret = g_test_run();

    printf("Tests done\n");

    qtest_end();
    printf("Tests end\n");

    return ret;
}
