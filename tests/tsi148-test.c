#include <stdio.h>

#include "libqtest.h"
#include "libqos/libqos.h"
#include "libqos/pci.h"
#include "libqos/pci-pc.h"

#define assert_equal(A, B) g_assert_cmphex((A), ==, (B))

static QPCIBus *pcibus;
static QPCIDevice *pcidev;
static uint64_t bar0, vme16;

static void save_fn(QPCIDevice *dev, int devfn, void *data)
{
    QPCIDevice **pdev = (QPCIDevice **) data;

    *pdev = dev;
}

static
void test_init(void)
{
    uint64_t bar0size = 0;

    pcibus = qpci_init_pc();
    g_assert(pcibus != NULL);

    qpci_device_foreach(pcibus, 0x10e3, 0x0148, save_fn, &pcidev);

    g_assert(pcidev != NULL);

    bar0 = (uint64_t)qpci_iomap(pcidev, 0, &bar0size);
    assert_equal(bar0size, 0x1000);

    qpci_device_enable(pcidev);

    assert_equal(readl(bar0+0x234), 3);

    writel(bar0+0x604, 1<<27); /* master enable */

    vme16 = bar0+0x10000;

    /* map the A16 USR DATA address space */
    /* PCI Start */
    writel(bar0+0x100, vme16>>32);
    writel(bar0+0x104, vme16);
    /* PCI End */
    writel(bar0+0x108, (vme16+0x10000)>>32);
    writel(bar0+0x10c, vme16+0x10000);
    /* VME Start */
    writel(bar0+0x110, 0);
    writel(bar0+0x114, 0);

    /* Enable map, disable prefetch, single cycle, D16, no-SUP, no-PGM, A16 */
    writel(bar0+0x11c, 0x80040000);

    assert_equal(readl(vme16+0x1000), 0x12345678);
}

int main(int argc, char *argv[])
{
    int ret;
    g_test_init(&argc, &argv, NULL);

    qtest_start("-device tsi148,id=vbridge "
                "-device vme-test,base=0x1000,vector=42,level=3 ");

    irq_intercept_out("vbridge");

    qtest_add_func("/tsi148/init", test_init);

    ret = g_test_run();

    printf("Tests done\n");

    qtest_end();
    printf("Tests end\n");

    return ret;
}
