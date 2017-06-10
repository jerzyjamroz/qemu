#include <stdio.h>

#include "qemu/osdep.h"
#include "libqtest.h"
#include "libqos/libqos.h"
#include "libqos/i2c.h"

#define assert_equal(A, B) g_assert_cmphex((A), ==, (B))

static
I2CAdapter *i2c;

static
void test_ccsr(void)
{
    /* CCSRBAR is self referential */
    assert_equal(readl(0xff700000), 0x000ff700);

    /* introspect memory size */
    assert_equal(readl(0xff702080), 0x80000000);
    /* value is (ram_size-1)>>24 */
    assert_equal(readl(0xff702000), 15);
}

static
void test_cpld(void)
{
    /* read/write to test register */
    assert_equal(readl(0xe2000010), 0x00000000);
    assert_equal(readl(0xe2000014), 0xffffffff);

    writel(0xe2000010, 0x12345678);

    assert_equal(readl(0xe2000010), 0x12345678);
    assert_equal(readl(0xe2000014), 0x12345678 ^ 0xffffffff);
}

static
void test_eeprom(void)
{
    char buf[] = "\x00\x00MOTOROLA";

    /* 1. zero address pointer
     * 2. write 8 bytes,
     * 3. re-zero address pointer
     */
    i2c_send(i2c, 0xa8, (uint8_t *)buf, 10);
    i2c_send(i2c, 0xa8, (uint8_t *)buf, 2);

    /* read 8 bytes */
    i2c_recv(i2c, 0xa8, (uint8_t *)buf, 8);
    buf[8] = '\0';

    /* Read header for Motorola VPD info */
    g_assert_cmpstr(buf, ==, "MOTOROLA");
}

int main(int argc, char *argv[])
{
    int ret;
    g_test_init(&argc, &argv, NULL);

    qtest_start("-machine mvme3100-1152");

    i2c = e500_i2c_create(0xff700000);

    qtest_add_func("/mvme3100/ccsr", test_ccsr);
    qtest_add_func("/mvme3100/cpld", test_cpld);
    qtest_add_func("/mvme3100/eeprom", test_eeprom);

    ret = g_test_run();

    printf("Tests done\n");

    qtest_end();
    printf("Tests end\n");

    return ret;
}
