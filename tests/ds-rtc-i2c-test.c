/* Testing of Dallas/Maxim I2C bus RTC devices
 *
 * Copyright (c) 2017 Michael Davidsaver
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the LICENSE file in the top-level directory.
 */
#include <stdio.h>

#include "qemu/osdep.h"
#include "qemu/bcd.h"
#include "qemu/cutils.h"
#include "qemu/timer.h"
#include "libqtest.h"
#include "libqos/libqos.h"
#include "libqos/i2c.h"

#define IMX25_I2C_0_BASE 0x43F80000
#define DS1338_ADDR 0x68

#define E500_CCSR_BASE 0xff700000
#define DS1375_ADDR 0xd0

static I2CAdapter *i2c;
static uint8_t addr;
static bool use_century;

static
time_t rtc_gettime(void)
{
    struct tm parts;
    uint8_t buf[7];

    buf[0] = 0;
    i2c_send(i2c, addr, buf, 1);
    i2c_recv(i2c, addr, buf, 7);

    parts.tm_sec = from_bcd(buf[0]);
    parts.tm_min = from_bcd(buf[1]);
    if (buf[2] & 0x40) {
        /* 12 hour */
        parts.tm_hour = from_bcd(buf[2] & 0x1f) % 12u;
        if (buf[2] & 0x20) {
            parts.tm_hour += 12u;
        }
    } else {
        /* 24 hour */
        parts.tm_hour = from_bcd(buf[2] & 0x3f);
    }
    parts.tm_wday = from_bcd(buf[3]);
    parts.tm_mday = from_bcd(buf[4]);
    parts.tm_mon =  from_bcd((buf[5] & 0x1f) - 1u);
    parts.tm_year = from_bcd(buf[6]);
    if (!use_century || (buf[5] & 0x80)) {
        parts.tm_year += 100u;
    }

    return mktimegm(&parts);
}

/* read back and compare with current system time */
static
void test_rtc_current(void)
{
    uint8_t buf;
    time_t expected, actual;

    /* magic address to zero RTC time offset
     * as tests may be run in any order
     */
    buf = 0xff;
    i2c_send(i2c, addr, &buf, 1);

    actual = time(NULL);
    /* new second may start here */
    expected = rtc_gettime();
    g_assert_cmpuint(expected, <=, actual + 1);
    g_assert_cmpuint(expected, >=, actual);
}


static uint8_t test_time_24[8] = {
    0, /* address */
    /* Wed, 22 Nov 2017 18:30:53 +0000 */
    0x53,
    0x30,
    0x18, /* 6 PM in 24 hour mode */
    0x03, /* monday is our day 1 */
    0x22,
    0x11 | 0x80,
    0x17,
};

static uint8_t test_time_12[8] = {
    0, /* address */
    /* Wed, 22 Nov 2017 18:30:53 +0000 */
    0x53,
    0x30,
    0x67, /* 6 PM in 12 hour mode */
    0x03, /* monday is our day 1 */
    0x22,
    0x11 | 0x80,
    0x17,
};

/* write in and read back known time */
static
void test_rtc_set(const void *raw)
{
    const uint8_t *testtime = raw;
    uint8_t buf[7];
    unsigned retry = 2;

    for (; retry; retry--) {
        i2c_send(i2c, addr, testtime, 8);
        /* new second may start here */
        i2c_send(i2c, addr, testtime, 1);
        i2c_recv(i2c, addr, buf, 7);

        if (testtime[1] == buf[0]) {
            break;
        }
        /* we raced start of second, retry */
    };

    g_assert_cmpuint(testtime[1], ==, buf[0]);
    g_assert_cmpuint(testtime[2], ==, buf[1]);
    g_assert_cmpuint(testtime[3], ==, buf[2]);
    g_assert_cmpuint(testtime[4], ==, buf[3]);
    g_assert_cmpuint(testtime[5], ==, buf[4]);
    if (use_century) {
        g_assert_cmpuint(testtime[6], ==, buf[5]);
    } else {
        g_assert_cmpuint(testtime[6] & 0x7f, ==, buf[5]);
    }
    g_assert_cmpuint(testtime[7], ==, buf[6]);

    g_assert_cmpuint(retry, >, 0);
}

int main(int argc, char *argv[])
{
    int ret;
    const char *arch = qtest_get_arch();

    g_test_init(&argc, &argv, NULL);

    if (strcmp(arch, "arm") == 0) {
        qtest_start("-display none -machine imx25-pdk");
        i2c = imx_i2c_create(IMX25_I2C_0_BASE);
        addr = DS1338_ADDR;
        use_century = false;

    } else if (strcmp(arch, "ppc") == 0) {
        qtest_start("-machine mvme3100-1152");
        i2c = e500_i2c_create(E500_CCSR_BASE);
        addr = DS1375_ADDR;
        use_century = true;
    }

    qtest_add_data_func("/ds-rtc-i2c/set24", test_time_24, test_rtc_set);
    qtest_add_data_func("/ds-rtc-i2c/set12", test_time_12, test_rtc_set);
    qtest_add_func("/ds-rtc-i2c/current", test_rtc_current);

    ret = g_test_run();

    qtest_end();

    return ret;
}
