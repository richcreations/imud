/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * test_imu_gpio_null.c — the no-libgpiod GPIO backend (src/imu_gpio_null.c).
 *
 * Links that one file and nothing else: it is what imud carries on a host with
 * no libgpiod, so a suite that needed the daemon to link would not be testing
 * the case.  This runs on every platform, gpiod present or not.
 *
 * What is pinned here is the contract src/imu.c depends on, not the stubs
 * themselves.  imu_gpio_open must fail rather than hand back a handle, because
 * a non-NULL line puts both reader threads onto imu_gpio_wait_edge and off the
 * rate-sized timer; and it must fail with ENOSYS specifically, because that is
 * how imud-imutest tells a build with no backend (IMT_GPIO_UNSUPPORTED) from a
 * line it could not get (IMT_GPIO_EIO), and how src/imu.c knows to say "built
 * without libgpiod" instead of "Function not implemented".
 */

#include <errno.h>
#include <stdio.h>
#include <string.h>

#include "imu_gpio.h"

static int g_fail;
static int g_checks;

#define EXPECT(c, msg) do {                                       \
        g_checks++;                                               \
        if (!(c)) { printf("  FAIL: %s\n", (msg)); g_fail++; }     \
    } while (0)

static void test_open_always_fails_with_enosys(void)
{
    printf("test_open_always_fails_with_enosys\n");

    errno = 0;
    imu_gpio_line_t *l = imu_gpio_open("gpiochip0", 17, "imud");
    EXPECT(l == NULL, "open returns no line");
    EXPECT(errno == ENOSYS, "and sets ENOSYS");

    /* Not conditional on the arguments: there is no backend to ask, so a
     * different chip, offset or consumer cannot change the answer. */
    errno = 0;
    l = imu_gpio_open("gpiochip4", 27, "imud-imutest");
    EXPECT(l == NULL, "a different line fails too");
    EXPECT(errno == ENOSYS, "with the same ENOSYS");

    errno = 0;
    l = imu_gpio_open(NULL, 0, NULL);
    EXPECT(l == NULL, "NULL arguments do not crash it");
    EXPECT(errno == ENOSYS, "and still report ENOSYS");
}

static void test_wait_edge_fails(void)
{
    printf("test_wait_edge_fails\n");

    /*
     * Unreachable through imu.c, which only waits on a non-NULL line -- but
     * -1 rather than 0 is what keeps it that way: a 0 would read as "timed
     * out, read anyway" and spin a reader at full tilt on a line that does
     * not exist.
     */
    errno = 0;
    EXPECT(imu_gpio_wait_edge(NULL, 20) == -1, "wait_edge fails");
    EXPECT(errno == ENOSYS, "with ENOSYS");
}

static void test_close_is_safe(void)
{
    printf("test_close_is_safe\n");

    /* imu.c closes both lines unconditionally on the teardown paths, so this
     * is called with NULL on every shutdown of a no-gpiod build. */
    imu_gpio_close(NULL);
    EXPECT(1, "close(NULL) returns");
}

int main(void)
{
    printf("=== test_imu_gpio_null ===\n");

    test_open_always_fails_with_enosys();
    test_wait_edge_fails();
    test_close_is_safe();

    printf("\n%d passed, %d failed\n", g_checks - g_fail, g_fail);
    return g_fail ? 1 : 0;
}
