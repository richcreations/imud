/*
 * imud — IMU daemon
 * Copyright (c) 2026 Richard Simpson
 * SPDX-License-Identifier: MIT
 */

/*
 * host_time.h — the clock primitives a host has to supply.
 *
 * Everything above this line is portable: the thread model is plain pthreads,
 * every mutex, pthread_create, sigwait and pthread_kill in the tree is POSIX
 * that macOS and the BSDs implement unchanged.  Below it are the two POSIX
 * options that a host may not have, and one Linux extension.
 *
 * The implementation is src/host_time_linux.c, src/host_time_posix.c or
 * src/host_time_fallback.c; the Makefile picks one, the same way it picks a
 * bus backend behind include/bus_backend.h and a GPIO backend behind
 * include/imu_gpio.h.
 *
 * The three that exist are a LADDER, not a matrix -- each rung is the one above
 * it minus one capability, which is why there are three files and not four:
 *
 *   linux     clock selection, and CLOCK_TAI via adjtimex(2)
 *   posix     clock selection; no TAI            (FreeBSD, NetBSD, OpenBSD)
 *   fallback  neither                            (macOS, and any host that
 *                                                 has only POSIX.1-2001)
 *
 * THE LIST IS OPEN.  A host that fits none of them adds a rung -- one file, the
 * entry points below, no #ifdef and no edit above this line -- and selects it
 * with `make HOST_TIME_SRC=src/host_time_<host>.c` or
 * `./configure --with-host-time=...`.  Neither the Makefile's probe nor the
 * configure script has to learn the host's name first, which is the point:
 * a port that must patch the build before it can compile anything is a port
 * that does not get attempted.
 *
 * Write a new rung against the contracts below and nothing else.  They are
 * complete on purpose -- what each call returns, what it does on error, and
 * which calls must agree with which -- so no rung has to be read to write
 * another.  test_host_time is the conformance suite for exactly those
 * contracts; point it at a new backend with
 * `make test_host_time HOST_TIME_TEST_SRC=src/host_time_<host>.c` and it will
 * hold that rung to the same terms as the three here.
 *
 * CLOCK SELECTION is _POSIX_CLOCK_SELECTION, and it is one option rather than
 * two unrelated gaps -- which is the whole reason this header exists.  macOS
 * does not implement it, and that single absence breaks the tree in two places
 * that look unconnected: src/ring.c does not LINK (no pthread_condattr_setclock)
 * and src/output.c silently loses its output cadence (no clock_nanosleep, so
 * the deadline sleep becomes a relative one).  Naming the option once, here,
 * is what stops the next port fixing half of it.
 *
 * The calls come in PAIRS, and each pair is why the seam is drawn where it is
 * rather than one call further in.  A backend owns both ends of a pair, so it
 * can answer both with one clock:
 *
 *   host_monotonic_now()        + host_sleep_until()
 *   host_cond_init_monotonic()  + host_cond_deadline() + host_cond_timedwait()
 *
 * Splitting a pair is the bug this shape exists to prevent.  A condition
 * variable's timeout clock is fixed when it is created, so a deadline computed
 * on a DIFFERENT clock is not a small error: CLOCK_MONOTONIC is seconds-since-
 * boot and CLOCK_REALTIME is seconds-since-1970, so mixing them either returns
 * instantly or waits five decades.  No caller is handed the clockid to get
 * wrong.  test_host_time pins the agreement.
 *
 * WHAT A macOS RUNG WOULD PUT HERE.  Recorded rather than written, and still
 * unwritten now that macOS builds and runs: the fallback rung passes this
 * header's whole conformance suite there, so a mach rung would buy accuracy
 * rather than function, and the daemon does not need it to run.  What it would
 * buy is below — the answers are the awkward part of that port and finding
 * them twice is waste:
 *
 *   host_sleep_until      mach_wait_until(), <mach/mach_time.h>.  An absolute
 *                         deadline the kernel compares against, so it needs
 *                         none of src/host_time_fallback.c's recompute loop.
 *   host_monotonic_now    mach_absolute_time(), or clock_gettime(
 *                         CLOCK_UPTIME_RAW) -- NOT CLOCK_MONOTONIC.  This is
 *                         the trap, and it is why reading the clock is in this
 *                         header at all: macOS CLOCK_MONOTONIC counts time
 *                         asleep and mach_wait_until's timebase does not, so a
 *                         backend that mixed them would be correct until the
 *                         lid closed.  Both ends must come from one source.
 *   host_cond_timedwait   pthread_cond_timedwait_relative_np(), converting the
 *                         deadline to a remainder.  Relative, so a wall-clock
 *                         step cannot stretch the wait -- which is the one
 *                         thing the fallback rung gets wrong.
 *
 * Holding an output rate is a scheduling question beyond any of these: macOS
 * wants pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE), or
 * thread_policy_set(THREAD_TIME_CONSTRAINT_POLICY) for a hard one.  Linux
 * wants SCHED_FIFO.  The tree sets neither on either host.
 *
 * NOT everything that reads a clock belongs here.  The ~30 other
 * clock_gettime(CLOCK_MONOTONIC) calls in src/ measure elapsed intervals, and
 * every host answers those the same way.  Only the two pairs above -- where one
 * call's output is another call's input -- have to agree.
 */
#ifndef IMUD_HOST_TIME_H
#define IMUD_HOST_TIME_H

#include <pthread.h>
#include <time.h>

/*
 * The current time on the clock host_sleep_until() sleeps against.  Build every
 * deadline for it from this, never from clock_gettime() -- see the header
 * comment on why the two are one pair.  Cannot fail.
 */
void host_monotonic_now(struct timespec *out);

/*
 * Sleep until `deadline`, an absolute time on the monotonic clock.  Returns 0
 * when the deadline is reached -- including when it had already passed, which
 * is not an error -- or -1 with errno set.
 *
 * Absorbs EINTR: it returns at the deadline and not before, however many
 * signals arrive.  The output threads pace on this, so an early return is a
 * tick emitted early, and a caller that re-armed from `now` instead of from
 * the deadline would then drift.  A backend that cannot sleep absolutely must
 * loop rather than sleep once; see src/host_time_fallback.c.
 */
int host_sleep_until(const struct timespec *deadline);

/*
 * Initialise `c` to time out on the monotonic clock where the host has clock
 * selection, and on whatever libc defaults to where it does not.  Returns 0,
 * or -1 with errno set.
 *
 * Pair every wait on `c` with a deadline from host_cond_deadline(), never with
 * one from clock_gettime() -- see the header comment.
 */
int host_cond_init_monotonic(pthread_cond_t *c);

/*
 * Fill *out with now + `ms`, on the same clock host_cond_init_monotonic() gave
 * the cond.  Cannot fail.
 */
void host_cond_deadline(struct timespec *out, long ms);

/*
 * Wait on `c` until `deadline` from host_cond_deadline(), with `m` held.
 * Returns 0 when signalled, ETIMEDOUT at the deadline, or another errno value
 * -- the pthread convention, returned rather than set, because callers
 * distinguish ETIMEDOUT from failure.
 *
 * Spurious wake-ups happen, so this stays inside the caller's predicate loop.
 * Pass the SAME deadline each time round: rebuilding it per iteration restarts
 * the timeout and the wait never expires.
 *
 * The wait is in the seam so a backend can substitute a relative-timeout call
 * for it; see the macOS notes above.
 */
int host_cond_timedwait(pthread_cond_t *c, pthread_mutex_t *m,
                        const struct timespec *deadline);

/*
 * The TAI-UTC offset in whole seconds -- 37 since 2017.  Returns 0 and writes
 * *secs, or -1 with errno set; ENOSYS means this build has no way to ask,
 * which a caller must report differently from a query that failed.  The two
 * are separate diagnoses: one is the port, the other is the operator's clock
 * daemon, and src/main.c prints a different line for each.
 *
 * A 0 offset is returned as a success.  It means the host was asked and said
 * zero, which is chrony not having set tai_offset -- also src/main.c's call.
 */
int host_tai_offset(int *secs);

/*
 * The current time on TAI.  Returns 0, or -1 with errno set to ENOSYS on a
 * host with no TAI clock -- and in that case *ts is still filled, with
 * CLOCK_REALTIME, so a caller may carry on with a value that is UTC.
 *
 * Check the return before labelling the result TAI.  It is off by the leap
 * seconds otherwise, 37 of them today, and nothing downstream can tell.
 */
int host_clock_tai(struct timespec *ts);

#endif /* IMUD_HOST_TIME_H */
