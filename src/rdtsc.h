/**
 * Copyright (c) 2015-2016, Micron Technology, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * @brief Read TSC (time stamp counter) functions.
 */

#ifndef _RDTSC_H
#define _RDTSC_H

#include <stdint.h>
#include <unistd.h>


/**
 * Read tsc.
 * @return  tsc value.
 */
static inline uint64_t rdtsc(void)
{
    union {
        uint64_t val;
        struct {
            uint32_t lo;
            uint32_t hi;
        };
    } tsc;
    asm volatile ("rdtsc" : "=a" (tsc.lo), "=d" (tsc.hi));
    return tsc.val;
}

/**
 * Get the elapsed tsc since the specified started tsc.
 * @param   tsc         started tsc
 * @return  number of tsc elapsed.
 */
static inline uint64_t rdtsc_elapse(uint64_t tsc) {
    int64_t et;

    do {
        et = rdtsc() - tsc;
    } while (et <= 0);
    return et;
}

/**
 * Get tsc per second using sleeping for 1/100th of a second.
 */
static inline uint64_t rdtsc_second()
{
    static uint64_t tsc_ps = 0;
    if (!tsc_ps) {
        uint64_t tsc = rdtsc();
        usleep(10000);
        tsc_ps = (rdtsc() - tsc) * 100;
    }
    return tsc_ps;
}

#endif // _RDTSC_H

