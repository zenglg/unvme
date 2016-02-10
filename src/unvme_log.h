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
 * @brief UNVMe logging header file.
 */

#ifndef _UNVME_LOG_H 
#define _UNVME_LOG_H 

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/// @cond

#define INFO(fmt, arg...)     log_msg(0, fmt "\n", ##arg)
#define INFO_FN(fmt, arg...)  log_msg(0, "%s " fmt "\n", __func__, ##arg)
#define ERROR(fmt, arg...)    log_msg(1, "ERROR: %s " fmt "\n", __func__, ##arg)

#ifdef UNVME_DEBUG
    #define DEBUG             INFO
    #define DEBUG_FN          INFO_FN
    #define HEX_DUMP          hex_dump
#else
    #define DEBUG(arg...)
    #define DEBUG_FN(arg...)
    #define HEX_DUMP(arg...)
#endif

/// @endcond


// Export function
int log_open(const char* filename, const char* mode);
void log_close();
void log_msg(int err, const char* fmt, ...);


/**
 * Hex dump a data block byte-wise.
 * @param   buf     buffer to read into
 * @param   len     size
 */
static inline void hex_dump(void* buf, int len)
{
    unsigned char* b = buf;
    int i, k = 0, e = 44, t = 44;
    char ss[3906];

    if (len > 1024) len = 1024;

    for (i = 0; i < len; i++) {
        if (!(i & 15)) {
            if (i > 0) {
                ss[k] = ' ';
                ss[t++] = '\n';
                k = t;
            }
            e = t = k + 44;
            k += sprintf(ss+k, "  %04x:", i);
        }
        if (!(i & 3)) ss[k++] = ' ';
        k += sprintf(ss+k, "%02x", b[i]);
        ss[t++] = isprint(b[i]) ? b[i] : '.';
    }
    ss[t] = 0;
    for (i = k; i < e; i++) ss[i] = ' ';
    INFO("%s", ss);
}

/**
 * Invoke calloc and terminated on failure.
 */
static inline void* zalloc(int size)
{
    void* mem = calloc(1, size);
    if (!mem) {
        ERROR("calloc");
        exit(1);
    }
    return mem;
}

#endif // _UNVME_LOG_H

