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
 * @brief UNVMe logging support routines.
 */

#include <stdarg.h>
#include <pthread.h>

#include "unvme_log.h"
#include "unvme_shm.h"


// Static global variables
static FILE* log_fp = NULL;                 ///< log file pointer
static pthread_spinlock_t log_lock = 0;     ///< log locking


/**
 * Open log file.
 * @param   name        log filename
 * @param   mode        open mode
 * @return  0 if ok else -1.
 */
int log_open(const char* name, const char* mode)
{
    if (log_fp || !name) return 0;

    if (pthread_spin_init(&log_lock, PTHREAD_PROCESS_PRIVATE)) {
        perror("pthread_spin_init");
        return -1;
    }

    log_fp = fopen(name, mode);
    if (!log_fp) {
        perror("log_open");
        return -1;
    }

    return 0;
}

/**
 * Close the log file.
 */
void log_close()
{
    if (log_fp && log_fp != stdout) {
        fclose(log_fp);
        log_fp = NULL;
    }
    if (log_lock) {
        pthread_spin_destroy(&log_lock);
        log_lock = 0;
    }
}

/**
 * Write a formatted message to log file, if log file is opened.
 * If err flag is set then log also to stderr.
 * @param   err         print to stderr indication
 * @param   fmt         formatted message
 */
void log_msg(int err, const char* fmt, ...)
{
    va_list args;

    if (log_fp) {
        pthread_spin_lock(&log_lock);
        va_start(args, fmt);
        if (err) {
            char s[256];
            vsnprintf(s, sizeof(s), fmt, args);
            fprintf(stderr, "%s", s);
            fprintf(log_fp, "%s", s);
            fflush(log_fp);
        } else {
            vfprintf(log_fp, fmt, args);
            fflush(log_fp);
        }
        va_end(args);
        pthread_spin_unlock(&log_lock);
    } else {
        va_start(args, fmt);
        if (err) {
            vfprintf(stderr, fmt, args);
        } else {
            vfprintf(stdout, fmt, args);
            fflush(stdout);
        }
        va_end(args);
    }
}

