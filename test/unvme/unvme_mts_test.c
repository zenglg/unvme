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
 * @brief UNVMe multi threaded/session test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <error.h>
#include <string.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <sys/time.h>

#include "libunvme.h"

/// macro to print an error message
#define ERROR(fmt, arg...)  error(1, 0, "ERROR: " fmt "\n", ##arg)

/// macro to print an IO related error message
#define IOERROR(s,t,p)  ERROR(s " t=%d q=%d pi=%d lba=%#lx nb=%d stat=%#x", \
                        t, (p)->qid, (p)->id, (p)->slba, (p)->nlb, (p)->stat)

// Global variables
static char* vfioname;      ///< vfio device name
static int nsid = 1;        ///< namespace id
static int numses = 7;      ///< number of thread sessions
static int qcount = 4;      ///< number of queues to create
static int qsize = 256;     ///< queue size
static int minbpio = 1;     ///< minimum number of blocks per IO
static int maxbpio = 0;     ///< maximum number of blocks per IO
static int loop = 1;        ///< test loop count
static sem_t sm_open;       ///< semaphore to wait for all open completion
static sem_t sm_test;       ///< semaphore to start IO testing


/**
 * Test run session.
 */
void* test_session(void* arg)
{
    u32 *buf, pat;
    struct timeval t0, t1;
    int q, d, i, l, n, cpcount;
    int apoll = (long)arg >> 32;
    int sesid = (long)arg;
    int tid = sesid + 1;

    printf("Thread #%d started\n", tid);
    const unvme_ns_t* ns = unvme_open(vfioname, nsid, qcount, qsize);
    if (!ns) ERROR("unvme_open t=%d qc=%d qs=%d failed", tid, qcount, qsize);

    // max number of IO unique address spaces available for a thread session
    u64 maxlbas = ns->blockcount / numses;
    if ((maxlbas / maxbpio) <= qsize) ERROR("not enough disk space for test");
    u64 slba = sesid * maxlbas;
    u64 elba = slba + maxlbas - 1;
    printf("[%d] lba=%#lx-%#lx\n", tid, slba, elba);
    u8* cpstats = malloc(ns->maxiopq);

    sem_post(&sm_open);
    sem_wait(&sm_test);

    for (l = 1; l <= loop; l++) {
        u64 lba = (elba - maxbpio) & ~(ns->blocksize - 1);
        if (!(l & 0xf)) {
            printf("[%02d.%d] lba=%#lx          \r", tid, l, lba);
            fflush(stdout);
        }

        // write-read-verify test
        for (q = 0; q < qcount; q++) {

            // allocate all pages in a queue and fill each page with a pattern
            unvme_page_t* pages = unvme_alloc(ns, q, ns->maxppq);
            if (!pages) ERROR("unvme_alloc.%d q=%d n=%d", tid, q, ns->maxppq);
            unvme_page_t* p = pages;
            for (d = 0; d < ns->maxppq; d++) {
                buf = p->buf;
                pat = ((p->qid + 1) << 16) | p->id;
                for (i = 0; i < ns->pagesize; i += sizeof(pat)) {
                    *buf++ = pat;
                }
                p++;
            }

            // do async write of various size
            for (d = 0; d < ns->maxiopq; d++) {
                p = pages + d * ns->maxppio;
                n = rand() % (maxbpio + 1);
                p->nlb = n < minbpio ? minbpio : n;
                p->slba = lba;
                if (unvme_awrite(ns, p)) IOERROR("awrite", tid, p);
                lba -= maxbpio;
            }

            // poll for write completion then clear and read back
            for (d = 0; d < ns->maxiopq; d++) {
                if (apoll) {
                    if (!(p = unvme_apoll(ns, q, UNVME_TIMEOUT))) {
                        ERROR("t=%d page write completion timeout", tid);
                    }
                } else {
                    p = pages + d * ns->maxppio;
                    if (!unvme_poll(ns, p, UNVME_TIMEOUT) || p->stat) {
                        IOERROR("poll write", tid, p);
                    }
                }
                for (i = 0; i < ns->maxppio; i++) {
                    memset(p[i].buf, 0, ns->pagesize);
                }
                if (unvme_aread(ns, p)) IOERROR("aread", tid, p);
            }

            // poll on read completion and verify data
            gettimeofday(&t0, 0);
            memset(cpstats, 0, ns->maxiopq);
            for (d = 0, cpcount = 0; ; ) {
                if (cpstats[d]) goto next;

                gettimeofday(&t1, 0);
                if (apoll) {
                    if (!(p = unvme_apoll(ns, q, UNVME_TIMEOUT))) {
                        ERROR("t=%d page read completion timeout", tid);
                    }
                } else {
                    p = pages + d * ns->maxppio;
                    if (!unvme_poll(ns, p, 0)) {
                        if ((t1.tv_sec - t0.tv_sec) < UNVME_TIMEOUT) goto next;
                        IOERROR("poll read timeout", tid, p);
                    }
                    if (p->stat) IOERROR("read", tid, p);
                    cpstats[d] = 1;
                }

                int nlb = p->nlb;
                for (n = 0; n < nlb; n += ns->nbpp) {
                    buf = p->buf;
                    pat = ((p->qid + 1) << 16) | p->id;
                    int bc = (nlb - n) * ns->blocksize;
                    if (bc > ns->pagesize) bc = ns->pagesize;
                    for (i = 0; i < bc; i += sizeof(pat)) {
                        if (*buf != pat) {
                            ERROR("data mismatch t=%d l=%d q=%d pi=%d lba=%#lx buf[%d]=0x%08x",
                                  tid, l, p->qid, p->id, p->slba, i, *buf);
                        }
                        buf++;
                    }
                    p++;
                }

                if (++cpcount == ns->maxiopq) break;
next:
                if (++d == ns->maxiopq) d = 0;
            }

            // free all pages in a queue
            if (unvme_free(ns, pages)) ERROR("unvme_free %d", tid);
        }
    }

    free(cpstats);
    unvme_close(ns);
    printf("Test thread #%d completed\n", tid);
    return 0;
}

/**
 * Main program.
 */
int main(int argc, char* argv[])
{
    const char* usage =
"Usage: %s [OPTION]... vfioname\n\
         -n       nsid (default to 1)\n\
         -t       number of thread sessions (default 7)\n\
         -q       number of IO queues per thread (default 4)\n\
         -d       each IO queue size (default 256)\n\
         -m       minimum IO blockcount (default 1)\n\
         -x       maximum IO blockcount (depends on block size)\n\
         -l       number of test loop iterations\n\
         vfioname vfio device pathname\n";

    char* prog = strrchr(argv[0], '/');
    prog = prog ? prog + 1 : argv[0];

    int opt, i;
    while ((opt = getopt(argc, argv, "n:t:q:d:m:x:l:")) != -1) {
        switch (opt) {
        case 'n':
            nsid = atoi(optarg);
            break;
        case 't':
            numses = atoi(optarg);
            break;
        case 'q':
            qcount = atoi(optarg);
            if (qcount < 1) error(1, 0, "qcount must be > 0");
            break;
        case 'd':
            qsize = atoi(optarg);
            if (qsize < 2) {
                error(1, 0, "qsize must be > 1");
            }
            break;
        case 'm':
            minbpio = atoi(optarg);
            break;
        case 'x':
            maxbpio = atoi(optarg);
            break;
        case 'l':
            loop = atoi(optarg);
            break;
        default:
            error(1, 0, usage, prog);
        }
    }
    if (optind >= argc) error(1, 0, usage, prog);
    vfioname = argv[optind];

    printf("MULTI-SESSION TEST BEGIN\n");
    printf("threadcount=%d nsid=%d qcount=%d qsize=%d\n",
           numses, nsid, qcount, qsize);

    // open device to validate maxbpio
    const unvme_ns_t* ns = unvme_open(vfioname, nsid, qcount, qsize);
    if (!ns) error(1, 0, "unvme_open failed");
    if (!maxbpio) maxbpio = ns->maxbpio;
    if (((minbpio <= 0)) || (minbpio > maxbpio) || (maxbpio > ns->maxbpio)) {
        error(1, 0, "invalid IO block range (min=%d max=%d)", minbpio, maxbpio);
    }
    printf("pagesize=%d blocksize=%d blockcount=%ld maxppq=%d model=%s\n",
           ns->pagesize, ns->blocksize, ns->blockcount, ns->maxppq, ns->model);
    unvme_close(ns);

    sem_init(&sm_open, 0, 0);
    sem_init(&sm_test, 0, 0);
    pthread_t* ses = calloc(numses, sizeof(pthread_t));

    // poll test
    printf("=== Test poll\n");
    for (i = 0; i < numses; i++) {
        long arg = i;
        pthread_create(ses + i, 0, test_session, (void*)arg);
        sem_wait(&sm_open);
    }
    for (i = 0; i < numses; i++) sem_post(&sm_test);
    for (i = 0; i < numses; i++) pthread_join(ses[i], 0);

    // anonymous poll test
    printf("=== Test apoll\n");
    for (i = 0; i < numses; i++) {
        long arg = 0x100000000 + i;
        pthread_create(ses + i, 0, test_session, (void*)arg);
        sem_wait(&sm_open);
    }
    for (i = 0; i < numses; i++) sem_post(&sm_test);
    for (i = 0; i < numses; i++) pthread_join(ses[i], 0);
    free(ses);

    printf("MULTI-SESSION TEST COMPLETE\n");
    return 0;
}

