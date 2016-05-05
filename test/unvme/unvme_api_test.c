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
 * @brief UNVMe all API test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <error.h>
#include <string.h>
#include <sys/time.h>

#include "libunvme.h"

// Global variables
static int nsid = 1;        ///< namespace id
static int qcount = 4;      ///< number of queues to create
static int qsize = 257;     ///< size of each queue
static int loop = 1;        ///< test loop count
static int verbose = 0;     ///< verbose flag useful for debugging

/// macro to print a message if verbose flag is set
#define VPRINT(fmt, arg...) do { if (verbose) printf("@" fmt "\n", ##arg); } while (0)

/// macro to print an error message
#define ERROR(fmt, arg...)  error(1, 0, "ERROR: %s " fmt, __func__, ##arg)


/**
 * Open wrapper function.
 */
static const unvme_ns_t* do_open(const char* pciname, int nsid, int qc, int qs)
{
    errno = 0;
    VPRINT("open %s nsid=%d qc=%d qs=%d", pciname, nsid, qc, qs);
    const unvme_ns_t* ns = unvme_open(pciname, nsid, qc, qs);
    if (!ns) ERROR("unvme_open");
    return ns;
}

/**
 * Close wrapper function.
 */
static void do_close(const unvme_ns_t* ns)
{
    VPRINT("close %p", ns);
    if (unvme_close(ns)) ERROR("unvme_close");
}

/**
 * Page allocation wrapper function.
 */
static unvme_page_t* do_alloc(const unvme_ns_t* ns, int q, int n)
{
    unvme_page_t* pa = unvme_alloc(ns, q, n);
    if (!pa) ERROR("alloc");
    VPRINT("alloc q=%d p=%d n=%d", pa->qid, pa->id, n);
    return pa;
}

/**
 * Page free wrapper function.
 */
static void do_free(const unvme_ns_t* ns, unvme_page_t* pa)
{
    VPRINT("free q=%d p=%d", pa->qid, pa->id);
    if (unvme_free(ns, pa)) ERROR("free");
}

/**
 * Synchronous read wrapper function.
 */
static void do_read(const unvme_ns_t* ns, unvme_page_t* pa)
{
    VPRINT("read q=%d p=%d lba=%#lx", pa->qid, pa->id, pa->slba);
    if (unvme_read(ns, pa) || pa->stat) {
        ERROR("read q=%d p=%d lba=%#lx stat=%#x", pa->qid, pa->id, pa->slba, pa->stat);
    }
}

/**
 * Synchronous write wrapper function.
 */
static void do_write(const unvme_ns_t* ns, unvme_page_t* pa)
{
    VPRINT("write q=%d p=%d lba=%#lx", pa->qid, pa->id, pa->slba);
    if (unvme_write(ns, pa) || pa->stat) {
        ERROR("write q=%d p=%d lba=%#lx stat=%#x", pa->qid, pa->id, pa->slba, pa->stat);
    }
}

/**
 * Asynchronous read wrapper function.
 */
static void do_aread(const unvme_ns_t* ns, unvme_page_t* pa)
{
    VPRINT("aread q=%d p=%d lba=%#lx", pa->qid, pa->id, pa->slba);
    if (unvme_aread(ns, pa)) {
        ERROR("aread q=%d p=%d lba=%#lx", pa->qid, pa->id, pa->slba);
    }
}

/**
 * Asynchronous write wrapper function.
 */
static void do_awrite(const unvme_ns_t* ns, unvme_page_t* pa)
{
    VPRINT("awrite q=%d p=%d lba=%#lx", pa->qid, pa->id, pa->slba);
    if (unvme_awrite(ns, pa)) {
        ERROR("awrite q=%d p=%d lba=%#lx", pa->qid, pa->id, pa->slba);
    }
}

/**
 * Poll wrapper function.
 */
static unvme_page_t* do_poll(const unvme_ns_t* ns, unvme_page_t* pa)
{
    unvme_page_t* p = unvme_poll(ns, pa, UNVME_TIMEOUT);
    if (!p) ERROR("poll q=%d p=%d", pa->qid, pa->id);
    if (p->stat) ERROR("poll q=%d p=%d stat=%#x", p->qid, p->id, p->stat);
    VPRINT("poll %d ok", p->id);
    return p;
}

/**
 * Anonymous poll wrapper function.
 */
static unvme_page_t* do_apoll(const unvme_ns_t* ns, int q)
{
    unvme_page_t* p = unvme_apoll(ns, q, UNVME_TIMEOUT);
    if (!p) ERROR("apoll q=%d", q);
    if (p->stat) ERROR("apoll q=%d p=%d stat=%d", q, p->id, p->stat);
    VPRINT("apoll %d ok", p->id);
    return p;
}

/**
 * Verify page containing data pattern.
 */
static void do_verify(unvme_page_t* pa, u8 pat, int len)
{
    int i;
    u8* buf = pa->buf;

    for (i = 0; i < len; i++, buf++) {
        if (*buf != pat) {
            ERROR("data mismatch q=%d p=%d lba=%#lx pat=%#04x",
                  pa->qid, pa->id, pa->slba, pat);
        }
    }
}

/**
 * Print a test elapsed time.
 */
static void elapsed_time(struct timeval* t0)
{
    struct timeval t1;

    gettimeofday(&t1, 0);
    if (t1.tv_usec < t0->tv_usec) {
        t1.tv_sec--;
        t1.tv_usec += 1000000;
    }
    printf("    Elapsed time %lu msecs\n",
            (t1.tv_sec-t0->tv_sec) * 1000 + (t1.tv_usec-t0->tv_usec) / 1000);
}

/**
 * Main program.
 */
int main(int argc, char* argv[])
{
    const char* usage =
"Usage: %s [OPTION]... pciname\n\
         -v       verbose\n\
         -p       pause between tests\n\
         -n       nsid (default 1)\n\
         -q       number of IO queues to create (default 4)\n\
         -d       IO queue size (default 256)\n\
         -l       test loop count (default 1)\n\
         pciname  PCI device name (as BB:DD.F format)\n";

    const char* prog = strrchr(argv[0], '/');
    prog = prog ? prog + 1 : argv[0];

    struct timeval t0;
    char cr[64];
    unvme_page_t* pa;
    u8 pat;
    int opt, q, d, i, l;
    int pause = 0;

    while ((opt = getopt(argc, argv, "vpn:q:d:l:")) != -1) {
        switch (opt) {
        case 'v':
            verbose = 1;
            break;
        case 'p':
            pause = 1;
            break;
        case 'n':
            nsid = atoi(optarg);
            break;
        case 'q':
            qcount = atoi(optarg);
            break;
        case 'd':
            qsize = atoi(optarg);
            if (qsize < 2) error(1, 0, "qsize must be > 1");
            break;
        case 'l':
            loop = atoi(optarg);
            break;
        default:
            error(1, 0, usage, prog);
        }
    }
    if (optind >= argc) error(1, 0, usage, prog);
    char* pciname = argv[optind];

    printf("API TEST BEGIN\n");
    const unvme_ns_t* ns = do_open(pciname, nsid, qcount, qsize);
    printf("# Open: %s qc=%d qd=%d ps=%d bs=%d nb=%lu model=%s\n", pciname,
         qcount, qsize, ns->pagesize, ns->blocksize, ns->blockcount, ns->model);

    int lbaskip = 2 * ns->nbpp;
    u64 lba = (rand() % ns->blockcount) & ~(lbaskip - 1);

    unvme_page_t** qpa = malloc(qcount * sizeof(unvme_page_t*));
    int* amap = malloc(ns->maxppq * sizeof(int));

    printf("# Testing allocation and free %dx%d pages...", qcount, ns->maxppq);
    if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
    for (i = 0; i < 2; i++) {
        for (q = 0; q < qcount; q++) qpa[q] = do_alloc(ns, q, ns->maxppq);
        for (q = 0; q < qcount; q++) do_free(ns, qpa[q]);
    }

    for (l = 1; l <= loop; l++) {
        // allocate pages for each queue
        for (q = 0; q < qcount; q++) {
            qpa[q] = do_alloc(ns, q, ns->maxiopq);
        }

        printf("# Testing sync-read (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                pa->slba = lba;
                do_read(ns, pa);
            }
        }
        elapsed_time(&t0);

        printf("# Testing sync-write (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                pa->slba = lba;
                do_write(ns, pa);
            }
        }
        elapsed_time(&t0);

        printf("# Testing async-read (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);
            // do async read then poll for completion
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                pa->slba = lba;
                do_aread(ns, pa);
            }
            memset(amap, 0, ns->maxppq * sizeof(int));
            for (d = 0; d < ns->maxiopq; d++) {
                pa = do_apoll(ns, q);
                if (amap[pa->id]) ERROR("apoll page %d twice", pa->id);
                amap[pa->id] = 1;
            }
        }
        elapsed_time(&t0);

        printf("# Testing async-write (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);
            // do async write then poll for completion
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                pa->slba = lba;
                do_awrite(ns, pa);
            }
            memset(amap, 0, ns->maxppq * sizeof(int));
            for (d = 0; d < ns->maxiopq; d++) {
                pa = do_apoll(ns, q);
                if (amap[pa->id]) ERROR("apoll page %d twice", pa->id);
                amap[pa->id] = 1;
            }
        }
        elapsed_time(&t0);

        printf("# Testing sync-write sync-read verify (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                pat = (((q + 1) << 4) & 0xf0) | (d & 0xf);
                VPRINT("test pattern %#04x", pat);
                memset(pa->buf, pat, ns->pagesize);
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                pa->slba = lba;
                do_write(ns, pa);
                memset(pa->buf, 0, ns->pagesize);
                do_read(ns, pa);
                do_verify(pa, pat, ns->pagesize);
            }
        }
        elapsed_time(&t0);

        printf("# Testing async-write sync-read verify (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);

            // do async write
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                pat = (((q + 1) << 4) & 0xf0) | (d & 0xf);
                VPRINT("write pattern %#04x", pat);
                memset(pa->buf, pat, ns->pagesize);
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                pa->slba = lba;
                do_awrite(ns, pa);
            }

            // poll on write completion
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                do_poll(ns, pa);
            }

            // read and compare
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                pat = (((q + 1) << 4) & 0xf0) | (d & 0xf);
                VPRINT("check pattern %#04x", pat);
                memset(pa->buf, 0, ns->pagesize);
                do_read(ns, pa);
                do_verify(pa, pat, ns->pagesize);
            }
        }
        elapsed_time(&t0);

        printf("# Testing sync-write async-read verify (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);
            // do sync write
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                pat = (((q + 1) << 4) & 0xf0) | (d & 0xf);
                VPRINT("write pattern %#04x", pat);
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                memset(pa->buf, pat, ns->pagesize);
                pa->slba = lba;
                do_write(ns, pa);
            }

            // do async read
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                memset(pa->buf, 0, ns->pagesize);
                do_aread(ns, pa);
            }

            // poll on read completion and compare
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                pat = (((q + 1) << 4) & 0xf0) | (d & 0xf);
                VPRINT("check pattern %#04x", pat);
                do_poll(ns, pa);
                do_verify(pa, pat, ns->pagesize);
            }
        }
        elapsed_time(&t0);

        printf("# Testing async-write async-read verify (loop %d)...", l);
        if (pause && fgets(cr, sizeof (cr), stdin)); else printf("\n");
        gettimeofday(&t0, 0);
        for (q = 0; q < qcount; q++) {
            printf("    Testing q=%d\n", q);

            // do async write
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                pat = (((q + 1) << 4) & 0xf0) | (d & 0xf);
                VPRINT("write pattern %#04x", pat);
                memset(pa->buf, pat, ns->pagesize);
                if ((lba += lbaskip) >= ns->blockcount) lba = 0;
                pa->slba = lba;
                do_awrite(ns, pa);
            }

            // poll for write completion then do async read
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                do_poll(ns, pa);
                memset(pa->buf, 0, ns->pagesize);
                do_aread(ns, pa);
            }

            // poll for read completion and compare
            for (d = 0; d < ns->maxiopq; d++) {
                pa = &qpa[q][d];
                pat = (((q + 1) << 4) & 0xf0) | (d & 0xf);
                VPRINT("check pattern %#04x", pat);
                do_poll(ns, pa);
                do_verify(pa, pat, ns->pagesize);
            }
        }
        elapsed_time(&t0);

        // free all pages in each queue
        for (q = 0; q < qcount; q++) {
            do_free(ns, qpa[q]);
        }
    }

    free(qpa);
    do_close(ns);

    printf("API TEST COMPLETE\n");
    return 0;
}

