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
 * @brief UNVMe simple write-read-verify test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <error.h>

#include "libunvme.h"


int main(int argc, char** argv)
{
    const char* usage =
    "Usage: %s [OPTION]... pciname\n\
             -n       nsid (default 1)\n\
             -q       number of IO queues to create (default 4)\n\
             -d       IO queue size (default 32)\n\
             pciname  PCI device name (as BB:DD.F format)\n";

    int opt, nsid = 1, qcount = 8, qsize = 32;
    const char* prog = strrchr(argv[0], '/');
    prog = prog ? prog + 1 : argv[0];

    while ((opt = getopt(argc, argv, "n:q:d:")) != -1) {
        switch (opt) {
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
        default:
            error(1, 0, usage, prog);
        }
    }
    if (optind >= argc) error(1, 0, usage, prog);

    char* pciname = argv[optind];
    const unvme_ns_t* ns = unvme_open(pciname, nsid, qcount, qsize);
    if (!ns) error(1, 0, "unvme_open %s failed", argv[0]);

    int len = ns->pagesize / sizeof(u64);
    int q, b;

    printf("SIMPLE WRITE-READ-VERIFY TEST BEGIN\n");
    for (q = 0; q < qcount; q++) {
        printf("Test q=%d\n", q);
        unvme_page_t* p = unvme_alloc(ns, q, 1);
        u64 lba = q * ns->nbpp;
        p->slba = lba;
        u64* buf = p->buf;
        for (b = 0; b < len; b++) buf[b] = lba;
        unvme_write(ns, p);
        memset(buf, 0, ns->pagesize);
        unvme_read(ns, p);
        for (b = 0; b < len; b++) {
            if (buf[b] != lba) error(1, 0, "mismatch at lba %#lx", lba);
        }
        unvme_free(ns, p);
    }

    unvme_close(ns);
    printf("SIMPLE WRITE-READ-VERIFY TEST COMPLETE\n");
    return 0;
}
