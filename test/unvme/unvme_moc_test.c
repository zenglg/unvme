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
 * @brief UNVMe multi-client simultaneous open and close test.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <error.h>
#include <string.h>

#include "libunvme.h"

// Global variables
static int nsid = 1;            ///< namespace id
static int qcount1 = 8;         ///< first process queue count
static int qcount2 = 12;        ///< second process queue count
static int qsize1 = 8;          ///< first process queue size
static int qsize2 = 12;         ///< second process queue size
static int loop = 10;           ///< loop count

/**
 * Main program.
 */
int main(int argc, char* argv[])
{
    const char* usage =
"Usage: %s [OPTION]... pciname\n\
         -n       nsid (default to 1)\n\
         -q       first process queue count (default 8)\n\
         -Q       second process queue count (default 12)\n\
         -d       first process queue size (default 8)\n\
         -D       second process queue size (default 12)\n\
         -l       loop count (default 10)\n\
         pciname  PCI device name (as BB:DD.F format)\n";

    char* prog = strrchr(argv[0], '/');
    prog = prog ? prog + 1 : argv[0];

    int opt;
    while ((opt = getopt(argc, argv, "n:q:Q:d:D:l:")) != -1) {
        switch (opt) {
        case 'l':
            loop = atoi(optarg);
            if (loop < 1) error(1, 0, "loop must be > 0");
            break;
        case 'n':
            nsid = atoi(optarg);
            break;
        case 'q':
            qcount1 = atoi(optarg);
            if (qcount1 < 1) error(1, 0, "qcount must be > 0");
            break;
        case 'Q':
            qcount2 = atoi(optarg);
            if (qcount2 < 1) error(1, 0, "qcount must be > 0");
            break;
        case 'd':
            qsize1 = atoi(optarg);
            if (qsize1 < 2) error(1, 0, "qsize must be > 1");
            break;
        case 'D':
            qsize2 = atoi(optarg);
            if (qsize2 < 2) error(1, 0, "qsize must be > 1");
            break;
        default:
            error(1, 0, usage, prog);
        }
    }
    if (optind >= argc) error(1, 0, usage, prog);
    char* pciname = argv[optind];

    const unvme_ns_t* ns = unvme_open(pciname, nsid, qcount1, qsize1);
    if (!ns) error(1, 0, "unvme_open failed");
    if (strcmp(ns->model, "CS")) {
        printf("This test is only for model CS\n");
        unvme_close(ns);
        return 0;
    }
    unvme_close(ns);

    int qcount, qsize;
    pid_t pid = fork();
    if (pid < 0) {
        perror("fork");
        exit(1);
    } else if (pid > 0) {
        qcount = qcount1;
        qsize = qcount1;
    } else {
        qcount = qcount2;
        qsize = qcount2;
    }
    pid = getpid();
    printf("MULTI-CLIENT.%d OPEN/CLOSE TEST BEGIN\n", pid);

    int i;
    for (i = 0; i < loop; i++) {
        ns = unvme_open(pciname, nsid, qcount, qsize);
        if (!ns) error(1, 0, "%d: unvme_open #%d failed", pid, i);
        printf("%d: open/close q=%d-%d\n", pid, ns->sid, ns->sid + qcount - 1);
        unvme_close(ns);
    }
    printf("MULTI-CLIENT.%d OPEN/CLOSE TEST COMPLETE\n", pid);
    sleep(1);

    return 0;
}

