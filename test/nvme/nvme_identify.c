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
 * @brief Invoke NVMe identify command.
 */

#include "nvme_common.c"

/**
 * Print namespace info.
 */
void print_namespace(void* buf)
{
    nvme_identify_ns_t* ns = buf;

    printf("Identify Namespace\n");
    printf("==================\n");
    printf("nsze    : %lu\n", ns->nsze);
    printf("ncap    : %lu\n", ns->ncap);
    printf("nuse    : %lu\n", ns->nuse);
    printf("nsfeat  : %d\n", ns->nsfeat);
    printf("nlbaf   : %d\n", ns->nlbaf);
    printf("flbas   : %d\n", ns->flbas);
    printf("mc      : %d\n", ns->mc);
    printf("dpc     : %d\n", ns->dpc);
    printf("dps     : %d\n", ns->dps);

    int i;
    for (i = 0; i <= ns->nlbaf; i++) {
        printf("lbaf.%-2d : ms=%-3d lbads=%-3d rp=%d  %s\n",
               i, ns->lbaf[i].ms, ns->lbaf[i].lbads, ns->lbaf[i].rp,
               (ns->flbas & 0xf) == i ? "(formatted)" : "");
    }
}

/**
 * Print controller info.
 */
void print_controller(void* buf)
{
    nvme_identify_ctlr_t* ctlr = buf;

    printf("Identify Controller\n");
    printf("===================\n");
    printf("vid     : %d\n", ctlr->vid);
    printf("ssvid   : %d\n", ctlr->ssvid);
    printf("sn      : %.20s\n", ctlr->sn);
    printf("mn      : %.40s\n", ctlr->mn);
    printf("fr      : %.8s\n", ctlr->fr);
    printf("rab     : %d\n", ctlr->rab);
    printf("ieee    : %02x%02x%02x\n", ctlr->ieee[0], ctlr->ieee[1], ctlr->ieee[2]);
    printf("mic     : %d\n", ctlr->mic);
    printf("mdts    : %d\n", ctlr->mdts);
    printf("oacs    : %d\n", ctlr->oacs);
    printf("acl     : %d\n", ctlr->acl);
    printf("aerl    : %d\n", ctlr->aerl);
    printf("frmw    : %d\n", ctlr->frmw);
    printf("lpa     : %d\n", ctlr->lpa);
    printf("elpe    : %d\n", ctlr->elpe);
    printf("npss    : %d\n", ctlr->npss);
    printf("avscc   : %d\n", ctlr->avscc);
    printf("sqes    : %d\n", ctlr->sqes);
    printf("cqes    : %d\n", ctlr->cqes);
    printf("nn      : %d\n", ctlr->nn);
    printf("oncs    : %d\n", ctlr->oncs);
    printf("fuses   : %d\n", ctlr->fuses);
    printf("fna     : %d\n", ctlr->fna);
    printf("vwc     : %d\n", ctlr->vwc);
    printf("awun    : %d\n", ctlr->awun);
    printf("awupf   : %d\n", ctlr->awupf);
    printf("nvscc   : %d\n", ctlr->nvscc);
}

/**
 * Main program.
 */
int main(int argc, char* argv[])
{
    const char* usage = "Usage: %s vfioname [nsid]\n\
Specify nsid to identify specific namespace\n";

    error_print_progname = no_progname;
    if (argc < 2 || argc > 3) error(1, 0, usage, argv[0]);
    int nsid = 0;
    if (argc > 2) {
        char* s = argv[2];
        nsid = strtol(s, &s, 0);
        if (*s || nsid < 0) error(1, 0, usage, argv[0]);
    }

    nvme_setup(argv[1], 8);
    vfio_dma_t* dma = vfio_dma_alloc(vfiodev, 8192);
    if (!dma) error(1, 0, "vfio_dma_alloc");

    int err = nvme_cmd_identify(nvmedev, nsid, dma->addr, dma->addr + 4096);
    if (err) error(1, 0, "nvme_cmd_identify");

    if (nsid) print_namespace(dma->buf);
    else print_controller(dma->buf);

    nvme_cleanup();
    return 0;
}

