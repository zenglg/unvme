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
 * @brief UNVMe APC/TPC library model specific routines.
 */

#include <string.h>

#include "unvme.h"

/**
 * Allocate DMA data pool in an IO queue.
 * @param   ioq         io queue
 */
void unvme_datapool_alloc(unvme_queue_t* ioq)
{
    unvme_device_t* dev = ioq->ses->dev;
    unvme_ns_t* ns = &ioq->ses->ns;
    size_t datasize = ns->maxppq * ns->pagesize;
    size_t statsize = ns->maxppq * sizeof(unvme_piostat_t);
    size_t cpqsize = sizeof(unvme_piocpq_t) + ioq->nvq->size * sizeof(unvme_page_t*);

    DEBUG_FN("%x.%d", dev->vfiodev->pci, ioq->id);
    unvme_datapool_t* datapool = &ioq->datapool;
    datapool->data = vfio_dma_alloc(dev->vfiodev, datasize);
    if (!datapool->data) FATAL();
    datapool->piostat = zalloc(statsize + cpqsize);
    datapool->piocpq = (void*)(datapool->piostat) + statsize;
    datapool->piocpq->size = ioq->nvq->size;

    // also allocate PRP list for large IO transfer
    size_t prplistsize = ns->maxppq * ns->pagesize;
    datapool->prplist = vfio_dma_alloc(dev->vfiodev, prplistsize);
    if (!datapool->prplist) FATAL();
}

/**
 * Free a DMA data pool in an IO queue.
 * @param   ioq         io queue
 */
void unvme_datapool_free(unvme_queue_t* ioq)
{
    DEBUG_FN("%x.%d", ioq->ses->dev->vfiodev->pci, ioq->id);
    if (ioq->datapool.prplist && vfio_dma_free(ioq->datapool.prplist)) FATAL();
    if (ioq->datapool.data && vfio_dma_free(ioq->datapool.data)) FATAL();
    if (ioq->datapool.piostat) free(ioq->datapool.piostat);
}

/**
 * Open a client session.
 * @param   pci         PCI device id
 * @param   nsid        namespace id
 * @param   qcount      queue count
 * @param   qsize       queue size
 * @return  newly created session
 */
unvme_session_t* client_open(int pci, int nsid, int qcount, int qsize)
{
    return unvme_do_open(NULL, pci, getpid(), nsid, qcount, qsize);
}

/**
 * Close a client session.
 * @param   ns          namespace
 * @return  0 if ok else -1.
 */
int client_close(const unvme_ns_t* ns)
{
    unvme_session_t* ses = ns->ses;
    return unvme_do_close(ses->dev, ses->cpid, ses->id);
}

/**
 * Allocate a client page entry.
 * @param   ns          namespace
 * @param   pal         client page entry
 * @return  0 if ok else -1.
 */
int client_alloc(const unvme_ns_t* ns, unvme_pal_t* pal)
{
    unvme_page_t* p = pal->pa;
    unvme_queue_t* ioq = ((unvme_session_t*)(ns->ses))->queues + p->qid;
    int i;
    for (i = 0; i < pal->count; i++) {
        p->id = unvme_do_alloc(ioq);
        if (p->id < 0) return -1;
        p->qid = pal->pa->qid;
        p->nlb = ns->nbpp;
        p->buf = ioq->datapool.data->buf + p->id * ns->pagesize;
        p++;
    }
    return 0;
}

/**
 * Free a client page entry.
 * @param   ns          namespace
 * @param   pal         client page entry
 * @return  0 if ok else -1.
 */
int client_free(const unvme_ns_t* ns, unvme_pal_t* pal)
{
    unvme_queue_t* ioq = ((unvme_session_t*)(ns->ses))->queues + pal->pa->qid;
    int i;
    for (i = 0; i < pal->count; i++) {
        if (unvme_do_free(ioq, pal->pa[i].id)) return -1;
    }
    return 0;
}

/**
 * Send a client IO request.
 * @param   ns          namespace
 * @param   pa          page array
 * @param   opc         op code
 * @return  0 if ok else error code.
 */
int client_rw(const unvme_ns_t* ns, unvme_page_t* pa, int opc)
{
    unvme_queue_t* ioq = ((unvme_session_t*)(ns->ses))->queues + pa->qid;
    ioq->datapool.piostat[pa->id].cpa = pa;
    return unvme_do_rw(ioq, pa, opc);
}

