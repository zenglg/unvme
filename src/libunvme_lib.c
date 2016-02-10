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

extern client_main_t client;


/**
 * Create admin queue model extended function.
 * @param   adminq      admin queue
 */
void unvme_adminq_create_ext(unvme_queue_t* adminq)
{
}

/**
 * Delete admin queue model extended function.
 * @param   adminq      admin queue
 */
void unvme_adminq_delete_ext(unvme_queue_t* adminq)
{
}

/**
 * Allocate DMA data pool in an IO queue.
 * @param   ioq         io queue
 */
void unvme_datapool_alloc(unvme_queue_t* ioq)
{
    unvme_device_t* dev = ioq->dev;
    unvme_datapool_t* datapool = &ioq->datapool;
    size_t datasize = ioq->ns->maxppq * ioq->ns->pagesize;
    size_t statsize = ioq->ns->maxppq * sizeof(unvme_piostat_t);
    size_t cpqsize = sizeof(unvme_piocpq_t) + ioq->nvq->size * sizeof(unvme_page_t*);

    DEBUG_FN("%d.%d", dev->vfiodev->id, ioq->nvq->id);
    datapool->data = vfio_dma_alloc(dev->vfiodev, datasize);
    if (!datapool->data) FATAL();
    datapool->piostat = zalloc(statsize + cpqsize);
    datapool->piocpq = (void*)(datapool->piostat) + statsize;
    datapool->piocpq->size = ioq->nvq->size;

    // allocate PRP list for large IO transfer
    size_t prplistsize = ioq->ns->maxppq * ioq->ns->pagesize;
    datapool->prplist = vfio_dma_alloc(dev->vfiodev, prplistsize);
    if (!datapool->prplist) FATAL();
}

/**
 * Free a DMA data pool in an IO queue.
 * @param   ioq         io queue
 */
void unvme_datapool_free(unvme_queue_t* ioq)
{
    unvme_datapool_t* datapool = &ioq->datapool;

    DEBUG_FN("%d.%d", ioq->dev->vfiodev->id, ioq->nvq->id);
    if (vfio_dma_free(datapool->prplist)) FATAL();
    if (vfio_dma_free(datapool->data)) FATAL();
    free(datapool->piostat);
}

/**
 * Open a client session.
 * @param   ses         session
 * @param   vfid        vfio device id
 * @param   nsid        namespace id
 * @return  0 if ok else -1.
 */
int client_open(client_session_t* ses, int vfid, int nsid)
{
    unvme_queue_t* ioq = unvme_do_open(NULL, vfid, client.cpid,
                                       nsid, ses->qcount, ses->qsize);
    if (!ioq) return -1;

    memcpy(&ses->ns, ioq->ns, sizeof(unvme_ns_t));
    memcpy(ses->ns.model, ioq->ns->model, sizeof(ses->ns.model));
    ses->ns.ses = ses;

    int i;
    for (i = 0; i < ses->qcount; i++) {
        client_queue_t* q = &ses->queues[i];
        q->id = i;
        q->ses = ses;
        q->ioq = ioq;
        ioq = ioq->next;
    }

    return 0;
}

/**
 * Close a client session.
 * @param   ses         session
 * @return  0 if ok else -1.
 */
int client_close(client_session_t* ses)
{
    unvme_queue_t* ioq = ses->queues->ioq;
    if (unvme_do_close(ioq->dev, client.cpid, ioq->nvq->id)) return -1;

    if (ses->next == ses) {
        client.ses = NULL;
    } else {
        ses->next->prev = ses->prev;
        ses->prev->next = ses->next;
        if (client.ses == ses) client.ses = ses->next;
    }
    return 0;
}

/**
 * Allocate a client page entry.
 * @param   q           client queue
 * @param   pal         client page entry
 * @return  0 if ok else -1.
 */
int client_alloc(client_queue_t* q, client_pal_t* pal)
{
    unvme_page_t* p = pal->pa;
    int i;
    for (i = 0; i < pal->count; i++) {
        p->id = unvme_do_alloc(q->ioq);
        if (p->id < 0) return -1;
        p->qid = q->id;
        p->nlb = q->ses->ns.nbpp;
        p->buf = q->ioq->datapool.data->buf + p->id * q->ses->ns.pagesize;
        p++;
    }
    return 0;
}

/**
 * Free a client page entry.
 * @param   q           client queue
 * @param   pal         client page entry
 * @return  0 if ok else -1.
 */
int client_free(client_queue_t* q, client_pal_t* pal)
{
    int i;
    for (i = 0; i < pal->count; i++) {
        if (unvme_do_free(q->ioq, pal->pa[i].id)) return -1;
    }
    return 0;
}

/**
 * Send a client IO request.
 * @param   ns          namespace handle
 * @param   pa          page array
 * @param   opc         op code
 * @return  0 if ok else error code.
 */
int client_rw(const unvme_ns_t* ns, unvme_page_t* pa, int opc)
{
    unvme_queue_t* ioq = ((client_session_t*)(ns->ses))->queues[pa->qid].ioq;
    int ustat = ioq->datapool.piostat[pa->id].ustat;
    if (ustat != UNVME_PS_READY) {
        ERROR("page %d is busy stat=%d", pa->id, ustat);
        return -1;
    }
    ioq->datapool.piostat[pa->id].cpa = pa;
    return unvme_do_rw(ioq, pa, opc);
}

