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
 * @brief UNVMe thread processing completion routines.
 */

#include "unvme.h"

extern sem_t unvme_sem;


/**
 * Thread to process completion queues.
 * @param   arg         queue pointer
 * @return  value from thread
 */
void* unvme_tpc_thread(void* arg)
{
    unvme_session_t* ses = arg;
    unvme_device_t* dev = ses->dev;
    unvme_queue_t* ioqs = ses->queues;
    int qcount = ses->qcount;
    int toqid = ioqs[qcount-1].id;

    INFO_FN("%x: start q=%d-%d", dev->vfiodev->pci, ses->id, toqid);
    sem_post(&unvme_sem);

    while (sem_wait(&ses->tpc.sem) == 0) {
        int cid, stat;
        int i = 0;
        for (;;) {
            if (ses->tpc.stop) goto end;
            cid = nvme_check_completion(ioqs[i].nvq, &stat);
            if (cid >= 0) break;
            if (++i == qcount) i = 0;
        }
        unvme_datapool_t* datapool = &ioqs[i].datapool;
        datapool->piostat[cid].cstat = stat;
        datapool->piostat[cid].ustat = UNVME_PS_READY;
        unvme_piocpq_t* cpq = datapool->piocpq;
        cpq->pa[cpq->tail] = datapool->piostat[cid].cpa;
        if (++cpq->tail == cpq->size) cpq->tail = 0;
        atomic_add(&cpq->count, 1);
    }

end:
    INFO_FN("%x: end q=%d-%d", dev->vfiodev->pci, ses->id, toqid);
    return 0;
}

/**
 * Create thread process completion data.
 * @param   ses         session
 */
void unvme_tpc_create(unvme_session_t* ses)
{
    if (sem_init(&ses->tpc.sem, 0, 0)) FATAL("sem_init");
    if (pthread_create(&ses->tpc.thread, 0, unvme_tpc_thread, ses)) {
        FATAL("pthread_create");
    }
    sem_wait(&unvme_sem);
}

/**
 * Delete thread process completion data.
 * @param   ses         session
 */
void unvme_tpc_delete(unvme_session_t* ses)
{
    if (ses->tpc.thread) {
        ses->tpc.stop = 1;
        sem_post(&ses->tpc.sem);
        pthread_join(ses->tpc.thread, 0);
        sem_destroy(&ses->tpc.sem);
    }
}

