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
 * @brief UNVMe thread processing completion client polling routines.
 */

#include "unvme.h"

/**
 * Find and remove a page from the page IO completion queue.
 * @param   piocpq      page IO completion queue
 * @param   id          page id
 * @return  page pointer or NULL if not found.
 */
static unvme_page_t* unvme_cpq_get(unvme_piocpq_t* piocpq, int id)
{
    volatile int h = piocpq->head;

    while (h != piocpq->tail) {
        if (piocpq->pa[h]->id == id) {
            unvme_page_t* page = piocpq->pa[h];
            if (h != piocpq->head) piocpq->pa[h] = piocpq->pa[piocpq->head];
            if (++piocpq->head == piocpq->size) piocpq->head = 0;
            atomic_sub(&piocpq->count, 1);
            return page;
        }
        if (++h == piocpq->size) h = 0;
    }
    return NULL;
}

/**
 * Poll and wait for a specific page io completion in a queue.
 * @param   ns          namespace handle
 * @param   pa          page array
 * @param   sec         number of seconds to wait before timeout
 * @return  pointer to the page array completed or NULL if timeout.
 */
unvme_page_t* unvme_poll(const unvme_ns_t* ns, unvme_page_t* pa, int sec)
{
    unvme_queue_t* ioq = ((unvme_session_t*)(ns->ses))->queues + pa->qid;
    int cid = pa->id;
    volatile unvme_piostat_t* piostat = ioq->datapool.piostat + cid;

    if (sec == 0) {
        if (piostat->ustat != UNVME_PS_READY) return NULL;
    } else {
        u64 timeout = 0;
        while (piostat->ustat != UNVME_PS_READY) {
            if (timeout == 0) {
                timeout = rdtsc() + sec * rdtsc_second();
            } else if (rdtsc() > timeout) {
                //ERROR("timeout %d cid=%#x", sec, cid);
                return NULL;
            }
        }
    }

    unvme_piocpq_t* piocpq = ioq->datapool.piocpq;
    unvme_page_t* p;
    while (!(p = unvme_cpq_get(piocpq, cid))) sched_yield();
    if (p != pa) ERROR("page cid=%#x address mismatch (%p != %p)", cid, p, pa);

    p->stat = piostat->cstat;
    return p;
}

/**
 * Poll and wait for any page io completion in the specified queue.
 * @param   ns          namespace handle
 * @param   qid         client queue id
 * @param   sec         number of seconds to poll before timeout
 * @return  pointer to the page array completed or NULL if timeout.
 */
unvme_page_t* unvme_apoll(const unvme_ns_t* ns, int qid, int sec)
{
    unvme_queue_t* ioq = ((unvme_session_t*)(ns->ses))->queues + qid;
    volatile unvme_piocpq_t* piocpq = ioq->datapool.piocpq;

    if (sec == 0) {
        if (piocpq->count == 0) return NULL;
    } else {
        u64 timeout = 0;
        while (piocpq->count == 0) {
            if (timeout == 0) {
                timeout = rdtsc() + sec * rdtsc_second();
            } else if (rdtsc() > timeout) {
                //ERROR("timeout %d seconds", sec);
                return NULL;
            }
        }
    }

    // retrieve the first completion queue entry
    unvme_page_t* pa = piocpq->pa[piocpq->head];
    pa->stat = ioq->datapool.piostat[pa->id].cstat;
    if (++piocpq->head == piocpq->size) piocpq->head = 0;
    atomic_sub(&piocpq->count, 1);

    return pa;
}

