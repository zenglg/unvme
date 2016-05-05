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
 * @brief UNVMe APC model specific routines.
 */

#include "unvme.h"

/// APC library model
int unvme_model = UNVME_MODEL_APC;


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
            piocpq->count--;
            return page;
        }
        if (++h == piocpq->size) h = 0;
    }
    return NULL;
}

/**
 * Put a page into a thread processing queue.
 * @param   piocpq     thread processing queue
 * @param   page        page reference
 */
static void unvme_cpq_put(unvme_piocpq_t* piocpq, unvme_page_t* page)
{
    piocpq->pa[piocpq->tail] = page;
    if (++piocpq->tail == piocpq->size) piocpq->tail = 0;
    piocpq->count++;
}

/**
 * Check a completion queue.
 * @param   ioq         io queue
 * @return  a completed page or NULL if no completion yet.
 */
static unvme_page_t* unvme_check_cq(unvme_queue_t* ioq)
{
    int stat;
    int cid = nvme_check_completion(ioq->nvq, &stat);
    if (cid < 0) return NULL;
    unvme_piostat_t* piostat = ioq->datapool.piostat;
    piostat[cid].cpa->stat = stat;
    piostat[cid].ustat = UNVME_PS_READY;
    return piostat[cid].cpa;
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
    unvme_queue_t* q = ((unvme_session_t*)(ns->ses))->queues + pa->qid;
    unvme_piocpq_t* piocpq = q->datapool.piocpq;
    int cid = pa->id;

    // check the already processed queue first
    unvme_page_t* p = unvme_cpq_get(piocpq, cid);
    if (p) return p;

    // check to process all completion
    unvme_page_t* ap;
    while ((ap = unvme_check_cq(q))) {
        if (ap->id == cid) p = ap;
        else unvme_cpq_put(piocpq, ap);
    }
    if (p) return p;

    if (sec > 0) {
        u64 timeout = rdtsc() + sec * rdtsc_second();
        do {
            if ((p = unvme_check_cq(q))) {
                if (p->id == cid) return p;
                unvme_cpq_put(piocpq, p);
            }
        } while (rdtsc() < timeout);
        //ERROR("timeout %d seconds", sec);
    }

    return NULL;
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
    unvme_queue_t* q = ((unvme_session_t*)(ns->ses))->queues + qid;
    unvme_piocpq_t* piocpq = q->datapool.piocpq;
    unvme_page_t* p = NULL;

    // check to get a page from the already processed queue first
    if (piocpq->count) {
        p = piocpq->pa[piocpq->head];
        if (++piocpq->head == piocpq->size) piocpq->head = 0;
        piocpq->count--;
        return p;
    }

    // check to process all completion
    unvme_page_t* ap;
    while ((ap = unvme_check_cq(q))) {
        if (!p) p = ap;
        else unvme_cpq_put(piocpq, ap);
    }
    if (p) return p;

    if (sec > 0) {
        u64 timeout = rdtsc() + sec * rdtsc_second();
        do {
            if ((p = unvme_check_cq(q))) return p;
        } while (rdtsc() < timeout);
        //ERROR("timeout %d seconds", sec);
    }

    return NULL;
}

/**
 * Create session extended function for the model.
 * @param   ses         session
 * @return  0 if ok else -1.
 */
void unvme_session_create_ext(unvme_session_t* ses)
{
}

/**
 * Delete session extended function for the model.
 * @param   ses         session
 * @return  0 if ok else -1.
 */
void unvme_session_delete_ext(unvme_session_t* ses)
{
}

