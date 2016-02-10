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
 * @brief UNVMe client library common functions.
 */

#include "unvme.h"

// Global client variables
client_main_t    client;                                    ///< client handle
pthread_mutex_t  client_lock = PTHREAD_MUTEX_INITIALIZER;   ///< lock


/**
 * Open a client session to create io queues.
 * @param   vfioname    vfio device filename
 * @param   nsid        namespace id
 * @param   qcount      number of io queues
 * @param   qsize       io queue size
 * @return  namespace pointer or NULL if error.
 */
const unvme_ns_t* unvme_open(const char* vfioname, int nsid, int qcount, int qsize)
{
    int vfid;
    if (sscanf(vfioname, "/dev/vfio/%d", &vfid) != 1) {
        ERROR("invalid vfio device name %s", vfioname);
        return NULL;
    }
    if (access(vfioname, F_OK)) {
        ERROR("no %s", vfioname);
        return NULL;
    }
    if (qcount < 1 || qsize < 2) {
        ERROR("qcount must be > 0 and qsize must be > 1");
        return NULL;
    }

    // allocating client queues
    client_session_t* ses = zalloc(sizeof(client_session_t));
    ses->queues = zalloc(qcount * sizeof(client_queue_t));
    ses->qcount = qcount;
    ses->qsize = qsize;

    // invoke open
    pthread_mutex_lock(&client_lock);
    if (!client.cpid) client.cpid = getpid();

    if (client_open(ses, vfid, nsid)) {
        pthread_mutex_unlock(&client_lock);
        free(ses->queues);
        free(ses);
        return NULL;
    }

    // add client session to the global list
    if (!client.ses) {
        client.ses = ses;
        ses->prev = ses->next = ses;
    } else {
        ses->prev = client.ses->prev;
        ses->next = client.ses;
        client.ses->prev->next = ses;
        client.ses->prev = ses;
    }

    pthread_mutex_unlock(&client_lock);
    return &ses->ns;
}

/**
 * Close a client session and delete all its io queues.
 * @param   ns          namespace handle
 * @return  0 if ok else error code.
 */
int unvme_close(const unvme_ns_t* ns)
{
    client_session_t* ses = (client_session_t*)ns->ses;

    pthread_mutex_lock(&client_lock);
    // free all pages in session queues
    int i;
    for (i = 0; i < ses->qcount; i++) {
        client_queue_t* q = &ses->queues[i];
        while (q->pal) {
            unvme_free(ns, q->pal->pa);
        }
    }
    client_close(ses);
    pthread_mutex_unlock(&client_lock);

    free(ses->queues);
    free(ses);
    return 0;
}

/**
 * Allocate an array of pages from a given client queue.
 * @param   ns          namespace handle
 * @param   qid         client queue id
 * @param   numpages    number of pages
 * @return  an array of allocated pages or NULL if failure.
 */
unvme_page_t* unvme_alloc(const unvme_ns_t* ns, int qid, int numpages)
{
    client_queue_t* q = &((client_session_t*)(ns->ses))->queues[qid];

    if (numpages == 0) return NULL;
    if ((q->pac + numpages) > ns->maxppq) {
        ERROR("%d will exceed queue limit of %d pages", numpages, ns->maxppq);
        return NULL;
    }

    // allocate the page array
    client_pal_t* pal = zalloc(sizeof(client_pal_t) +
                               numpages * sizeof(unvme_page_t));
    pal->pa = (unvme_page_t*)(pal + 1);
    pal->count = numpages;

    if (client_alloc(q, pal)) {
        free(pal);
        return NULL;
    }

    // add to the page allocation track list
    if (!q->pal) {
        q->pal = pal;
        pal->prev = pal->next = pal;
    } else {
        pal->prev = q->pal->prev;
        pal->next = q->pal;
        q->pal->prev->next = pal;
        q->pal->prev = pal;
    }

    q->pac += pal->count;
    return pal->pa;
}

/**
 * Free a page array that was allocated.
 * @param   ns          namespace handle
 * @param   pa          page array pointer
 * @return  0 if ok else error code.
 */
int unvme_free(const unvme_ns_t* ns, unvme_page_t* pa)
{
    client_queue_t* q = &((client_session_t*)(ns->ses))->queues[pa->qid];
    if (!q->pal) return -1;

    // search in the page allocation list
    client_pal_t* pal = q->pal;
    while ((pal->pa != pa) && (pal->next != q->pal)) pal = pal->next;
    if (pal->pa != pa || client_free(q, pal)) return -1;

    // remove from the page allocation entry
    if (pal->next == pal) {
        q->pal = NULL;
    } else {
        pal->next->prev = pal->prev;
        pal->prev->next = pal->next;
        if (q->pal == pal) q->pal = pal->next;
    }
    q->pac -= pal->count;

    free(pal);
    return 0;
}

/**
 * Read a page and then poll to wait for completion.
 * @param   ns          namespace handle
 * @param   pa          page array
 * @return  0 if ok else error code.
 */
int unvme_read(const unvme_ns_t* ns, unvme_page_t* pa)
{
    if (!client_rw(ns, pa, NVME_CMD_READ) &&
         unvme_poll(ns, pa, UNVME_TIMEOUT)) return 0;
    return -1;
}

/**
 * Write a page and then poll to wait for completion.
 * @param   ns          namespace handle
 * @param   pa          page array
 * @return  0 if ok else error code.
 */
int unvme_write(const unvme_ns_t* ns, unvme_page_t* pa)
{
    if (!client_rw(ns, pa, NVME_CMD_WRITE) &&
         unvme_poll(ns, pa, UNVME_TIMEOUT)) return 0;
    return -1;
}

/**
 * Read a page asynchronously (caller is to poll for completion).
 * @param   ns          namespace handle
 * @param   pa          page array
 * @return  0 if ok else error code.
 */
int unvme_aread(const unvme_ns_t* ns, unvme_page_t* pa)
{
    return client_rw(ns, pa, NVME_CMD_READ);
}

/**
 * Write a page asynchronously (caller is to poll for completion).
 * @param   ns          namespace handle
 * @param   pa          page array
 * @return  0 if ok else error code.
 */
int unvme_awrite(const unvme_ns_t* ns, unvme_page_t* pa)
{
    return client_rw(ns, pa, NVME_CMD_WRITE);
}

