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

#include <stddef.h>
#include "unvme.h"


/// Global client info
unvme_client_t  client = {  .lock = PTHREAD_MUTEX_INITIALIZER,
                            .csif = { 0 },
                            .ses = NULL
                         };


/**
 * Open a client session to create io queues.
 * @param   pciname     PCI device name (as BB:DD.F format)
 * @param   nsid        namespace id
 * @param   qcount      number of io queues
 * @param   qsize       io queue size
 * @return  namespace pointer or NULL if error.
 */
const unvme_ns_t* unvme_open(const char* pciname, int nsid, int qcount, int qsize)
{
    int b, d, f;
    if (sscanf(pciname, "%02x:%02x.%1x", &b, &d, &f) != 3) {
        ERROR("invalid PCI device %s (expect BB:DD.F format)", pciname);
        return NULL;
    }
    if (qcount < 1 || qsize < 2) {
        ERROR("qcount must be > 0 and qsize must be > 1");
        return NULL;
    }

    int pci = (b << 16) + (d << 8) + f;

    pthread_mutex_lock(&client.lock);
    unvme_session_t* ses = client_open(pci, nsid, qcount, qsize);
    if (ses && !client.ses) client.ses = ses;
    pthread_mutex_unlock(&client.lock);
    return ses ? &ses->ns : NULL;
}

/**
 * Close a client session and delete its contained io queues.
 * @param   ns          namespace handle
 * @return  0 if ok else error code.
 */
int unvme_close(const unvme_ns_t* ns)
{
    unvme_session_t* ses = (unvme_session_t*)ns->ses;

    pthread_mutex_lock(&client.lock);
    // free all the allocated pages in the session
    int i;
    for (i = 0; i < ses->qcount; i++) {
        while (ses->queues[i].pal) unvme_free(ns, ses->queues[i].pal->pa);
    }
    client_close(ns);
    pthread_mutex_unlock(&client.lock);
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
    unvme_queue_t* ioq = ((unvme_session_t*)(ns->ses))->queues + qid;

    if (numpages == 0) return NULL;
    if ((ioq->pac + numpages) > ns->maxppq) {
        ERROR("%d will exceed queue limit of %d pages", numpages, ns->maxppq);
        return NULL;
    }

    // allocate the page array
    unvme_pal_t* pal = zalloc(sizeof(unvme_pal_t) +
                               numpages * sizeof(unvme_page_t));
    pal->count = numpages;
    pal->pa->qid = qid;

    if (client_alloc(ns, pal)) {
        free(pal);
        return NULL;
    }

    // add to the page allocation track list
    if (!ioq->pal) {
        ioq->pal = pal;
        pal->prev = pal->next = pal;
    } else {
        pal->prev = ioq->pal->prev;
        pal->next = ioq->pal;
        ioq->pal->prev->next = pal;
        ioq->pal->prev = pal;
    }

    ioq->pac += pal->count;
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
    unvme_queue_t* ioq = ((unvme_session_t*)(ns->ses))->queues + pa->qid;
    unvme_pal_t* pal = (unvme_pal_t*)((void*)pa - offsetof(unvme_pal_t, pa));
    if (!ioq->pal || pal->pa != pa || client_free(ns, pal)) return -1;

    // remove from the page allocation entry
    if (pal->next == pal) {
        ioq->pal = NULL;
    } else {
        pal->next->prev = pal->prev;
        pal->prev->next = pal->next;
        if (ioq->pal == pal) ioq->pal = pal->next;
    }
    ioq->pac -= pal->count;

    free(pal);
    return 0;
}

/**
 * Read a page array and then poll to wait for completion.
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
 * Write a page array and then poll to wait for completion.
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
 * Read a page array asynchronously (caller is to poll for completion).
 * @param   ns          namespace handle
 * @param   pa          page array
 * @return  0 if ok else error code.
 */
int unvme_aread(const unvme_ns_t* ns, unvme_page_t* pa)
{
    return client_rw(ns, pa, NVME_CMD_READ);
}

/**
 * Write a page array asynchronously (caller is to poll for completion).
 * @param   ns          namespace handle
 * @param   pa          page array
 * @return  0 if ok else error code.
 */
int unvme_awrite(const unvme_ns_t* ns, unvme_page_t* pa)
{
    return client_rw(ns, pa, NVME_CMD_WRITE);
}

