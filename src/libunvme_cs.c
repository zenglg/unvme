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
 * @brief UNVMe CS model client specific routines.
 */

#include <string.h>

#include "unvme.h"

extern unvme_client_t client;


/**
 * Map the client server interface.
 * @param   csif        interface handle
 * @param   ses         session
 * @param   pci         PCI device number
 */
static void csif_map(unvme_csif_t* csif, unvme_session_t* ses, int pci)
{
    char path[32];
    sprintf(path, "/unvme.csif.%x.%d", pci, ses ? ses->id : 0);
    csif->sf = shm_map(path);
    if (!csif->sf) exit(1);
    csif->lock = (pthread_spinlock_t*)csif->sf->buf;
    csif->sem = (sem_t*)(csif->lock + 1);
    csif->msgbuf = csif->sem + 1;
    if (ses) {
        csif->mqsize = ses->qcount + 1;
        csif->msglen = sizeof(unvme_msg_t) + sizeof(unvme_page_t) * ses->ns.maxppio;
        csif->mqcount = (int*)(csif->msgbuf + csif->msglen * ses->qcount);
        csif->mqhead = csif->mqcount + 1;
        csif->mqtail = csif->mqhead + 1;
        csif->mq = csif->mqtail + 1;
    }
}

/**
 * Unmap the client server interface.
 * @param   csif        interface handle
 */
static inline void csif_unmap(unvme_csif_t* csif)
{
    shm_unmap(csif->sf);
}

/**
 * Send an admin type command to the server and wait for acknowledgement.
 * @param   csif    interface
 * @param   msg     message
 */
static inline void csif_admin(unvme_csif_t* csif, unvme_msg_t* msg)
{
    msg->ack = UNVME_CMD_NULL;
    msg->stat = -1;
    sem_post(csif->sem);
    while (msg->ack != msg->cmd) sched_yield();
}

/**
 * Send an IO queue type command to server and wait for acknowledgement.
 * @param   csif    interface
 * @param   qid     client session queue id
 * @param   msg     message
 */
static inline void csif_ioq(unvme_csif_t* csif, int qid, unvme_msg_t* msg)
{
    msg->ack = UNVME_CMD_NULL;
    msg->stat = -1;

    pthread_spin_lock(csif->lock);
    csif->mq[*csif->mqtail] = qid;
    if (++(*csif->mqtail) == csif->mqsize) *csif->mqtail = 0;
    pthread_spin_unlock(csif->lock);

    atomic_add(csif->mqcount, 1);
    sem_post(csif->sem);

    while (msg->ack != msg->cmd) sched_yield();
}

/**
 * Map the data pool.
 * @param   q           client queue
 * @param   pci         PCI device number
 * @param   qid         queue id
 */
static void datapool_map(unvme_queue_t* q, int pci, int qid)
{
    size_t datasize = q->ses->ns.maxppq * q->ses->ns.pagesize;
    size_t statsize = q->ses->ns.maxppq * sizeof(unvme_piostat_t);
    char path[32];
    sprintf(path, "/unvme.data.%x.%d.%d", pci, q->ses->id, qid);
    q->datapool.sf = shm_map(path);
    if (!q->datapool.sf) exit(1);
    q->datapool.piostat = q->datapool.sf->buf + datasize;
    q->datapool.piocpq = (void*)(q->datapool.piostat) + statsize;
}

/**
 * Unmap the client server interface.
 * @param   q           client queue
 */
static inline void datapool_unmap(unvme_queue_t* q)
{
    shm_unmap(q->datapool.sf);
}

/**
 * Open a client session.
 * @param   pci         PCI device number
 * @param   nsid        namespace id
 * @param   qcount      queue count
 * @param   qsize       queue size
 * @return  newly created session
 */
unvme_session_t* client_open(int pci, int nsid, int qcount, int qsize)
{
    unvme_session_t* ses = NULL;
    if (!client.csif.msgbuf) csif_map(&client.csif, NULL, pci);
    unvme_msg_t* msg = client.csif.msgbuf;

    // only one client process can access the admin message at a time
    pthread_spin_lock(client.csif.lock);

    msg->cmd = UNVME_CMD_OPEN;
    msg->cpid = getpid();
    msg->nsid = nsid;
    msg->qcount = qcount;
    msg->qsize = qsize;
    csif_admin(&client.csif, msg);
    if (msg->stat) goto end;

    ses = zalloc(sizeof(unvme_session_t) + sizeof(unvme_queue_t) * qcount);
    ses->queues = (unvme_queue_t*)(ses + 1);
    ses->id = msg->sid;
    ses->cpid = msg->cpid;
    ses->qcount = qcount;
    ses->qsize = qsize;
    memcpy(&ses->ns, &msg->ns, sizeof(unvme_ns_t));
    ses->ns.ses = ses;

    int i;
    for (i = 0; i < ses->qcount; i++) {
        ses->queues[i].ses = ses;
        datapool_map(ses->queues + i, pci, ses->id + i);
    }
    csif_map(&ses->csif, ses, pci);

    if (!client.ses) {
        client.ses = ses;
        ses->next = ses;
        ses->prev = ses;
    } else {
        ses->next = client.ses;
        ses->prev = client.ses->prev;
        client.ses->prev->next = ses;
        client.ses->prev = ses;
    }

end:
    pthread_spin_unlock(client.csif.lock);
    return ses;
}

/**
 * Close a client session and delete all its io queues.
 * @param   ns          namespace
 * @return  0 if ok else -1.
 */
int client_close(const unvme_ns_t* ns)
{
    int err = -1;
    unvme_session_t* ses = ns->ses;

    // only one client process can access the admin message at a time
    pthread_spin_lock(client.csif.lock);

    unvme_msg_t* msg = client.csif.msgbuf;
    msg->cmd = UNVME_CMD_CLOSE;
    msg->cpid = ses->cpid;
    msg->sid = ses->id;
    csif_admin(&client.csif, msg);
    if (msg->stat) goto end;

    csif_unmap(&ses->csif);
    int i;
    for (i = 0; i < ses->qcount; i++) datapool_unmap(ses->queues + i);
    if (ses == ses->next) {
        client.ses = NULL;
    } else {
        ses->next->prev = ses->prev;
        ses->prev->next = ses->next;
        if (client.ses == ses) client.ses = ses->next;
    }

    free(ses);
    err = 0;

end:
    pthread_spin_unlock(client.csif.lock);
    return err;
}

/**
 * Allocate a client page entry.
 * @param   ns          namespace
 * @param   pal         client page entry
 * @return  0 if ok else -1.
 */
int client_alloc(const unvme_ns_t* ns, unvme_pal_t* pal)
{
    unvme_session_t* ses = ns->ses;
    unvme_csif_t* csif = &ses->csif;
    unvme_page_t* p = pal->pa;
    int qid = p->qid;
    unvme_msg_t* msg = csif->msgbuf + qid * csif->msglen;
    int i;
    for (i = 0; i < pal->count; i++) {
        msg->cmd = UNVME_CMD_ALLOC;
        csif_ioq(csif, qid, msg);
        if (msg->stat) return -1;
        p->id = msg->pgid;
        p->qid = qid;
        p->nlb = ns->nbpp;
        p->buf = ses->queues[qid].datapool.sf->buf + p->id * ns->pagesize;
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
    unvme_session_t* ses = ns->ses;
    unvme_csif_t* csif = &ses->csif;
    int qid = pal->pa->qid;
    unvme_msg_t* msg = csif->msgbuf + qid * csif->msglen;
    int i;
    for (i = 0; i < pal->count; i++) {
        msg->pgid = pal->pa[i].id;
        msg->cmd = UNVME_CMD_FREE;
        csif_ioq(csif, qid, msg);
        if (msg->stat) return -1;
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
    unvme_session_t* ses = ns->ses;
    unvme_csif_t* csif = &ses->csif;
    int qid = pa->qid;
    unvme_msg_t* msg = csif->msgbuf + qid * csif->msglen;
    ses->queues[qid].datapool.piostat[pa->id].cpa = pa;
    msg->cmd = opc;
    int numpages = (pa->nlb + ns->nbpp - 1) / ns->nbpp;
    memcpy(msg->pa, pa, numpages * sizeof(unvme_page_t));
    csif_ioq(csif, qid, msg);
    return 0;
}

