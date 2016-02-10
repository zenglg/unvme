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

extern client_main_t client;


/**
 * Map the client server interface.
 * @param   csif        interface handle
 * @param   vfid        vfio device number
 * @param   qid         queue id
 */
static void csif_map(unvme_csif_t* csif, int vfid, int qid)
{
    char path[32];
    sprintf(path, "/unvme.csif.%d.%d", vfid, qid);
    csif->sf = shm_map(path);
    if (!csif->sf) exit(1);
    csif->req = (sem_t*)(csif->sf->buf);
    csif->ack = csif->req + 1;
    csif->msg = (unvme_msg_t*)(csif->ack + 1);
}

/**
 * Unmap the client server interface.
 * @param   csif        interface handle
 */
static void csif_unmap(unvme_csif_t* csif)
{
    shm_unmap(csif->sf);
}

/**
 * Send a command to the server and wait for response.
 * @param   csif    interface handle
 * @return  0 if ok else -1.
 */
static int csif_cmd(unvme_csif_t* csif)
{
    static u32 msgid = 0;
    unvme_msg_t* msg = csif->msg;

    msg->id = ++msgid;
    msg->stat = -1;
    if (sem_post(csif->req) || sem_wait(csif->ack)) return msg->stat;
    return 0;
}

/**
 * Map the data pool.
 * @param   q           client queue
 * @param   vfid        vfio device number
 * @param   qid         queue id
 */
static void datapool_map(client_queue_t* q, int vfid, int qid)
{
    size_t datasize = q->ses->ns.maxppq * q->ses->ns.pagesize;
    size_t statsize = q->ses->ns.maxppq * sizeof(unvme_piostat_t);
    char path[32];
    sprintf(path, "/unvme.data.%d.%d", vfid, qid);
    q->datapool.sf = shm_map(path);
    if (!q->datapool.sf) exit(1);
    q->datapool.piostat = q->datapool.sf->buf + datasize;
    q->datapool.piocpq = (void*)(q->datapool.piostat) + statsize;
}

/**
 * Unmap the client server interface.
 * @param   q           client queue
 */
static void datapool_unmap(client_queue_t* q)
{
    shm_unmap(q->datapool.sf);
}

/**
 * Open a client session.
 * @param   ses         session
 * @param   vfid        vfio id
 * @param   nsid        namespace id
 * @return  0 if ok else -1.
 */
int client_open(client_session_t* ses, int vfid, int nsid)
{
    if (!client.ses) csif_map(&client.csifadm, vfid, 0);

    unvme_msg_t* msg = client.csifadm.msg;
    msg->cmd = UNVME_CMD_OPEN;
    msg->cpid = client.cpid;
    msg->nsid = nsid;
    msg->qcount = ses->qcount;
    msg->qsize = ses->qsize;
    if (csif_cmd(&client.csifadm) || msg->stat) return -1;

    memcpy(&ses->ns, &msg->ns, sizeof(unvme_ns_t));
    memcpy(ses->ns.model, msg->ns.model, sizeof(ses->ns.model));
    ses->ns.ses = ses;

    int i;
    int nvqid = msg->nvqid;
    for (i = 0; i < ses->qcount; i++, nvqid++) {
        client_queue_t* q = &ses->queues[i];
        q->id = i;
        q->ses = ses;
        q->nvqid = nvqid;
        csif_map(&q->csif, vfid, nvqid);
        datapool_map(q, vfid, nvqid);
    }

    return 0;
}

/**
 * Close a client session and delete all its io queues.
 * @param   ses         session
 * @return  0 if ok else -1.
 */
int client_close(client_session_t* ses)
{
    unvme_msg_t* msg = client.csifadm.msg;
    msg->cmd = UNVME_CMD_CLOSE;
    msg->cpid = client.cpid;
    msg->nvqid = ses->queues[0].nvqid;

    if (csif_cmd(&client.csifadm) || msg->stat) return -1;

    int i;
    for (i = 0; i < ses->qcount; i++) {
        client_queue_t* q = &ses->queues[i];
        datapool_unmap(q);
        csif_unmap(&q->csif);
    }

    if (ses->next == ses) {
        csif_unmap(&client.csifadm);
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
    unvme_msg_t* msg = q->csif.msg;
    unvme_page_t* p = pal->pa;
    int i;
    for (i = 0; i < pal->count; i++) {
        msg->cmd = UNVME_CMD_ALLOC;
        if (csif_cmd(&q->csif) || msg->stat) return -1;
        p->id = msg->pgid;
        p->qid = q->id;
        p->nlb = q->ses->ns.nbpp;
        p->buf = q->datapool.sf->buf + p->id * q->ses->ns.pagesize;
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
    unvme_msg_t* msg = q->csif.msg;
    for (i = 0; i < pal->count; i++) {
        msg->cmd = UNVME_CMD_FREE;
        msg->pgid = pal->pa[i].id;
        if (csif_cmd(&q->csif) || msg->stat) return -1;
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
    client_queue_t* q = &((client_session_t*)(ns->ses))->queues[pa->qid];
    int ustat = q->datapool.piostat[pa->id].ustat;
    if (ustat != UNVME_PS_READY) {
        ERROR("page %d is busy stat=%d", pa->id, ustat);
        return -1;
    }
    q->datapool.piostat[pa->id].cpa = pa;
    unvme_msg_t* msg = q->csif.msg;
    msg->cmd = opc;
    int numpages = (pa->nlb + ns->nbpp - 1) / ns->nbpp;
    memcpy(msg->pa, pa, numpages * sizeof(unvme_page_t));
    return csif_cmd(&q->csif);
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
    client_queue_t* q = &((client_session_t*)(ns->ses))->queues[pa->qid];
    return unvme_tpc_poll(&q->datapool, pa, sec);
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
    client_queue_t* q = &((client_session_t*)(ns->ses))->queues[qid];
    return unvme_tpc_apoll(&q->datapool, sec);
}

