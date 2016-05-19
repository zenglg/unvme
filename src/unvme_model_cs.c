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
 * @brief UNVMe CS model server support routines.
 */

#include <string.h>
#include <signal.h>

#include "unvme.h"

/// CS client server model
int unvme_model = UNVME_MODEL_CS;

extern int unvme_devcount;
extern sem_t unvme_sem;
static void* unvme_csif_thread(void* arg);


/**
 * Allocate DMA data pool in an IO queue.
 * @param   ioq         io queue
 */
void unvme_datapool_alloc(unvme_queue_t* ioq)
{
    unvme_device_t* dev = ioq->ses->dev;
    unvme_ns_t* ns = &ioq->ses->ns;
    unvme_datapool_t* datapool = &ioq->datapool;
    size_t datasize = ns->maxppq * ns->pagesize;
    size_t statsize = ns->maxppq * sizeof(unvme_piostat_t);
    size_t cpqsize = sizeof(unvme_piocpq_t) + ioq->nvq->size * sizeof(unvme_page_t*);

    DEBUG_FN("%x.%d", dev->vfiodev->pci, ioq->id);
    char path[32];
    sprintf(path, "/unvme.data.%x.%d.%d", dev->vfiodev->pci, ioq->ses->id, ioq->id);
    datapool->sf = shm_create(path, datasize + statsize + cpqsize);
    if (!datapool->sf) FATAL();
    datapool->data = vfio_dma_map(dev->vfiodev, datasize, datapool->sf->buf);
    if (!datapool->data) FATAL();
    datapool->piostat = datapool->sf->buf + datasize;
    datapool->piocpq = (void*)(datapool->piostat) + statsize;
    datapool->piocpq->size = ioq->nvq->size;

    // allocate PRP list for large IO transfer
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
    unvme_datapool_t* datapool = &ioq->datapool;

    DEBUG_FN("%x.%d", ioq->ses->dev->vfiodev->pci, ioq->ses->id, ioq->id);
    if (vfio_dma_free(datapool->prplist) ||
        vfio_dma_unmap(datapool->data) ||
        shm_delete(datapool->sf)) FATAL();
}

/**
 * Process client open command to create io queues.
 * @param   ses         session
 */
static void unvme_client_open(unvme_session_t* ses)
{
    unvme_device_t* dev = ses->dev;
    unvme_msg_t* msg = ses->csif.msgbuf;
    unvme_session_t* newses = unvme_do_open(dev, dev->vfiodev->pci, msg->cpid,
                                            msg->nsid, msg->qcount, msg->qsize);
    if (newses) {
        msg->sid = newses->id;
        memcpy(&msg->ns, &newses->ns, sizeof(unvme_ns_t));
        msg->stat = 0;
    } else {
        msg->sid = -1;
        msg->stat = -1;
    }
    msg->ack = msg->cmd;
}

/**
 * Process client close request.
 * @param   ses         session
 */
static void unvme_client_close(unvme_session_t* ses)
{
    unvme_msg_t* msg = ses->csif.msgbuf;
    unvme_do_close(ses->dev, msg->cpid, msg->sid);
    msg->stat = 0;
    msg->ack = msg->cmd;
}

/**
 * Process client allocation request.
 * @param   ioq         io queue
 * @param   msg         message
 */
static inline void unvme_client_alloc(unvme_queue_t* ioq, unvme_msg_t* msg)
{
    msg->pgid = unvme_do_alloc(ioq);
    msg->stat = msg->pgid >= 0 ? 0 : -1;
    msg->ack = msg->cmd;
}

/**
 * Process client free request.
 * @param   ioq         io queue
 * @param   msg         message
 */
static inline void unvme_client_free(unvme_queue_t* ioq, unvme_msg_t* msg)
{
    msg->stat = unvme_do_free(ioq, msg->pgid);
    msg->ack = msg->cmd;
}

/**
 * Process client write/read request.
 * @param   ioq         io queue
 * @param   msg         message
 */
static inline void unvme_client_rw(unvme_queue_t* ioq, unvme_msg_t* msg)
{
    msg->stat = unvme_do_rw(ioq, msg->pa, msg->cmd);
    msg->ack = msg->cmd;
}

/**
 * Create a session client server interface to process client commands. 
 * @param   ses         session
 */
static void unvme_csif_create(unvme_session_t* ses)
{
    char path[32];
    sprintf(path, "/unvme.csif.%x.%d", ses->dev->vfiodev->pci, ses->id);
    unvme_csif_t* csif = &ses->csif;
    csif->mqsize = ses->qcount + 1;
    csif->msglen = sizeof(unvme_msg_t) + sizeof(unvme_page_t) * ses->ns.maxppio;
    csif->sf = shm_create(path, sizeof(pthread_spinlock_t) + sizeof(sem_t) +
                                csif->msglen * ses->qcount +
                                sizeof(int) * (3 + csif->mqsize));
    if (!csif->sf) FATAL();
    csif->lock = (pthread_spinlock_t*)csif->sf->buf;
    csif->sem = (sem_t*)(csif->lock + 1);
    csif->msgbuf = csif->sem + 1;
    csif->mqcount = (int*)(csif->msgbuf + csif->msglen * ses->qcount);
    csif->mqhead = csif->mqcount + 1;
    csif->mqtail = csif->mqhead + 1;
    csif->mq = csif->mqtail + 1;

    if (pthread_spin_init(csif->lock, PTHREAD_PROCESS_SHARED) ||
        sem_init(csif->sem, 1, 0) ||
        pthread_create(&csif->thread, 0, unvme_csif_thread, ses)) FATAL();

    sem_wait(&unvme_sem);
}

/**
 * Delete a session client server interface.
 * @param   ses         session
 */
static void unvme_csif_delete(unvme_session_t* ses)
{
    unvme_csif_t* csif = &ses->csif;
    if (!csif->sf) return;
    csif->stop = 1;
    sem_post(csif->sem);
    pthread_join(csif->thread, 0);
    sem_destroy(csif->sem);
    pthread_spin_destroy(csif->lock);
    shm_delete(csif->sf);
}

/**
 * Receive a message from client.
 * @param   csif        interface
 * @return  session queue index.
 */
static inline int csif_recv(unvme_csif_t* csif)
{
    while (*csif->mqcount == 0) {
        if (csif->stop) return -1;
        sem_wait(csif->sem);
    }

    int* mqhead = csif->mqhead;
    int sqi = csif->mq[*mqhead];
    if (++(*mqhead) == csif->mqsize) *mqhead = 0;
    atomic_sub(csif->mqcount, 1);
    return sqi;
}

/**
 * Thread to process client commands.
 * @param   arg         queue
 */
static void* unvme_csif_thread(void* arg)
{
    unvme_session_t* ses = arg;
    unvme_csif_t* csif = &ses->csif;
    unvme_msg_t* msg = csif->msgbuf;

    INFO_FN("%x: start ses=%d", ses->dev->vfiodev->pci, ses->id);
    sem_post(&unvme_sem);

    if (ses->id == 0) {
        for (;;) {
            sem_wait(csif->sem);
            if (csif->stop) goto end;

            pthread_spin_lock(&ses->dev->lock);
            switch (msg->cmd) {
            case UNVME_CMD_OPEN:
                unvme_client_open(ses);
                break;
            case UNVME_CMD_CLOSE:
                unvme_client_close(ses);
                break;
            default:
                ERROR("cmd=%d", msg->cmd);
                goto end;
            }
            pthread_spin_unlock(&ses->dev->lock);
        }
    } else {
        for (;;) {
            int sqi = csif_recv(csif);
            if (sqi < 0) goto end;
            unvme_queue_t* ioq = ses->queues + sqi;
            msg = csif->msgbuf + sqi * csif->msglen;

            switch (msg->cmd) {
            case UNVME_CMD_ALLOC:
                unvme_client_alloc(ioq, msg);
                break;
            case UNVME_CMD_FREE:
                unvme_client_free(ioq, msg);
                break;
            case UNVME_CMD_READ:
            case UNVME_CMD_WRITE:
                unvme_client_rw(ioq, msg);
                break;
            default:
                ERROR("ses=%d.%d cmd=%d", ses->id, sqi, msg->cmd);
                goto end;
            }
        }
    }

end:
    INFO_FN("%x: end ses=%d", ses->dev->vfiodev->pci, ses->id);
    return 0;
}

/**
 * Create session extended function for the model.
 * @param   ses         session
 * @return  0 if ok else -1.
 */
void unvme_session_create_ext(unvme_session_t* ses)
{
    if (ses->id > 0) unvme_tpc_create(ses);
    unvme_csif_create(ses);
}

/**
 * Delete session extended function for the model.
 * @param   ses         session
 * @return  0 if ok else -1.
 */
void unvme_session_delete_ext(unvme_session_t* ses)
{
    if (ses->id > 0) unvme_csif_delete(ses);
    unvme_tpc_delete(ses);
}

/**
 * Program termination signal handler function.
 * @param   sig         signal number
 */
static void unvme_sighandler(int sig)
{
    INFO("\n===\nkill %d", sig);
    unvme_cleanup();
    exit(1);
}

/**
 * Main program.
 */
int main(int argc, char* argv[])
{
    const char* usage = "Usage: %s [-f] pciname...\n\
         -f       run program in foreground\n\
         pciname  PCI device name (as BB:DD.F format)\n";

    extern char* unvme_logname;
    char* prog = strrchr(argv[0], '/');
    prog = prog ? prog + 1 : argv[0];
    int run_fg = 0;

    int opt;
    while ((opt = getopt(argc, argv, "f")) != -1) {
        switch (opt) {
        case 'f':
            run_fg = 1;
            unvme_logname = NULL;
            break;
        default:
            goto usage;
        }
    }
    if (optind >= argc) goto usage;

    if (!run_fg) {
        pid_t child = fork();
        if (child > 0) return 0;
        else if (child < 0) {
            perror("fork");
            exit(1);
        }
    }

    if (signal(SIGINT,  unvme_sighandler) == SIG_ERR ||
        signal(SIGABRT, unvme_sighandler) == SIG_ERR ||
        signal(SIGTERM, unvme_sighandler) == SIG_ERR ||
        signal(SIGQUIT, unvme_sighandler) == SIG_ERR) {
        perror("signal");
        exit(1);
    }

    // allocate and initialize each specified device
    unvme_device_t* dev = unvme_init(argc - optind);
    int i;
    for (i = 0; i < unvme_devcount; i++) {
        char* pciname = argv[optind + i];
        int b, d, f;
        if (sscanf(pciname, "%02x:%02x.%1x", &b, &d, &f) != 3) {
            FATAL("invalid PCI device %s (expect BB:DD.F format)", pciname);
        }
        int pci = (b << 16) + (d << 8) + f;
        unvme_dev_init(dev + i, pci);
    }

    // loop and detect dead clients
    for (i = 0; ;) {
        sleep(8);
        pthread_spin_lock(&dev[i].lock);
        unvme_session_t* ses = dev[i].ses->next;
        while (ses != dev[i].ses) {
            if (kill(ses->cpid, 0)) {
                INFO("===\ndead client pid %d", ses->cpid);
                unvme_do_close(&dev[i], ses->cpid, 0);
                break;
            }
            ses = ses->next;
        }
        pthread_spin_unlock(&dev[i].lock);
        if (++i == unvme_devcount) i = 0;
    }

    /* NOT REACHED */
    return 0;

usage:
    fprintf(stderr, usage, prog);
    return 1;
}

