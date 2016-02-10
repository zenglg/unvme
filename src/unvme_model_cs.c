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

int unvme_model = UNVME_MODEL_CS;       /// CS client server model

extern int unvme_devcount;
extern sem_t unvme_sem;


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
    char path[32];
    sprintf(path, "/unvme.data.%d.%d", dev->vfiodev->id, ioq->nvq->id);
    datapool->sf = shm_create(path, datasize + statsize + cpqsize);
    if (!datapool->sf) FATAL();
    datapool->data = vfio_dma_map(dev->vfiodev, datasize, datapool->sf->buf);
    if (!datapool->data) FATAL();
    datapool->piostat = datapool->sf->buf + datasize;
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
    if (vfio_dma_unmap(datapool->data)) FATAL();
    if (shm_delete(datapool->sf)) FATAL();
}

/**
 * Create a per queue client server interface to process client commands. 
 * @param   q           queue
 */
void unvme_csif_create(unvme_queue_t* q)
{
    void* unvme_csif_thread(void* arg);
    char path[32];
    sprintf(path, "/unvme.csif.%d.%d", q->dev->vfiodev->id, q->nvq->id);
    size_t size = 2 * sizeof(sem_t) +
                  sizeof(unvme_msg_t) + sizeof(unvme_page_t) * q->ns->maxppio;
    q->csif.sf = shm_create(path, size);
    if (!q->csif.sf) FATAL();
    q->csif.req = (sem_t*)(q->csif.sf->buf);
    q->csif.ack = q->csif.req + 1;
    q->csif.msg = (unvme_msg_t*)(q->csif.ack + 1);

    if (sem_init(q->csif.req, 1, 0) || sem_init(q->csif.ack, 1, 0)) {
        FATAL("sem_init");
    }
    if (pthread_create(&q->csif.thread, 0, unvme_csif_thread, q)) {
        FATAL("pthread_create");
    }
    sem_wait(&unvme_sem);
}

/**
 * Delete a per queue client server interface.
 * @param   q           queue
 */
void unvme_csif_delete(unvme_queue_t* q)
{
    if (!q->csif.sf) return;

    q->csif.msg->cmd = UNVME_CMD_QUIT;
    sem_post(q->csif.req);
    if (q->csif.thread) pthread_join(q->csif.thread, 0);
    sem_destroy(q->csif.req);
    sem_destroy(q->csif.ack);
    if (shm_delete(q->csif.sf)) FATAL();
}

/**
 * Create admin queue model extended function.
 * @param   adminq      admin queue
 */
void unvme_adminq_create_ext(unvme_queue_t* adminq)
{
    unvme_csif_create(adminq);
}

/**
 * Delete admin queue model extended function.
 * @param   adminq      admin queue
 */
void unvme_adminq_delete_ext(unvme_queue_t* adminq)
{
    unvme_csif_delete(adminq);
}

/**
 * Create IO queue model extended function.
 * @param   ioq         io queue
 */
void unvme_ioq_create_ext(unvme_queue_t* ioq)
{
    unvme_tpc_create(ioq);
    unvme_csif_create(ioq);
}

/**
 * Delete IO queue model extended function.
 * @param   ioq         io queue
 */
void unvme_ioq_delete_ext(unvme_queue_t* ioq)
{
    unvme_csif_delete(ioq);
    unvme_tpc_delete(ioq);
}

/**
 * Receive a client message.
 * @param   q           queue
 * @return  0 if ok else -1.
 */
static int unvme_csif_recv(unvme_queue_t* q)
{
    int err = sem_wait(q->csif.req);
    if (err) ERROR("sem_wait q=%d", q->nvq->id);
    return err;
}

/**
 * Send a reply message to client.
 * @param   q           queue
 * @return  0 if ok else -1.
 */
static int unvme_csif_ack(unvme_queue_t* q)
{
    q->csif.msg->cmd = UNVME_CMD_NULL;
    int err = sem_post(q->csif.ack);
    if (err) ERROR("sem_post q=%d", q->nvq->id);
    return err;
}

/**
 * Process client open command to create io queues.
 * @param   dev         device context
 */
static void unvme_client_open(unvme_device_t* dev)
{
    unvme_msg_t* msg = dev->adminq.csif.msg;
    unvme_queue_t* ioq = unvme_do_open(dev, dev->vfiodev->id, msg->cpid,
                                       msg->nsid, msg->qcount, msg->qsize);
    if (ioq) {
        msg->nvqid = ioq->nvq->id;
        memcpy(&msg->ns, ioq->ns, sizeof(unvme_ns_t));
        msg->stat = 0;
    } else {
        msg->nvqid = 0;
        msg->stat = -1;
    }
    unvme_csif_ack(&dev->adminq);
}

/**
 * Process client close request.
 * @param   dev         device context
 */
static void unvme_client_close(unvme_device_t* dev)
{
    unvme_msg_t* msg = dev->adminq.csif.msg;
    unvme_do_close(dev, msg->cpid, msg->nvqid);
    msg->stat = 0;
    unvme_csif_ack(&dev->adminq);
}

/**
 * Process client allocation request.
 * @param   ioq         io queue
 */
static void unvme_client_alloc(unvme_queue_t* ioq)
{
    unvme_msg_t* msg = ioq->csif.msg;
    msg->pgid = unvme_do_alloc(ioq);
    msg->stat = msg->pgid >= 0 ? 0 : -1;
    unvme_csif_ack(ioq);
}

/**
 * Process client free request.
 * @param   ioq         io queue
 */
static void unvme_client_free(unvme_queue_t* ioq)
{
    unvme_msg_t* msg = ioq->csif.msg;
    msg->stat = unvme_do_free(ioq, msg->pgid);
    unvme_csif_ack(ioq);
}

/**
 * Process client write/read request.
 * @param   ioq         io queue
 */
static inline void unvme_client_rw(unvme_queue_t* ioq)
{
    unvme_msg_t* msg = ioq->csif.msg;
    msg->stat = unvme_do_rw(ioq, msg->pa, msg->cmd);
    unvme_csif_ack(ioq);
}

/**
 * Thread to process client commands.
 * @param   arg         queue
 */
void* unvme_csif_thread(void* arg)
{
    unvme_queue_t* q = arg;
    unvme_device_t* dev = q->dev;
    unvme_msg_t* msg = q->csif.msg;
    int quit = 0;

    DEBUG_FN("q=%d start", q->nvq->id);
    sem_post(&unvme_sem);

    if (q->nvq->id == 0) {
        // admin message queue processes open and close commands
        while (!quit) {
            if (unvme_csif_recv(q)) break;

            pthread_spin_lock(&dev->lock);
            switch (msg->cmd) {
            case UNVME_CMD_OPEN:
                unvme_client_open(dev);
                break;
            case UNVME_CMD_CLOSE:
                unvme_client_close(dev);
                break;
            case UNVME_CMD_QUIT:
                quit = 1;
                break;
            default:
                ERROR("q=0 id=%d cmd=%d", msg->id, msg->cmd);
            }
            pthread_spin_unlock(&dev->lock);
        }
    } else {
        while (!quit) {
            if (unvme_csif_recv(q)) break;

            switch (msg->cmd) {
            case UNVME_CMD_ALLOC:
                unvme_client_alloc(q);
                break;
            case UNVME_CMD_FREE:
                unvme_client_free(q);
                break;
            case UNVME_CMD_READ:
            case UNVME_CMD_WRITE:
                unvme_client_rw(q);
                break;
            case UNVME_CMD_QUIT:
                quit = 1;
                break;
            default:
                ERROR("q=%d id=%d cmd=%d", q->nvq->id, msg->id, msg->cmd);
            }
        }
    }

    DEBUG_FN("q=%d exit", q->nvq->id);
    return 0;
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
    const char* usage = "Usage: %s [-f] vfioname...\n\
         -f       run program in foreground\n\
         vfioname vfio device name list\n";

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
    int d;
    for (d = 0; d < unvme_devcount; d++) {
        int vfid;
        if (sscanf(argv[optind + d], "/dev/vfio/%d", &vfid) != 1) {
            FATAL("invalid vfio device name %s", argv[optind + d]);
        }
        unvme_dev_init(dev + d, vfid);
    }

    // loop and detect dead clients
    for (d = 0; ;) {
        int i;
        sleep(8);
        pthread_spin_lock(&dev[d].lock);
        unvme_queue_t* ioq = dev[d].ioqlist;
        for (i = 0; i < dev[d].numioqs; i++) {
            if (kill(ioq->cpid, 0)) {
                INFO("===\ndead client pid %d", ioq->cpid);
                unvme_do_close(&dev[d], ioq->cpid, 0);
                break;
            }
            ioq = ioq->next;
        }
        pthread_spin_unlock(&dev[d].lock);
        if (++d == unvme_devcount) d = 0;
    }

usage:
    fprintf(stderr, usage, prog);
    return 1;
}

