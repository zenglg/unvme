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
 * @brief UNVMe core common functions.
 */

#include <sys/mman.h>
#include <string.h>
#include <signal.h>

#include "unvme.h"

// Global variables
unvme_device_t* unvme_devlist = 0;      ///< main device list array
int             unvme_devcount = 0;     ///< device count
sem_t           unvme_sem;              ///< thread semaphore

/// model names
char* unvme_model_names[] = { "APC", "TPC", "CS" };

/// default log filename
char* unvme_logname = "/dev/shm/unvme.log";

extern int unvme_model;


/**
 * Create a namespace object.
 * @param   ses         session
 * @param   nsid        namespace id
 */
static void unvme_ns_init(unvme_session_t* ses, int nsid)
{
    unvme_device_t* dev = ses->dev;
    unvme_ns_t* ns = &ses->ns;
    strcpy(ns->model, unvme_model_names[unvme_model]);
    ns->maxqsize = dev->nvmedev->maxqsize;
    ns->pagesize = 1 << dev->nvmedev->pageshift;

    vfio_dma_t* dma = vfio_dma_alloc(dev->vfiodev, ns->pagesize << 1);
    if (!dma) FATAL();
    if (nvme_cmd_identify(dev->nvmedev, nsid,
                          dma->addr, dma->addr + ns->pagesize)) FATAL();

    if (nsid == 0) {
        int i;
        nvme_identify_ctlr_t* idc = (nvme_identify_ctlr_t*)dma->buf;
        ns->vid = idc->vid;
        memcpy(ns->sn, idc->sn, sizeof(ns->sn));
        for (i = sizeof(ns->sn) - 1; i > 0 && ns->sn[i] == ' '; i--) ns->sn[i] = 0;
        memcpy(ns->mn, idc->mn, sizeof(ns->mn));
        for (i = sizeof(ns->mn) - 1; i > 0 && ns->mn[i] == ' '; i--) ns->mn[i] = 0;
        memcpy(ns->fr, idc->fr, sizeof(ns->fr));
        for (i = sizeof(ns->fr) - 1; i > 0 && ns->fr[i] == ' '; i--) ns->fr[i] = 0;
        ns->maxppio = 128 * 1024 / ns->pagesize;
        if (idc->mdts) {
            int maxp = 2;
            for (i = 1; i < idc->mdts; i++) maxp *= 2;
            ns->maxppio = maxp;
        }
    } else {
        memcpy(ns, &dev->ses->ns, sizeof(unvme_ns_t));
        nvme_identify_ns_t* idns = (nvme_identify_ns_t*)dma->buf;
        ns->blockcount = (u64) idns->nuse;
        ns->blocksize = 1 << idns->lbaf[idns->flbas & 0xF].lbads;
        if (ns->blocksize > ns->pagesize || ns->blockcount < 8) {
            FATAL("ps=%d bs=%d bc=%ld",
                  ns->pagesize, ns->blocksize, ns->blockcount);
        }
        ns->nbpp = ns->pagesize / ns->blocksize;
        ns->maxbpio = ns->maxppio * ns->nbpp;
    }
    ns->maxppq =  ses->qsize * ns->maxppio;
    ns->maxiopq = ses->qsize - 1;
    ns->ses = ses;
    ns->id = nsid;

    if (vfio_dma_free(dma)) FATAL();
}

/**
 * Create an IO queue.
 * @param   ses         session
 * @param   sqi         session queue id
 */
static void unvme_ioq_create(unvme_session_t* ses, int sqi)
{
    unvme_device_t* dev = ses->dev;
    DEBUG_FN("%d: qs=%d", dev->vfiodev->id, ses->qsize);

    unvme_queue_t* ioq = &ses->queues[sqi];
    ioq->ses = ses;
    int nvqid = (sqi == 0 ? ses->prev->queues[ses->prev->qcount-1].nvq->id
                          : ioq[-1].nvq->id) + 1;
    if (nvqid == 0) FATAL("qid wraps to 0");

    ioq->sqdma = vfio_dma_alloc(dev->vfiodev, ses->qsize * sizeof(nvme_sq_entry_t));
    if (!ioq->sqdma) FATAL();
    ioq->cqdma = vfio_dma_alloc(dev->vfiodev, ses->qsize * sizeof(nvme_cq_entry_t));
    if (!ioq->cqdma) FATAL();

    ioq->nvq = nvme_create_ioq(dev->nvmedev, nvqid, ses->qsize,
                               ioq->sqdma->buf, ioq->sqdma->addr,
                               ioq->cqdma->buf, ioq->cqdma->addr);
    if (!ioq->nvq) FATAL();

    unvme_datapool_alloc(ioq);
    dev->numioqs++;
    INFO_FN("%d: q=%d qs=%d db=%#lx qc=%d",
            dev->vfiodev->id, ioq->nvq->id, ioq->nvq->size,
            (u64)ioq->nvq->sq_doorbell - (u64)dev->nvmedev->reg, dev->numioqs);
}

/**
 * Delete an IO queue.
 * @param   ioq         io queue
 */
static void unvme_ioq_delete(unvme_queue_t* ioq)
{
    DEBUG_FN("q=%d", ioq->nvq->id);
    if (nvme_delete_ioq(ioq->nvq) ||
        vfio_dma_free(ioq->cqdma) || vfio_dma_free(ioq->sqdma)) FATAL();

    unvme_datapool_free(ioq);
    ioq->ses->dev->numioqs--;
    INFO_FN("q=%d qc=%d", ioq->nvq->id, ioq->ses->dev->numioqs);
}

/**
 * Setup admin queue.
 * @param   ses         session
 */
static void unvme_adminq_create(unvme_session_t* ses)
{
    unvme_device_t* dev = ses->dev;
    DEBUG_FN("%d: qs=%d", dev->vfiodev->id, ses->qsize);

    unvme_queue_t* adminq = ses->queues;
    adminq->ses = ses;
    adminq->sqdma = vfio_dma_alloc(dev->vfiodev, ses->qsize * sizeof(nvme_sq_entry_t));
    if (!adminq->sqdma) FATAL();
    adminq->cqdma = vfio_dma_alloc(dev->vfiodev, ses->qsize * sizeof(nvme_cq_entry_t));
    if (!adminq->cqdma) FATAL();

    adminq->nvq = nvme_setup_adminq(dev->nvmedev, ses->qsize,
                                    adminq->sqdma->buf, adminq->sqdma->addr,
                                    adminq->cqdma->buf, adminq->cqdma->addr);
    if (!adminq->nvq) FATAL();
}

/**
 * Delete admin queue.
 * @param   adminq      admin queue
 */
static void unvme_adminq_delete(unvme_queue_t* adminq)
{
    DEBUG_FN("%d", adminq->ses->dev->vfiodev->id);
    if (vfio_dma_free(adminq->sqdma) || vfio_dma_free(adminq->cqdma)) FATAL();
}

/**
 * Create a session and its associated queues.
 * @param   dev         device context
 * @param   cpid        client process id
 * @param   nsid        namespace id
 * @param   qcount      queue count
 * @param   qsize       queue size
 * @return  newly created session.
 */
static unvme_session_t* unvme_session_create(unvme_device_t* dev, int cpid,
                                             int nsid, int qcount, int qsize)
{
    DEBUG_FN("%d: cpid=%d ns=%d qc=%d qs=%d",
             dev->vfiodev->id, cpid, nsid, qcount, qsize);
    if ((nsid == 0 && dev->ses) || (nsid == 0 && qcount != 1) ||
        (nsid != 0 && !dev->ses)) FATAL("nsid %d", nsid);

    // alloca session and an array of queues
    unvme_session_t* ses = zalloc(sizeof(unvme_session_t) +
                                  sizeof(unvme_queue_t) * qcount);
    ses->queues = (unvme_queue_t*)(ses + 1);
    ses->dev = dev;
    ses->cpid = cpid;
    ses->qcount = qcount;
    ses->qsize = qsize;

    if (!dev->ses) {
        dev->ses = ses;
        ses->next = ses;
        ses->prev = ses;
        unvme_adminq_create(ses);
        unvme_ns_init(ses, nsid);
        DEBUG_FN("%d: adminq", dev->vfiodev->id);
    } else {
        ses->next = dev->ses;
        ses->prev = dev->ses->prev;
        dev->ses->prev->next = ses;
        dev->ses->prev = ses;
        unvme_ns_init(ses, nsid);
        int i;
        for (i = 0; i < qcount; i++) unvme_ioq_create(ses, i);
        DEBUG_FN("%d: q=%d-%d bs=%d nb=%lu", dev->vfiodev->id,
                 ses->queues[0].nvq->id, ses->queues[qcount-1].nvq->id,
                 ses->ns.blocksize, ses->ns.blockcount);
    }
    ses->id = ses->queues[0].nvq->id;

    unvme_session_create_ext(ses);
    return ses;
}

/**
 * Delete a session and its associated queues.
 * @param   ses         session
 */
static void unvme_session_delete(unvme_session_t* ses)
{
    unvme_device_t* dev = ses->dev;
    unvme_session_delete_ext(ses);

    if (ses == ses->next) {
        DEBUG_FN("%d: adminq", dev->vfiodev->id);
        unvme_adminq_delete(ses->queues);
        dev->ses = NULL;
    } else {
        DEBUG_FN("%d: q=%d-%d", dev->vfiodev->id, ses->queues[0].nvq->id,
                                ses->queues[ses->qcount-1].nvq->id);
        while (--ses->qcount >= 0) unvme_ioq_delete(&ses->queues[ses->qcount]);
        ses->next->prev = ses->prev;
        ses->prev->next = ses->next;
    }
    free(ses);
}

/**
 * Initialize device.
 * @param   dev         device context
 * @param   vfid        vfio device id
 */
void unvme_dev_init(unvme_device_t* dev, int vfid) 
{
    DEBUG_FN("%d", vfid);
    if (pthread_spin_init(&dev->lock, PTHREAD_PROCESS_PRIVATE)) {
        FATAL("pthread_spin_init");
    }

    dev->vfiodev = vfio_create(vfid);
    if (!dev->vfiodev) FATAL();
    dev->nvmedev = nvme_create(dev->vfiodev->fd);
    if (!dev->nvmedev) FATAL();

    unvme_session_create(dev, 0, 0, 1, 8);
    INFO_FN("%d: (%.40s) is ready", vfid, dev->ses->ns.mn);
}

/**
 * Cleanup a device.
 * @param   dev         device context
 */
void unvme_dev_cleanup(unvme_device_t* dev)
{
    if (dev && dev->vfiodev) {
        INFO_FN("%d", dev->vfiodev->id);

        //pthread_spin_lock(&dev->lock);
        while (dev->ses) unvme_session_delete(dev->ses->prev);
        nvme_delete(dev->nvmedev);
        vfio_delete(dev->vfiodev);
        //pthread_spin_unlock(&dev->lock);
        pthread_spin_destroy(&dev->lock);
    }
}

/**
 * Initialize and allocate a device array list.
 * @param   count       number of devices
 * @return  device list
 */
unvme_device_t* unvme_init(int count)
{
    if (log_open(unvme_logname, "w")) exit(1);

    if (mlockall(MCL_CURRENT|MCL_FUTURE) < 0) FATAL("mlockall");
    if (sem_init(&unvme_sem, 0, 0)) FATAL("sem_init");
    unvme_devlist = zalloc(count * sizeof(unvme_device_t));
    unvme_devcount = count;

    INFO_FN("Model %s", unvme_model_names[unvme_model]);

    return unvme_devlist;
}

/**
 * Cleanup and exit.
 */
void unvme_cleanup()
{
    INFO_FN();
    if (unvme_devlist) {
        while (unvme_devcount--) {
            unvme_dev_cleanup(&unvme_devlist[unvme_devcount]);
        }
        free(unvme_devlist);
        unvme_devlist = 0;
        sem_destroy(&unvme_sem);
    }
    log_close();
}

/**
 * Process unvme_open and create a new session.
 * @param   dev         device context
 * @param   vfid        vfio id
 * @param   cpid        client process id
 * @param   nsid        namespace id
 * @param   qcount      number of io queues
 * @param   qsize       size of each queue
 * @return  the new session or NULL if failure.
 */
unvme_session_t* unvme_do_open(unvme_device_t* dev, int vfid, pid_t cpid,
                               int nsid, int qcount, int qsize)
{
    if (!unvme_devlist) unvme_init(1);

    INFO("===\n%s %d: cpid=%d ns=%d qc=%d qs=%d",
         __func__, vfid, cpid, nsid, qcount, qsize);

    if (!dev) dev = unvme_devlist;
    if (!dev->vfiodev) unvme_dev_init(dev, vfid);

    return unvme_session_create(dev, cpid, nsid, qcount, qsize);
}

/**
 * Process unvme_close for a given client and delete the associated IO queues.
 * @param   dev         device context
 * @param   cpid        client process id
 * @param   sid         session id
 * @return  0 if ok else -1.
 */
int unvme_do_close(unvme_device_t* dev, pid_t cpid, int sid)
{
    INFO("===\n%s %d: cpid=%d.%d", __func__, dev->vfiodev->id, cpid, sid);
    unvme_session_t* ses = dev->ses->next;
    while (ses != dev->ses) {
        if (ses->cpid == cpid && (sid == 0 || sid == ses->id)) {
            unvme_session_delete(ses);
            ses = dev->ses->next;
        } else {
            ses = ses->next;
        }
    }
    ses = dev->ses->prev;
    INFO_FN("last qid %d", ses->queues[ses->qcount-1].nvq->id);
    if (unvme_model != UNVME_MODEL_CS && ses == ses->next) unvme_cleanup();
    return 0;
}

/**
 * Get a free page from the data pool in an IO queue.
 * @param   ioq         io queue
 * @return  page index or -1 if no free page is available.
 */
int unvme_do_alloc(unvme_queue_t* ioq)
{
    unvme_datapool_t* datapool = &ioq->datapool;
    int id = datapool->nextsi;
    do {
        if (datapool->piostat[id].ustat == UNVME_PS_FREE) {
            datapool->piostat[id].ustat = UNVME_PS_READY;
            datapool->nextsi = id;
            if (++datapool->nextsi == ioq->ses->ns.maxppq) datapool->nextsi = 0;
            return id;
        }
        if (++id == ioq->ses->ns.maxppq) id = 0;
    } while (id != datapool->nextsi);
    ERROR("q=%d out of pages", ioq->nvq->id);
    return -1;
}

/**
 * Put a free page back into the data pool of an IO queue.
 * @param   ioq         io queue
 * @param   id          page id
 * @return  0 if page can be freed else -1.
 */
int unvme_do_free(unvme_queue_t* ioq, int id)
{
    if (ioq->datapool.piostat[id].ustat == UNVME_PS_FREE) return -1;
    ioq->datapool.piostat[id].ustat = UNVME_PS_FREE;
    return 0;
}

/**
 * Process read write command.
 * @param   ioq         io queue
 * @param   pa          page array
 * @param   opc         op code
 * @return  0 if ok else -1.
 */
int unvme_do_rw(unvme_queue_t* ioq, unvme_page_t* pa, int opc)
{
    unvme_session_t* ses = ioq->ses;
    unvme_datapool_t* datapool = &ioq->datapool;
    int cid = pa->id;

    if (datapool->piostat[cid].ustat != UNVME_PS_READY) {
        ERROR("page %d ustat=%d", cid, datapool->piostat[cid].ustat);
        return -1;
    }
    datapool->piostat[cid].ustat = UNVME_PS_PENDING;

    int pagesize = ses->ns.pagesize;
    int nbpp = ses->ns.nbpp;
    int numpages = (pa->nlb + nbpp - 1) / nbpp;
    int slot = cid * pagesize;
    u64 prp1 = datapool->data->addr + slot + pa->offset;
    u64 prp2 = 0;

    if (numpages >= 2) {
        if (numpages == 2) {
            prp2 = datapool->data->addr + pa[1].id * pagesize;
        } else {
            u64* prplist = datapool->prplist->buf + slot;
            int i;
            for (i = 1; i < numpages; i++) {
                *prplist++ = datapool->data->addr + pa[i].id * pagesize;
            }
            prp2 = datapool->prplist->addr + slot;
        }
    }

    int err = nvme_cmd_rw(opc, ioq->nvq, ses->ns.id,
                          cid, pa->slba, pa->nlb, prp1, prp2);

    if (unvme_model != UNVME_MODEL_APC && !err) err = sem_post(&ses->tpc.sem);

    return err;
}

