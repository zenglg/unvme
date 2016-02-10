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
 * @param   dev         device context
 * @param   nsid        namespace id
 * @param   qsize       queue size
 * @return  a newly created namespace.
 */
static unvme_ns_t* unvme_ns_create(unvme_device_t* dev, int nsid, int qsize)
{
    unvme_ns_t* ns = zalloc(sizeof(unvme_ns_t));
    strcpy(ns->model, unvme_model_names[unvme_model]);
    ns->maxqsize = dev->nvmedev->maxqsize;
    ns->pagesize = 1 << dev->nvmedev->pageshift;

    vfio_dma_t* dma = vfio_dma_alloc(dev->vfiodev, ns->pagesize << 1);
    if (!dma) FATAL();

    if (nvme_cmd_identify(dev->nvmedev, nsid, dma->addr, dma->addr + ns->pagesize)) FATAL();

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
        int maxiolen = 2;
        for (i = 1; i < idc->mdts; i++) maxiolen *= 2;
        maxiolen *= ns->pagesize;
        dev->maxiolen = idc->mdts == 0 ? 0x20000 : maxiolen;
        ns->maxppio =  dev->maxiolen / ns->pagesize;
    } else {
        memcpy(ns, dev->ns, sizeof(unvme_ns_t));
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
    ns->maxppq =  qsize * ns->maxppio;
    ns->maxiopq = qsize - 1;
    ns->id = nsid;

    if (vfio_dma_free(dma)) FATAL();
    return ns;
}

/**
 * Delete a namespace object.
 * @param   ns          namespace object
 */
static void unvme_ns_delete(unvme_ns_t* ns)
{
    free(ns);
}

/**
 * Create an IO queue.
 * @param   dev         device context
 * @param   cpid        client process id
 * @param   nsid        namespace id
 * @param   qsize       queue size
 * @param   firstq      pointer to first queue in the group
 * @return  the newly created queue pointer.
 */
static unvme_queue_t* unvme_ioq_create(unvme_device_t* dev, int cpid, int nsid,
                                       int qsize, unvme_queue_t* firstq)
{
    DEBUG_FN("%d: qs=%d", dev->vfiodev->id, qsize);

    // allocate queue and add to device ioq list
    unvme_queue_t* ioq = zalloc(sizeof(unvme_queue_t));
    ioq->dev = dev;
    ioq->cpid = cpid;
    int qid;
    if (!dev->ioqlist) {
        qid = 1;
        ioq->prev = ioq;
        ioq->next = ioq;
        dev->ioqlist = ioq;
    } else {
        qid = dev->ioqlist->prev->nvq->id + 1;
        if (qid == 0) FATAL("qid wraps to 0");
        ioq->prev = dev->ioqlist->prev;
        ioq->next = dev->ioqlist;
        dev->ioqlist->prev->next = ioq;
        dev->ioqlist->prev = ioq;
    }

    // create namespace for the first queue and assign to others
    if (!firstq) {
        ioq->ns = unvme_ns_create(dev, nsid, qsize);
        ioq->gid = qid;
    } else {
        ioq->ns = firstq->ns;
        ioq->gid = firstq->nvq->id;
    }

    // allocate io memory and create the submission/completion queue pair
    ioq->sqdma = vfio_dma_alloc(dev->vfiodev, qsize * sizeof(nvme_sq_entry_t));
    ioq->cqdma = vfio_dma_alloc(dev->vfiodev, qsize * sizeof(nvme_cq_entry_t));
    if (!ioq->sqdma || !ioq->cqdma) FATAL();
    ioq->nvq = nvme_create_ioq(dev->nvmedev, qid, qsize,
                               ioq->sqdma->buf, ioq->sqdma->addr,
                               ioq->cqdma->buf, ioq->cqdma->addr);
    if (!ioq->nvq) FATAL();

    unvme_datapool_alloc(ioq);
    unvme_ioq_create_ext(ioq);

    dev->numioqs++;
    INFO_FN("%d: q=%d qs=%d db=%#lx qc=%d",
            dev->vfiodev->id, ioq->nvq->id, ioq->nvq->size,
            (u64)ioq->nvq->sq_doorbell - (u64)dev->nvmedev->reg, dev->numioqs);
    return ioq;
}

/**
 * Delete an IO queue.
 * @param   ioq         io queue
 */
static void unvme_ioq_delete(unvme_queue_t* ioq)
{
    unvme_device_t* dev = ioq->dev;
    DEBUG_FN("q=%d", ioq->nvq->id);

    unvme_ioq_delete_ext(ioq);
    unvme_datapool_free(ioq);

    if (nvme_delete_ioq(ioq->nvq)) FATAL();
    if (vfio_dma_free(ioq->sqdma)) FATAL();
    if (vfio_dma_free(ioq->cqdma)) FATAL();

    // remove from device io queue list
    if (--dev->numioqs) {
        ioq->next->prev = ioq->prev;
        ioq->prev->next = ioq->next;
        if (dev->ioqlist == ioq) dev->ioqlist = ioq->next; 
    } else {
        dev->ioqlist = 0;
    }

    if (ioq->nvq->id == ioq->gid) unvme_ns_delete(ioq->ns);
    free(ioq);
    INFO_FN("q=%d qc=%d", ioq->nvq->id, dev->numioqs);
}

/**
 * Setup admin queue.
 * @param   dev         device context
 * @param   qsize       queue size
 */
static void unvme_adminq_create(unvme_device_t* dev, int qsize)
{
    DEBUG_FN("qs=%d", qsize);
    unvme_queue_t* adminq = &dev->adminq;
    adminq->dev = dev;
    adminq->sqdma = vfio_dma_alloc(dev->vfiodev, qsize * sizeof(nvme_sq_entry_t));
    adminq->cqdma = vfio_dma_alloc(dev->vfiodev, qsize * sizeof(nvme_cq_entry_t));
    if (!adminq->sqdma || !adminq->cqdma) FATAL();
    adminq->nvq = nvme_setup_adminq(dev->nvmedev, qsize,
                                    adminq->sqdma->buf, adminq->sqdma->addr,
                                    adminq->cqdma->buf, adminq->cqdma->addr);
    if (!adminq->nvq) FATAL();

    dev->ns = unvme_ns_create(dev, 0, qsize);
    adminq->ns = dev->ns;
    unvme_adminq_create_ext(adminq);
}

/**
 * Delete admin queue.
 * @param   dev         device context
 */
static void unvme_adminq_delete(unvme_device_t* dev)
{
    DEBUG_FN("%d", dev->vfiodev->id);
    unvme_queue_t* adminq = &dev->adminq;

    unvme_adminq_delete_ext(adminq);
    unvme_ns_delete(dev->ns);
    vfio_dma_free(adminq->sqdma);
    vfio_dma_free(adminq->cqdma);
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

    unvme_adminq_create(dev, 8);
    INFO_FN("%d (%s) ready :)", vfid, dev->ns->mn);
}

/**
 * Cleanup a device.
 * @param   dev         device context
 */
void unvme_dev_cleanup(unvme_device_t* dev)
{
    if (dev && dev->vfiodev) {
        INFO_FN("%d", dev->vfiodev->id);

        pthread_spin_lock(&dev->lock);
        while (dev->numioqs) {
            unvme_ioq_delete(dev->ioqlist);
        }
        dev->ioqlist = 0;
        unvme_adminq_delete(dev);
        nvme_delete(dev->nvmedev);
        vfio_delete(dev->vfiodev);
        pthread_spin_unlock(&dev->lock);
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
    DEBUG_FN();
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
 * Process unvme_open for a given client and create a set of IO queues.
 * @param   dev         device context
 * @param   vfid        vfio id
 * @param   cpid        client process id
 * @param   nsid        namespace id
 * @param   qcount      number of io queues
 * @param   qsize       size of each queue
 * @return  pointer to a list of created queues or NULL if error.
 */
unvme_queue_t* unvme_do_open(unvme_device_t* dev, int vfid, pid_t cpid,
                             int nsid, int qcount, int qsize)
{
    if (!unvme_devlist) unvme_init(1);

    INFO("===\n%s vfio=%d cpid=%d ns=%d qc=%d qs=%d",
         __func__, vfid, cpid, nsid, qcount, qsize);

    if (!dev) dev = unvme_devlist;
    if (!dev->vfiodev) unvme_dev_init(dev, vfid);

    unvme_queue_t* fioq = NULL;
    unvme_queue_t* ioq = NULL;
    int i;
    for (i = 0; i < qcount; i++) {
        ioq = unvme_ioq_create(dev, cpid, nsid, qsize, fioq);
        if (i == 0) fioq = ioq;
    }

    DEBUG_FN("done q=%d-%d bs=%d nb=%lu", fioq->nvq->id, ioq->nvq->id,
                                fioq->ns->blocksize, fioq->ns->blockcount);
    return fioq;
}

/**
 * Process unvme_close for a given client and delete the associated IO queues.
 * @param   dev         device context
 * @param   cpid        client process id
 * @param   gid         io group id
 * @return  0 if ok else -1.
 */
int unvme_do_close(unvme_device_t* dev, pid_t cpid, int gid)
{
    INFO("===\n%s vfio=%d cpid=%d", __func__, dev->vfiodev->id, cpid);
    unvme_queue_t* ioq = dev->ioqlist;
    int i;
    for (i = 0; i < dev->numioqs; ) {
        if (ioq->cpid == cpid && (ioq->gid == gid || gid == 0)) {
            unvme_ioq_delete(ioq);
            ioq = dev->ioqlist;
            i = 0;
        } else {
            ioq = ioq->next;
            i++;
        }
    }

    if (unvme_model != UNVME_MODEL_CS && dev->numioqs == 0) unvme_cleanup();
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
    for (;;) {
        if (datapool->piostat[id].ustat == UNVME_PS_FREE) {
            datapool->piostat[id].ustat = UNVME_PS_READY;
            datapool->nextsi = id;
            if (++datapool->nextsi == ioq->ns->maxppq) datapool->nextsi = 0;
            return id;
        }
        if (++id == ioq->ns->maxppq) id = 0;
        if (id == datapool->nextsi) break;
    }
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
    unvme_datapool_t* datapool = &ioq->datapool;
    int cid = pa->id;
    if (datapool->piostat[cid].ustat != UNVME_PS_READY) {
        ERROR("page %d ustat=%d", cid, datapool->piostat[cid].ustat);
        return -1;
    }
    datapool->piostat[cid].ustat = UNVME_PS_PENDING;

    int pagesize = ioq->ns->pagesize;
    int nbpp = ioq->ns->nbpp;
    int numpages = (pa->nlb + nbpp - 1) / nbpp;
    int slot = cid * pagesize;
    u64 prp1 = datapool->data->addr + slot + pa->offset;
    u64 prp2 = 0;
    int i;

    if (numpages >= 2) {
        if (numpages == 2) {
            prp2 = datapool->data->addr + pa[1].id * pagesize;
        } else {
            u64* prplist = datapool->prplist->buf + slot;
            for (i = 1; i < numpages; i++) {
                *prplist++ = datapool->data->addr + pa[i].id * pagesize;
            }
            prp2 = datapool->prplist->addr + slot;
        }
    }

    i = nvme_cmd_rw(opc, ioq->nvq, ioq->ns->id, cid, pa->slba, pa->nlb, prp1, prp2);
    if (unvme_model != UNVME_MODEL_APC && i == 0) i = sem_post(&ioq->tpc.sem);
    return i;
}

