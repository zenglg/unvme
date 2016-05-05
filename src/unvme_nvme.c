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
 * @brief NVMe implementation.
 */

#include <sys/mman.h>
#include <string.h>
#include <errno.h>

#include "rdtsc.h"
#include "unvme_log.h"
#include "unvme_nvme.h"


/// @cond

static inline u32 r32(nvme_device_t* dev, u32* addr)
{
    u32 val = *addr;
    DEBUG("r32 %#lx %#x", (u64) addr - (u64) dev->reg, val);
    return val;
}

static inline void w32(nvme_device_t* dev, u32* addr, u32 val)
{
    DEBUG("w32 %#lx %#x", (u64) addr - (u64) dev->reg, val);
    *addr = val;
}

static inline u64 r64(nvme_device_t* dev, u64* addr)
{
    u64 val = *addr;
    DEBUG("r64 %#lx %#lx", (u64) addr - (u64) dev->reg, val);
    return val;
}

static inline void w64(nvme_device_t* dev, u64* addr, u64 val)
{
    DEBUG("w64 %#lx %#lx", (u64) addr - (u64) dev->reg, val);
    *addr = val;
}

/// @endcond


/**
 * Create an NVMe device context and map the controller register.
 * @param   mapfd       file descriptor for register mapping
 * @return  device context or NULL if failure.
 */
nvme_device_t* nvme_create(int mapfd)
{
    nvme_device_t* dev = zalloc(sizeof(*dev));

    dev->reg = mmap(0, sizeof(nvme_controller_reg_t),
                    PROT_READ|PROT_WRITE, MAP_SHARED, mapfd, 0);
    if (dev->reg == MAP_FAILED) {
        ERROR("mmap errno %d", errno);
        return NULL;
    }
    return dev;
}

/**
 * Delete an NVMe device context
 * @param   dev         device context
 */
void nvme_delete(nvme_device_t* dev)
{
    if (munmap(dev->reg, sizeof(nvme_controller_reg_t))) {
        ERROR("munmap errno %d", errno);
    }
    free(dev);
}

/**
 * Wait for controller enabled/disabled state.
 * @param   dev         device context
 * @param   ready       ready state (enabled/disabled)
 * @return  0 if ok else -1.
 */
static int nvme_ctlr_wait_ready(nvme_device_t* dev, int ready)
{
    nvme_controller_cap_t cap;
    cap.val = r64(dev, &dev->reg->cap.val);
    int timeout = cap.to; // in 500ms unit

    int i;
    for (i = 0; i < timeout; i++) {
        usleep(500000);
        nvme_controller_status_t csts;
        csts.val = r32(dev, &dev->reg->csts.val);
        if (csts.rdy == ready) return 0;
    }

    ERROR("timeout waiting for ready %d", ready);
    return -1;
}

/**
 * Disable controller.
 * @param   dev         device context
 * @return  0 if ok else -1.
 */
static int nvme_ctlr_disable(nvme_device_t* dev)
{
    DEBUG_FN();
    nvme_controller_config_t cc;
    cc.val = r32(dev, &dev->reg->cc.val);
    cc.en = 0;
    w32(dev, &dev->reg->cc.val, cc.val);
    return nvme_ctlr_wait_ready(dev, 0);
}

/**
 * Enable controller.
 * @param   dev         device context
 * @param   cc          controller configuration settings
 * @return  0 if ok else -1.
 */
static int nvme_ctlr_enable(nvme_device_t* dev, nvme_controller_config_t cc)
{
    DEBUG_FN();
    cc.en = 1;
    w32(dev, &dev->reg->cc.val, cc.val);
    return nvme_ctlr_wait_ready(dev, 1);
}

/**
 * Submit an entry at submission queue tail.
 * @param   q           queue
 */
static void nvme_submit_cmd(nvme_queue_t* q)
{
    //HEX_DUMP(&q->sq[q->sq_tail], sizeof(nvme_sq_entry_t));
    if (++q->sq_tail == q->size) q->sq_tail = 0;
    w32(q->dev, q->sq_doorbell, q->sq_tail);
}

/**
 * Check a completion queue and return the completed command id and status.
 * @param   q           queue
 * @param   stat        completion status reference
 * @return  the completed command id or -1 if there's no completion.
 */
int nvme_check_completion(nvme_queue_t* q, int* stat)
{
    nvme_cq_entry_t* cqe = &q->cq[q->cq_head];
    if (cqe->p == q->cq_phase) return -1;

    *stat = cqe->psf & 0xfe;
    if (++q->cq_head == q->size) {
        q->cq_head = 0;
        q->cq_phase = !q->cq_phase;
    }
    w32(q->dev, q->cq_doorbell, q->cq_head);

    if (*stat == 0) {
        DEBUG_FN("q=%d cid=%#x (C)", q->id, cqe->cid);
    } else {
        ERROR("q=%d cid=%#x stat=%#x (dnr=%d m=%d sct=%d sc=%#x) (C)",
              q->id, cqe->cid, *stat, cqe->dnr, cqe->m, cqe->sct, cqe->sc);
    }
    return cqe->cid;
}

/**
 * Wait for a given command completion until timeout.
 * @param   q           queue
 * @param   cid         cid
 * @param   timeout in seconds
 * @return  completion status (0 if ok).
 */
int nvme_wait_completion(nvme_queue_t* q, int cid, int timeout)
{
    u64 endtsc = 0;

    do {
        int stat;
        int ret = nvme_check_completion(q, &stat);
        if (ret >= 0) {
            if (ret == cid && stat == 0) return 0;
            if (ret != cid) {
                ERROR("cid wait=%#x recv=%#xd", cid, ret);
                stat = -1;
            } else ERROR("status %#x", stat);
            return stat;
        } else if (endtsc == 0) {
            endtsc = rdtsc() + timeout * rdtsc_second();
        }
    } while (rdtsc() < endtsc);

    ERROR("timeout");
    return -1;
}

/**
 * NVMe identify command.
 * Submit the command and wait for completion.
 * @param   dev         device context
 * @param   nsid        namespace id (< 0 implies cns)
 * @param   prp1        PRP1 address
 * @param   prp2        PRP2 address
 * @return  completion status (0 if ok).
 */
int nvme_acmd_identify(nvme_device_t* dev, int nsid, u64 prp1, u64 prp2)
{
    nvme_queue_t* adminq = &dev->adminq;
    int cid = adminq->sq_tail;
    nvme_acmd_identify_t* cmd = &adminq->sq[cid].identify;

    memset(cmd, 0, sizeof (*cmd));
    cmd->common.opc = NVME_ACMD_IDENTIFY;
    cmd->common.cid = cid;
    cmd->common.nsid = nsid;
    cmd->common.prp1 = prp1;
    cmd->common.prp2 = prp2;
    cmd->cns = nsid == 0 ? 1 : 0;

    DEBUG_FN("cid=%#x nsid=%d", cid, nsid);
    nvme_submit_cmd(adminq);
    return nvme_wait_completion(adminq, cid, 30);
}

/**
 * NVMe get log page command.
 * Submit the command and wait for completion.
 * @param   dev         device context
 * @param   nsid        namespace id
 * @param   numd        number of dwords
 * @param   lid         log page id
 * @param   prp1        PRP1 address
 * @param   prp2        PRP2 address
 * @return  completion status (0 if ok).
 */
int nvme_acmd_get_log_page(nvme_device_t* dev, int nsid,
                           int lid, int numd, u64 prp1, u64 prp2)
{
    nvme_queue_t* adminq = &dev->adminq;
    int cid = adminq->sq_tail;
    nvme_acmd_get_log_page_t* cmd = &adminq->sq[cid].get_log_page;

    memset(cmd, 0, sizeof (*cmd));
    cmd->common.opc = NVME_ACMD_GET_LOG_PAGE;
    cmd->common.cid = cid;
    cmd->common.nsid = nsid;
    cmd->common.prp1 = prp1;
    cmd->common.prp2 = prp2;
    cmd->lid = lid;
    cmd->numd = numd;

    DEBUG_FN("cid=%#x lid=%d", cid, lid);
    nvme_submit_cmd(adminq);
    return nvme_wait_completion(adminq, cid, 30);
}

/**
 * NVMe create I/O completion queue command.
 * Submit the command and wait for completion.
 * @param   ioq         io queue
 * @param   prp         PRP1 address
 * @param   ien         interrups enabled
 * @return  0 if ok, else -1.
 */
int nvme_acmd_create_cq(nvme_queue_t* ioq, u64 prp, int ien)
{
    nvme_queue_t* adminq = &ioq->dev->adminq;
    int cid = adminq->sq_tail;
    nvme_acmd_create_cq_t* cmd = &adminq->sq[cid].create_cq;

    memset(cmd, 0, sizeof (*cmd));
    cmd->common.opc = NVME_ACMD_CREATE_CQ;
    cmd->common.cid = cid;
    cmd->common.prp1 = prp;
    cmd->pc = 1;
    cmd->qid = ioq->id;
    cmd->qsize = ioq->size - 1;
    cmd->ien = ien;
    cmd->iv = ioq->id;

    DEBUG_FN("q=%d cid=%#x qs=%d ien=%d", ioq->id, cid, ioq->size, ien);
    nvme_submit_cmd(adminq);
    return nvme_wait_completion(adminq, cid, 30);
}

/**
 * NVMe create I/O submission queue command.
 * Submit the command and wait for completion.
 * @param   ioq         io queue
 * @param   prp         PRP1 address
 * @return  0 if ok, else -1.
 */
int nvme_acmd_create_sq(nvme_queue_t* ioq, u64 prp)
{
    nvme_queue_t* adminq = &ioq->dev->adminq;
    int cid = adminq->sq_tail;
    nvme_acmd_create_sq_t* cmd = &adminq->sq[cid].create_sq;

    memset(cmd, 0, sizeof (*cmd));
    cmd->common.opc = NVME_ACMD_CREATE_SQ;
    cmd->common.cid = cid;
    cmd->common.prp1 = prp;
    cmd->pc = 1;
    cmd->qprio = 2; // 0=urgent 1=high 2=medium 3=low
    cmd->qid = ioq->id;
    cmd->cqid = ioq->id;
    cmd->qsize = ioq->size - 1;

    DEBUG_FN("q=%d cid=%#x qs=%d", ioq->id, cid, ioq->size);
    nvme_submit_cmd(adminq);
    return nvme_wait_completion(adminq, cid, 30);
}

/**
 * NVMe delete I/O queue command.
 * Submit the command and wait for completion.
 * @param   ioq         io queue
 * @param   opc         op code
 * @return  0 if ok else error code.
 */
static int nvme_acmd_delete_ioq(nvme_queue_t* ioq, int opc)
{
    nvme_queue_t* adminq = &ioq->dev->adminq;
    int cid = adminq->sq_tail;
    nvme_acmd_delete_ioq_t* cmd = &adminq->sq[cid].delete_ioq;

    memset(cmd, 0, sizeof (*cmd));
    cmd->common.opc = opc;
    cmd->common.cid = cid;
    cmd->qid = ioq->id;

    DEBUG_FN("%cq=%d cid=%#x",
             opc == NVME_ACMD_DELETE_CQ ? 'c' : 's', ioq->id, cid);
    nvme_submit_cmd(adminq);
    return nvme_wait_completion(adminq, cid, 30);
}

/**
 * NVMe delete I/O submission queue command.
 * Submit the command and wait for completion.
 * @param   ioq         io queue
 * @return  0 if ok else error code.
 */
int nvme_acmd_delete_sq(nvme_queue_t* ioq)
{
    return nvme_acmd_delete_ioq(ioq, NVME_ACMD_DELETE_SQ);
}

/**
 * NVMe delete I/O completion queue command.
 * Submit the command and wait for completion.
 * @param   ioq         io queue
 * @return  0 if ok else error code.
 */
int nvme_acmd_delete_cq(nvme_queue_t* ioq)
{
    return nvme_acmd_delete_ioq(ioq, NVME_ACMD_DELETE_CQ);
}

/**
 * NVMe submit a read write command.
 * @param   opc         op code
 * @param   ioq         io queue
 * @param   nsid        namespace
 * @param   cid         command id
 * @param   lba         startling logical block address
 * @param   nb          number of blocks
 * @param   prp1        PRP1 address
 * @param   prp2        PRP2 address
 * @return  0 if ok else -1.
 */
inline int nvme_cmd_rw(int opc, nvme_queue_t* ioq, int nsid,
                       int cid, u64 lba, int nb, u64 prp1, u64 prp2)
{
    nvme_command_rw_t* cmd = &ioq->sq[ioq->sq_tail].rw;

    memset(cmd, 0, sizeof (*cmd));
    cmd->common.opc = opc;
    cmd->common.cid = cid;
    cmd->common.nsid = nsid;
    cmd->common.prp1 = prp1;
    cmd->common.prp2 = prp2;
    cmd->slba = lba;
    cmd->nlb = nb - 1;
    DEBUG_FN("q=%d sqt=%d cid=%#x nsid=%d lba=%#lx nb=%d (%c)", ioq->id,
             ioq->sq_tail, cid, nsid, lba, nb, opc == NVME_CMD_READ? 'R' : 'W');
    nvme_submit_cmd(ioq);
    return 0;
}

/**
 * NVMe submit a read command.
 * @param   ioq         io queue
 * @param   nsid        namespace
 * @param   cid         command id
 * @param   lba         startling logical block address
 * @param   nb          number of blocks
 * @param   prp1        PRP1 address
 * @param   prp2        PRP2 address
 * @return  0 if ok else -1.
 */
int nvme_cmd_read(nvme_queue_t* ioq, int nsid,
                  int cid, u64 lba, int nb, u64 prp1, u64 prp2)
{
    return nvme_cmd_rw(NVME_CMD_READ, ioq, nsid, cid, lba, nb, prp1, prp2);
}

/**
 * NVMe submit a write command.
 * @param   ioq         io queue
 * @param   nsid        namespace
 * @param   cid         command id
 * @param   lba         startling logical block address
 * @param   nb          number of blocks
 * @param   prp1        PRP1 address
 * @param   prp2        PRP2 address
 * @return  0 if ok else -1.
 */
int nvme_cmd_write(nvme_queue_t* ioq, int nsid,
               int cid, u64 lba, int nb, u64 prp1, u64 prp2)
{
    return nvme_cmd_rw(NVME_CMD_WRITE, ioq, nsid, cid, lba, nb, prp1, prp2);
}

/**
 * Create an IO queue pair of completion and submission.
 * @param   dev         device context
 * @param   id          queue id
 * @param   qsize       queue size
 * @param   sqbuf       submission queue buffer
 * @param   sqpa        submission queue IO physical address
 * @param   cqbuf       completion queue buffer
 * @param   cqpa        admin completion IO physical address
 * @param   ien         interrupts enabled
 * @return  pointer to the created io queue or NULL if failure.
 */
nvme_queue_t* nvme_create_ioq(nvme_device_t* dev, int id, int qsize,
                          void* sqbuf, u64 sqpa, void* cqbuf, u64 cqpa, int ien)
{
    nvme_queue_t* ioq = zalloc(sizeof(*ioq));
    ioq->dev = dev;
    ioq->id = id;
    ioq->size = qsize;
    ioq->sq = sqbuf;
    ioq->cq = cqbuf;
    ioq->sq_doorbell = dev->reg->sq0tdbl + (2 * id * dev->dbstride);
    ioq->cq_doorbell = ioq->sq_doorbell + dev->dbstride;

    if (nvme_acmd_create_cq(ioq, cqpa, ien) | nvme_acmd_create_sq(ioq, sqpa)) {
        free(ioq);
        return NULL;
    }
    return ioq;
}

/**
 * Create an IO queue pair of submission and completion.
 * @param   ioq         io queue to setup
 * @return  0 if ok else -1.
 */
int nvme_delete_ioq(nvme_queue_t* ioq)
{
    if (nvme_acmd_delete_sq(ioq) | nvme_acmd_delete_cq(ioq)) return -1;
    free(ioq);
    return 0;
}

/**
 * NVMe setup admin queue.
 * @param   dev         device context
 * @param   qsize       queue size
 * @param   sqbuf       submission queue buffer
 * @param   sqpa        submission queue IO physical address
 * @param   cqbuf       completion queue buffer
 * @param   cqpa        admin completion physical address
 * @return  pointer to the admin queue or NULL if failure.
 */
nvme_queue_t* nvme_setup_adminq(nvme_device_t* dev, int qsize,
                                void* sqbuf, u64 sqpa, void* cqbuf, u64 cqpa)
{
    if (nvme_ctlr_disable(dev)) return NULL;

    nvme_controller_cap_t cap;
    cap.val = r64(dev, &dev->reg->cap.val);
    dev->dbstride = 1 << cap.dstrd; // in u32 size offset
    dev->maxqsize = cap.mqes + 1;
    dev->pageshift = PAGESHIFT;

    nvme_queue_t* adminq = &dev->adminq;
    adminq->dev = dev;
    adminq->id = 0;
    adminq->size = qsize;
    adminq->sq = sqbuf;
    adminq->cq = cqbuf;
    adminq->sq_doorbell = dev->reg->sq0tdbl;
    adminq->cq_doorbell = adminq->sq_doorbell + dev->dbstride;

    nvme_adminq_attr_t aqa;
    aqa.val = 0;
    aqa.asqs = aqa.acqs = qsize - 1;
    w32(dev, &dev->reg->aqa.val, aqa.val);
    w64(dev, &dev->reg->asq, sqpa);
    w64(dev, &dev->reg->acq, cqpa);

    nvme_controller_config_t cc;
    cc.val = 0;
    cc.shn = 0;
    cc.ams = 0;
    cc.css = 0;
    cc.iosqes = 6;
    cc.iocqes = 4;
    cc.mps = dev->pageshift - 12;
    if (nvme_ctlr_enable(dev, cc)) return NULL;

    DEBUG_FN("qsize=%d cc=%#x aqa=%#x asq=%#lx acq=%#lx",
             qsize, cc.val, aqa.val, sqpa, cqpa);
    DEBUG_FN("cap=%#lx mps=%d-%d to=%d maxqs=%d dbs=%d", cap.val, cap.mpsmin,
             cap.mpsmax, cap.to, dev->maxqsize, dev->dbstride);
    DEBUG_FN("vs=%#x intms=%#x intmc=%#x csts=%#x", dev->reg->vs.val,
             dev->reg->intms, dev->reg->intmc, dev->reg->csts.val);
    return adminq;
}

