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
 * @file unvme.h
 * @brief UNVMe header file.
 */

#ifndef _UNVME_H
#define _UNVME_H

#include <sys/types.h>
#include <semaphore.h>

#include "rdtsc.h"
#include "unvme_log.h"
#include "unvme_vfio.h"
#include "unvme_shm.h"
#include "unvme_nvme.h"
#include "libunvme.h"


/// @cond

#if __GNUC__ < 4 || (__GNUC__ == 4 && (__GNUC_MINOR__ < 7))
    #define atomic_add(v,n)   __sync_add_and_fetch(v,n)
    #define atomic_sub(v,n)   __sync_sub_and_fetch(v,n)
#else
    #define atomic_add(v,n)   __atomic_add_fetch(v,n,__ATOMIC_SEQ_CST)
    #define atomic_sub(v,n)   __atomic_sub_fetch(v,n,__ATOMIC_SEQ_CST)
#endif

#define FATAL(fmt, arg...)  \
            do { ERROR(fmt, ##arg); unvme_cleanup(); exit(1); } while (0)

#define UNVME_PA_SIZE(sz, pgsz) (((sz)+(pgsz)-1) & ~((pgsz)-1))

/// @endcond


/// driver models
typedef enum {
    UNVME_MODEL_APC     = 0,            ///< application process completion
    UNVME_MODEL_TPC     = 1,            ///< thread process completion
    UNVME_MODEL_CS      = 2,            ///< client-server model
} unvme_model_t;

/// page tracking status code
typedef enum {
    UNVME_PS_FREE       = 0,            ///< page is free
    UNVME_PS_READY      = 1,            ///< page is idle
    UNVME_PS_PENDING    = 2,            ///< page is I/O pending
} unvme_ustat_t;

/// client server interface command (MODEL_CS)
typedef enum {
    UNVME_CMD_NULL      = 0,            ///< no command
    UNVME_CMD_WRITE     = 1,            ///< must be same as NVME_CMD_WRITE
    UNVME_CMD_READ      = 2,            ///< must be same as NVME_CMD_READ
    UNVME_CMD_OPEN      = 3,            ///< open
    UNVME_CMD_CLOSE     = 4,            ///< close
    UNVME_CMD_ALLOC     = 5,            ///< allocate a page
    UNVME_CMD_FREE      = 6,            ///< free a page
} unvme_cscmd_t;


/// client server interface message (MODEL_CS)
typedef struct _unvme_msg {
    unvme_cscmd_t           cmd;        ///< client server command
    unvme_cscmd_t           ack;        ///< command ack
    int                     stat;       ///< returned status
    union {
        // open-close message
        struct {
            pid_t           cpid;       ///< client pid
            int             nsid;       ///< nsid
            int             qcount;     ///< number of I/O queues
            int             qsize;      ///< I/O queue size
            int             sid;        ///< session id (starting queue id)
            unvme_ns_t      ns;         ///< returned namespace attributes
        };
        // alloc-free message
        int                 pgid;       ///< returned page id
        // read-write message
        unvme_page_t        pa[0];      ///< page array
    };
} unvme_msg_t;

/// client server interface structure (MODEL_CS)
typedef struct _unvme_csif {
    pthread_t               thread;     ///< thread array
    int                     stop;       ///< thread stop flag
    shm_file_t*             sf;         ///< shared memory file
    pthread_spinlock_t*     lock;       ///< client queue lock
    sem_t*                  sem;        ///< shared request semaphore
    int                     msglen;     ///< message length
    void*                   msgbuf;     ///< shared message buffer array
    int                     mqsize;     ///< message queue size
    int*                    mqcount;    ///< shared message queue count
    int*                    mqhead;     ///< shared message queue head
    int*                    mqtail;     ///< shared message queue tail
    int*                    mq;         ///< shared message queue
} unvme_csif_t;

/// thread process completion structure
typedef struct _unvme_tpc {
    pthread_t               thread;     ///< processing thread
    int                     stop;       ///< thread stop flag
    sem_t                   sem;        ///< submitted semaphore count
} unvme_tpc_t;

/// page I/O completion queue
typedef struct _unvme_piocpq {
    int                     size;       ///< queue size
    int                     count;      ///< completion count
    int                     head;       ///< completion queue head
    int                     tail;       ///< completion queue tail
    unvme_page_t*           pa[0];      ///< array of completion page pointers
} unvme_piocpq_t;

/// page I/O status tracking
typedef struct _unvme_piostat {
    unvme_page_t*           cpa;        ///< client page array reference
    unvme_ustat_t           ustat;      ///< page usage status
    int                     cstat;      ///< page completion status
} unvme_piostat_t;

/// data pool in a queue
typedef struct _unvme_datapool {
    shm_file_t*             sf;         ///< shared memory file (MODEL_CS)
    vfio_dma_t*             data;       ///< dma memory for page data
    vfio_dma_t*             prplist;    ///< dma memory for PRP list
    unvme_piostat_t*        piostat;    ///< page I/O status
    unvme_piocpq_t*         piocpq;     ///< page I/O completion queue
    int                     nextsi;     ///< next search index for free page
} unvme_datapool_t;

/// client page array allocation
typedef struct _unvme_pal {
    unvme_page_t*           pa;         ///< page array
    int                     count;      ///< number of pages
    struct _unvme_pal*      prev;       ///< previous allocated node
    struct _unvme_pal*      next;       ///< next allocated node
} unvme_pal_t;

struct _unvme_session;
struct _unvme_device;

/// queue context
typedef struct _unvme_queue {
    struct _unvme_session*  ses;        ///< session reference
    nvme_queue_t*           nvq;        ///< NVMe associated queue
    vfio_dma_t*             sqdma;      ///< submission queue allocation
    vfio_dma_t*             cqdma;      ///< completion queue allocation
    unvme_datapool_t        datapool;   ///< queue associated data pool
    int                     pac;        ///< client page allocation count
    unvme_pal_t*            pal;        ///< client page allocation list
} unvme_queue_t;

/// open session
typedef struct _unvme_session {
    struct _unvme_device*   dev;        ///< device reference
    unvme_ns_t              ns;         ///< namespace info
    pid_t                   cpid;       ///< client process id
    int                     id;         ///< session id (same as queues[0] id)
    int                     qcount;     ///< number of queues
    int                     qsize;      ///< queue size
    unvme_queue_t*          queues;     ///< array of queues
    unvme_tpc_t             tpc;        ///< thread process completion
    unvme_csif_t            csif;       ///< client server interface (MODEL_CS)
    struct _unvme_session*  prev;       ///< previous session node
    struct _unvme_session*  next;       ///< next session node
} unvme_session_t;

/// device context
typedef struct _unvme_device {
    vfio_device_t*          vfiodev;    ///< vfio device
    nvme_device_t*          nvmedev;    ///< nvme device
    unvme_session_t*        ses;        ///< session list
    int                     numioqs;    ///< total number of I/O queues
    pthread_spinlock_t      lock;       ///< device lock
} unvme_device_t;

/// client data structure
typedef struct _unvme_client {
    pthread_mutex_t         lock;       ///< client lock
    unvme_csif_t            csif;       ///< admin client server interface
    unvme_session_t*        ses;        ///< open session list
} unvme_client_t;


// Forward declarations
unvme_device_t* unvme_init(int count);
void unvme_cleanup(void);
void unvme_dev_init(unvme_device_t* dev, int vfid);
void unvme_dev_cleanup(unvme_device_t* dev);

void unvme_datapool_alloc(unvme_queue_t* ioq);
void unvme_datapool_free(unvme_queue_t* ioq);
void unvme_session_create_ext(unvme_session_t* ses);
void unvme_session_delete_ext(unvme_session_t* ses);

void* unvme_tpc_thread(void* arg);
void unvme_tpc_create(unvme_session_t* ses);
void unvme_tpc_delete(unvme_session_t* ses);

unvme_session_t* unvme_do_open(unvme_device_t* dev, int vfid, pid_t cpid,
                               int nsid, int qcount, int qsize);
int unvme_do_close(unvme_device_t* dev, pid_t cpid, int sid);
int unvme_do_alloc(unvme_queue_t* ioq);
int unvme_do_free(unvme_queue_t* ioq, int id);
int unvme_do_rw(unvme_queue_t* ioq, unvme_page_t* pa, int opc);

unvme_session_t* client_open(int vfid, int nsid, int qcount, int qsize);
int client_close(const unvme_ns_t* ns);
int client_alloc(const unvme_ns_t* ns, unvme_pal_t* pal);
int client_free(const unvme_ns_t* ns, unvme_pal_t* pal);
int client_rw(const unvme_ns_t* ns, unvme_page_t* pa, int opc);

#endif  // _UNVME_H
