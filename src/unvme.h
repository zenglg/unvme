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
enum {
    UNVME_PS_FREE       = 0,            ///< page is free
    UNVME_PS_READY      = 1,            ///< page is idle
    UNVME_PS_PENDING    = 2,            ///< page is io pending
};

// client server interface command code (MODEL_CS)
enum {
    UNVME_CMD_NULL      = 0,            ///< no command
    UNVME_CMD_WRITE     = NVME_CMD_WRITE, ///< must be same as NVME_CMD_WRITE
    UNVME_CMD_READ      = NVME_CMD_READ,  ///< must be same as NVME_CMD_READ
    UNVME_CMD_OPEN,                     ///< open
    UNVME_CMD_CLOSE,                    ///< close
    UNVME_CMD_ALLOC,                    ///< allocate a page
    UNVME_CMD_FREE,                     ///< free a page
    UNVME_CMD_QUIT,                     ///< quit command
};


/// client server interface message (MODEL_CS)
typedef struct _unvme_msg {
    u32                     id;         ///< message id
    int                     cmd;        ///< command code
    int                     stat;       ///< returned status
    union {
        // open-close command-response parameters
        struct {
            pid_t           cpid;       ///< client pid
            int             nsid;       ///< nsid
            int             qcount;     ///< number of io queues
            int             qsize;      ///< io queue size
            int             nvqid;      ///< starting NVMe qid
            unvme_ns_t      ns;         ///< returned namespace attributes
        };
        // read-write command parameters
        unvme_page_t    pa[0];      ///< page array
        // alloc-free command parameters
        int                 pgid;       ///< page id
    };
} unvme_msg_t;

/// client server interface structure (MODEL_CS)
typedef struct _unvme_csif {
    pthread_t               thread;     ///< processing thread
    shm_file_t*             sf;         ///< shared memory file
    sem_t*                  req;        ///< shared request semaphore
    sem_t*                  ack;        ///< shared ack semaphore
    unvme_msg_t*            msg;        ///< shared message
} unvme_csif_t;

/// thread process completion structure
typedef struct _unvme_tpc {
    pthread_t               thread;     ///< processing thread
    int                     stop;       ///< thread stop flag
    sem_t                   sem;        ///< submitted semaphore count
} unvme_tpc_t;

/// page io completion queue
typedef struct _unvme_piocpq {
    int                     size;       ///< queue size
    int                     count;      ///< completion count
    int                     head;       ///< completion queue head
    int                     tail;       ///< completion queue tail
    unvme_page_t*           pa[0];      ///< array of completion page pointers
} unvme_piocpq_t;

/// page io status tracking
typedef struct _unvme_piostat {
    unvme_page_t*           cpa;        ///< client page array reference
    int                     ustat;      ///< page usage status
    int                     cstat;      ///< page completion status
} unvme_piostat_t;

/// data pool in a queue
typedef struct _unvme_datapool {
    shm_file_t*             sf;         ///< shared memory file (MODEL_CS)
    vfio_dma_t*             data;       ///< dma memory for page data
    vfio_dma_t*             prplist;    ///< dma memory for PRP list
    unvme_piostat_t*        piostat;    ///< page io status
    unvme_piocpq_t*         piocpq;     ///< page io completion queue
    int                     nextsi;     ///< next search index for free page
} unvme_datapool_t;

struct _unvme_device;

/// queue context
typedef struct _unvme_queue {
    struct _unvme_device*   dev;        ///< device reference
    nvme_queue_t*           nvq;        ///< NVMe queue
    unvme_ns_t*             ns;         ///< namespace reference
    int                     gid;        ///< queue group id
    pid_t                   cpid;       ///< client process id
    vfio_dma_t*             sqdma;      ///< submission queue allocation
    vfio_dma_t*             cqdma;      ///< completion queue allocation
    unvme_datapool_t        datapool;   ///< data pool
    unvme_csif_t            csif;       ///< client server interface (MODEL_CS)
    unvme_tpc_t             tpc;        ///< thread process completion
    struct _unvme_queue*    prev;       ///< previous queue link
    struct _unvme_queue*    next;       ///< next queue link
} unvme_queue_t;

/// device context
typedef struct _unvme_device {
    vfio_device_t*          vfiodev;    ///< vfio device
    nvme_device_t*          nvmedev;    ///< nvme device
    unvme_ns_t*             ns;         ///< namespace list
    unvme_queue_t           adminq;     ///< admin queue
    int                     numioqs;    ///< number of io queues
    int                     maxiolen;   ///< max io size
    unvme_queue_t*          ioqlist;    ///< io queue list
    pthread_spinlock_t      lock;       ///< device access lock
} unvme_device_t;

/// client page array allocation entry
typedef struct _client_pal {
    unvme_page_t*           pa;         ///< page array
    int                     count;      ///< number of pages
    struct _client_pal*     prev;       ///< previous allocated node
    struct _client_pal*     next;       ///< next allocated node
} client_pal_t;

struct _client_session;

/// client queue
typedef struct _client_queue {
    struct _client_session* ses;        ///< queue session owner
    int                     id;         ///< client queue index
    int                     pac;        ///< page allocation count
    client_pal_t*           pal;        ///< page array allocation list
    unvme_queue_t*          ioq;        ///< NVMe io queue
    int                     nvqid;      ///< NVMe queue id (MODEL_CS)
    unvme_datapool_t        datapool;   ///< data pool (MODEL_CS)
    unvme_csif_t            csif;       ///< client server interface (MODEL_CS)
} client_queue_t;

/// client session
typedef struct _client_session {
    struct _client_session* prev;       ///< previous session
    struct _client_session* next;       ///< next session
    unvme_ns_t              ns;         ///< namespace info
    int                     qsize;      ///< queue size
    int                     qcount;     ///< number of queues
    client_queue_t*         queues;     ///< array of client queue entries
} client_session_t;

/// main client structure
typedef struct _client_data {
    client_session_t*       ses;        ///< client open session list
    pid_t                   cpid;       ///< client pid
    unvme_csif_t            csifadm;    ///< admin interface (MODEL_CS)
} client_main_t;


// Forward declarations
unvme_device_t* unvme_init(int count);
void unvme_cleanup(void);
void unvme_dev_init(unvme_device_t* dev, int vfid);
void unvme_dev_cleanup(unvme_device_t* dev);
void unvme_datapool_alloc(unvme_queue_t* ioq);
void unvme_datapool_free(unvme_queue_t* ioq);
void unvme_adminq_create_ext(unvme_queue_t* adminq);
void unvme_adminq_delete_ext(unvme_queue_t* adminq);
void unvme_ioq_create_ext(unvme_queue_t* ioq);
void unvme_ioq_delete_ext(unvme_queue_t* ioq);
void* unvme_tpc_thread(void* arg);
void unvme_tpc_create(unvme_queue_t* ioq);
void unvme_tpc_delete(unvme_queue_t* ioq);
unvme_page_t* unvme_tpc_poll(unvme_datapool_t* datapool, unvme_page_t* pa, int sec);
unvme_page_t* unvme_tpc_apoll(unvme_datapool_t* datapool, int sec);
unvme_queue_t* unvme_do_open(unvme_device_t* dev, int vfid, pid_t cpid, int nsid, int qcount, int qsize);
int unvme_do_close(unvme_device_t* dev, pid_t cpid, int gid);
int unvme_do_alloc(unvme_queue_t* ioq);
int unvme_do_free(unvme_queue_t* ioq, int id);
int unvme_do_rw(unvme_queue_t* ioq, unvme_page_t* pa, int opc);
int client_open(client_session_t* ses, int vfid, int nsid);
int client_close(client_session_t* ses);
int client_alloc(client_queue_t* q, client_pal_t* pal);
int client_free(client_queue_t* q, client_pal_t* pal);
int client_rw(const unvme_ns_t* ns, unvme_page_t* pa, int opc);

#endif  // _UNVME_H

