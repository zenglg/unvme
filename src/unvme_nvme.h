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
 * @brief NVMe header file
 */

#ifndef _UNVME_NVME_H
#define _UNVME_NVME_H

#include <stdint.h>

#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
    #pragma error "only support little endian CPU architecture"
#endif

#define PAGESHIFT           12      ///< system page size shift

#ifndef _UNVME_TYPE
#define _UNVME_TYPE                 ///< bit size data types
typedef uint8_t             u8;     ///< 8-bit unsigned
typedef uint16_t            u16;    ///< 16-bit unsigned
typedef uint32_t            u32;    ///< 32-bit unsigned
typedef uint64_t            u64;    ///< 64-bit unsigned
#endif // _UNVME_TYPE

typedef struct vfio_iommu_type1_dma_map nvme_dma_t; ///< dma map structure

/// NVMe command op code
enum {
    NVME_CMD_FLUSH          = 0x0,      ///< flush
    NVME_CMD_WRITE          = 0x1,      ///< write
    NVME_CMD_READ           = 0x2,      ///< read
    NVME_CMD_WRITE_UNCOR    = 0x4,      ///< write uncorrectable
    NVME_CMD_COMPARE        = 0x5,      ///< compare
    NVME_CMD_DS_MGMT        = 0x9,      ///< dataset management
};

/// NVMe admin command op code
enum {
    NVME_ACMD_DELETE_SQ     = 0x0,      ///< delete io submission queue
    NVME_ACMD_CREATE_SQ     = 0x1,      ///< create io submission queue
    NVME_ACMD_GET_LOG_PAGE  = 0x2,      ///< get log page
    NVME_ACMD_DELETE_CQ     = 0x4,      ///< delete io completion queue
    NVME_ACMD_CREATE_CQ     = 0x5,      ///< create io completion queue
    NVME_ACMD_IDENTIFY      = 0x6,      ///< identify
    NVME_ACMD_ABORT         = 0x8,      ///< abort
    NVME_ACMD_SET_FEATURES  = 0x9,      ///< set features
    NVME_ACMD_GET_FEATURES  = 0xA,      ///< get features
    NVME_ACMD_ASYNC_EVENT   = 0xC,      ///< asynchronous event
    NVME_ACMD_FW_ACTIVATE   = 0x10,     ///< firmware activate
    NVME_ACMD_FW_DOWNLOAD   = 0x11,     ///< firmware image download
};

/// Version
typedef union _nvme_version {
    u32                 val;            ///< whole value
    struct {
        u8              rsvd;           ///< reserved
        u8              mnr;            ///< minor version number
        u16             mjr;            ///< major version number
    };
} nvme_version_t;

/// Admin queue attributes
typedef union _nvme_adminq_attr {
    u32                 val;            ///< whole value
    struct {
        u16             asqs;           ///< admin submission queue size
        u16             acqs;           ///< admin completion queue size
    };
} nvme_adminq_attr_t;

/// Controller capabilities
typedef union _nvme_controller_cap {
    u64                 val;            ///< whole value
    struct {
        u16             mqes;           ///< max queue entries supported
        u8              cqr     : 1;    ///< contiguous queues required
        u8              ams     : 2;    ///< arbitration mechanism supported
        u8              rsvd    : 5;    ///< reserved
        u8              to;             ///< timeout

        u32             dstrd   : 4;    ///< doorbell stride
        u32             nssrs   : 1;    ///< NVM subsystem reset supported
        u32             css     : 8;    ///< command set supported
        u32             rsvd2   : 3;    ///< reserved
        u32             mpsmin  : 4;    ///< memory page size minimum
        u32             mpsmax  : 4;    ///< memory page size maximum
        u32             rsvd3   : 8;    ///< reserved
    };
} nvme_controller_cap_t;

/// Controller configuration register
typedef union _nvme_controller_config {
    u32                 val;            ///< whole value
    struct {
        u32             en      : 1;    ///< enable
        u32             rsvd    : 3;    ///< reserved
        u32             css     : 3;    ///< I/O command set selected
        u32             mps     : 4;    ///< memory page size
        u32             ams     : 3;    ///< arbitration mechanism selected
        u32             shn     : 2;    ///< shutdown notification
        u32             iosqes  : 4;    ///< I/O submission queue entry size
        u32             iocqes  : 4;    ///< I/O completion queue entry size
        u32             rsvd2   : 8;    ///< reserved
    };
} nvme_controller_config_t;

/// Controller status register
typedef union _nvme_controller_status {
    u32                 val;            ///< whole value
    struct {
        u32             rdy     : 1;    ///< ready
        u32             cfs     : 1;    ///< controller fatal status
        u32             shst    : 2;    ///< shutdown status
        u32             rsvd    : 28;   ///< reserved
    };
} nvme_controller_status_t;

/// Controller register (bar 0)
typedef struct _nvme_controller_reg {
    nvme_controller_cap_t   cap;        ///< controller capabilities
    nvme_version_t          vs;         ///< version
    u32                     intms;      ///< interrupt mask set
    u32                     intmc;      ///< interrupt mask clear
    nvme_controller_config_t cc;        ///< controller configuration
    u32                     rsvd;       ///< reserved
    nvme_controller_status_t csts;      ///< controller status
    u32                     nssr;       ///< NVM subsystem reset
    nvme_adminq_attr_t      aqa;        ///< admin queue attributes
    u64                     asq;        ///< admin submission queue base address
    u64                     acq;        ///< admin completion queue base address
    u32                     rcss[1010]; ///< reserved and command set specific
    u32                     sq0tdbl[1024]; ///< sq0 tail doorbell at 0x1000
} nvme_controller_reg_t;

/// Common command header (cdw 0-9)
typedef struct _nvme_command_common {
    u8                      opc;        ///< opcode
    u8                      fuse : 2;   ///< fuse
    u8                      rsvd : 6;   ///< reserved
    u16                     cid;        ///< command id
    u32                     nsid;       ///< namespace id
    u32                     cdw2_3[2];  ///< reserved (cdw 2-3)
    u64                     mptr;       ///< metadata pointer
    u64                     prp1;       ///< PRP entry 1
    u64                     prp2;       ///< PRP entry 2
} nvme_command_common_t;

/// NVMe command:  Read & Write
typedef struct _nvme_command_rw {
    nvme_command_common_t   common;     ///< common cdw 0
    u64                     slba;       ///< starting LBA (cdw 10)
    u16                     nlb;        ///< number of logical blocks
    u16                     rsvd12 : 10; ///< reserved (in cdw 12)
    u16                     prinfo : 4; ///< protection information field
    u16                     fua : 1;    ///< force unit access
    u16                     lr  : 1;    ///< limited retry
    u8                      dsm;        ///< dataset management
    u8                      rsvd13[3];  ///< reserved (in cdw 13)
    u32                     eilbrt;     ///< exp initial block reference tag
    u16                     elbat;      ///< exp logical block app tag
    u16                     elbatm;     ///< exp logical block app tag mask
} nvme_command_rw_t;

/// Admin command:  Delete I/O Submission & Completion Queue
typedef struct _nvme_acmd_delete_ioq {
    nvme_command_common_t   common;     ///< common cdw 0
    u16                     qid;        ///< queue id (cdw 10)
    u16                     rsvd10;     ///< reserved (in cdw 10)
    u32                     cwd11_15[5]; ///< reserved (cdw 11-15)
} nvme_acmd_delete_ioq_t;

/// Admin command:  Create I/O Submission Queue
typedef struct _nvme_acmd_create_sq {
    nvme_command_common_t   common;     ///< common cdw 0
    u16                     qid;        ///< queue id (cdw 10)
    u16                     qsize;      ///< queue size
    u16                     pc : 1;     ///< physically contiguous
    u16                     qprio : 2;  ///< interrupt enabled
    u16                     rsvd11 : 13; ///< reserved (in cdw 11)
    u16                     cqid;       ///< associated completion queue id
    u32                     cdw12_15[4]; ///< reserved (cdw 12-15)
} nvme_acmd_create_sq_t;

/// Admin command:  Get Log Page
typedef struct _nvme_acmd_get_log_page {
    nvme_command_common_t   common;     ///< common cdw 0
    u8                      lid;        ///< log page id (cdw 10)
    u8                      rsvd10a;    ///< reserved (in cdw 10)
    u16                     numd : 12;  ///< number of dwords
    u16                     rsvd10b : 4; ///< reserved (in cdw 10)
    u32                     rsvd11[5];  ///< reserved (cdw 11-15)
} nvme_acmd_get_log_page_t;

/// Admin command:  Create I/O Completion Queue
typedef struct _nvme_acmd_create_cq {
    nvme_command_common_t   common;     ///< common cdw 0
    u16                     qid;        ///< queue id (cdw 10)
    u16                     qsize;      ///< queue size
    u16                     pc : 1;     ///< physically contiguous
    u16                     ien : 1;    ///< interrupt enabled
    u16                     rsvd11 : 14; ///< reserved (in cdw 11)
    u16                     iv;         ///< interrupt vector
    u32                     cdw12_15[4]; ///< reserved (cdw 12-15)
} nvme_acmd_create_cq_t;

/// Admin command:  Identify
typedef struct _nvme_acmd_identify {
    nvme_command_common_t   common;     ///< common cdw 0
    u32                     cns;        ///< controller or namespace (cdw 10)
    u32                     cdw11_15[5]; ///< reserved (cdw 11-15)
} nvme_acmd_identify_t;

/// Admin command:  Abort
typedef struct _nvme_acmd_abort {
    nvme_command_common_t   common;     ///< common cdw 0
    u16                     sqid;       ///< submission queue id (cdw 10)
    u16                     cid;        ///< command id
    u32                     cdw11_15[5]; ///< reserved (cdw 11-15)
} nvme_acmd_abort_t;

/// Admin data:  Identify Controller Data
typedef struct _nvme_identify_ctlr {
    u16                     vid;        ///< PCI vendor id
    u16                     ssvid;      ///< PCI subsystem vendor id
    char                    sn[20];     ///< serial number
    char                    mn[40];     ///< model number
    char                    fr[8];      ///< firmware revision
    u8                      rab;        ///< recommended arbitration burst
    u8                      ieee[3];    ///< IEEE OUI identifier
    u8                      mic;        ///< multi-interface capabilities
    u8                      mdts;       ///< max data transfer size
    u8                      rsvd78[178]; ///< reserved (78-255)
    u16                     oacs;       ///< optional admin command support
    u8                      acl;        ///< abort command limit
    u8                      aerl;       ///< async event request limit
    u8                      frmw;       ///< firmware updates
    u8                      lpa;        ///< log page attributes
    u8                      elpe;       ///< error log page entries
    u8                      npss;       ///< number of power states support
    u8                      avscc;      ///< admin vendor specific config
    u8                      rsvd265[247]; ///< reserved (265-511)
    u8                      sqes;       ///< submission queue entry size
    u8                      cqes;       ///< completion queue entry size
    u8                      rsvd514[2]; ///< reserved (514-515)
    u32                     nn;         ///< number of namespaces
    u16                     oncs;       ///< optional NVM command support
    u16                     fuses;      ///< fused operation support
    u8                      fna;        ///< format NVM attributes
    u8                      vwc;        ///< volatile write cache
    u16                     awun;       ///< atomic write unit normal
    u16                     awupf;      ///< atomic write unit power fail
    u8                      nvscc;      ///< NVM vendoe specific config
    u8                      rsvd531[173]; ///< reserved (531-703)
    u8                      rsvd704[1344]; ///< reserved (704-2047)
    u8                      psd[1024];  ///< power state 0-31 descriptors
    u8                      vs[1024];   ///< vendor specific
} nvme_identify_ctlr_t;

/// Admin data:  Identify Namespace - LBA Format Data
typedef struct _nvme_lba_format {
    u16                     ms;         ///< metadata size
    u8                      lbads;      ///< LBA data size
    u8                      rp : 2;     ///< relative performance
    u8                      rsvd : 6;   ///< reserved
} nvme_lba_format_t;

/// Admin data:  Identify Namespace Data
typedef struct _nvme_identify_ns {
    u64                     nsze;       ///< namespace size
    u64                     ncap;       ///< namespace capacity
    u64                     nuse;       ///< namespace utilization
    u8                      nsfeat;     ///< namespace features
    u8                      nlbaf;      ///< number of LBA formats
    u8                      flbas;      ///< formatted LBA size
    u8                      mc;         ///< metadata capabilities
    u8                      dpc;        ///< data protection capabilities
    u8                      dps;        ///< data protection settings
    u8                      rsvd30[98]; ///< reserved (30-127)
    nvme_lba_format_t       lbaf[16];   ///< lba format support
    u8                      rsvd192[192]; ///< reserved (383-192)
    u8                      vs[3712];   ///< vendor specific
} nvme_identify_ns_t;

/// Admin data:  Get Log Page - Error Information
typedef struct _nvme_log_page_error {
    u64                     count;      ///< error count
    u16                     sqid;       ///< submission queue id
    u16                     cid;        ///< command id
    u16                     sf;         ///< status field
    u8                      byte;       ///< parameter byte error location
    u8                      bit: 3;     ///< parameter bit error location
    u8                      rsvd : 5;   ///< reserved
    u64                     lba;        ///< logical block address
    u32                     ns;         ///< name space
    u8                      vspec;      ///< vendor specific infomation
    u8                      rsvd29[35]; ///< reserved (29-63)
} nvme_log_page_error_t;

/// Admin data:  Get Log Page - SMART / Health Information
typedef struct _nvme_log_page_health {
    u8                      warn;       ///< critical warning
    u16                     temp;       ///< temperature
    u8                      avspare;     ///< available spare
    u8                      avsparethresh; ///< available spare threshold
    u8                      used;       ///< percentage used
    u8                      rsvd6[26];  ///< reserved (6-31)
    u64                     dur[2];     ///< data units read
    u64                     duw[2];     ///< data units written
    u64                     hrc[2];     ///< number of host read commands
    u64                     hwc[2];     ///< number of host write commands
    u64                     cbt[2];     ///< controller busy time
    u64                     pcycles[2]; ///< number of power cycles
    u64                     phours[2]; ///< power on hours
    u64                     unsafeshut[2]; ///< unsafe shutdowns
    u64                     merrors[2]; ///< media errors
    u64                     errlogs[2]; ///< number of error log entries
    u64                     rsvd192[320]; ///< reserved (192-511)
} nvme_log_page_health_t;

/// Admin data:  Get Log Page - Firmware Slot Information
typedef struct _nvme_log_page_fw {
    u8                      afi;        ///< active firmware info
    u8                      rsvd1[7];   ///< reserved (1-7)
    u64                     fr[7];      ///< firmware revision for slot 1-7
    u8                      rsvd64[448]; ///< reserved (64-511)
} nvme_log_page_fw_t;

/// Submission queue entry
typedef union _nvme_sq_entry {
    nvme_command_rw_t       rw;         ///< read/write command

    nvme_acmd_abort_t       abort;      ///< admin abort command
    nvme_acmd_create_cq_t   create_cq;  ///< admin create IO completion queue
    nvme_acmd_create_sq_t   create_sq;  ///< admin create IO submission queue
    nvme_acmd_delete_ioq_t  delete_ioq; ///< admin delete IO queue
    nvme_acmd_identify_t    identify;   ///< admin identify command
    nvme_acmd_get_log_page_t get_log_page; ///< get log page command
} nvme_sq_entry_t;

/// Completion queue entry
typedef struct _nvme_cq_entry {
    u32                     cs;         ///< command specific
    u32                     rsvd;       ///< reserved
    u16                     sqhd;       ///< submission queue head
    u16                     sqid;       ///< submission queue id
    u16                     cid;        ///< command id
    union {
        u16                 psf;        ///< phase bit and status field
        struct {
            u16             p : 1;      ///< phase tag id
            u16             sc : 8;     ///< status code
            u16             sct : 3;    ///< status code type
            u16             rsvd3 : 2;  ///< reserved
            u16             m : 1;      ///< more
            u16             dnr : 1;    ///< do not retry
        };
    };
} nvme_cq_entry_t;

struct _nvme_device;

/// Queue context (a submission-completion queue pair context)
typedef struct _nvme_queue {
    struct _nvme_device*    dev;        ///< device reference
    int                     id;         ///< queue id
    int                     size;       ///< queue size
    nvme_sq_entry_t*        sq;         ///< submission queue
    nvme_cq_entry_t*        cq;         ///< completion queue
    u32*                    sq_doorbell; ///< submission queue doorbell
    u32*                    cq_doorbell; ///< completion queue doorbell
    int                     sq_tail;    ///< submission queue tail
    int                     cq_head;    ///< completion queue head
    int                     cq_phase;   ///< completion queue phase bit
} nvme_queue_t;

/// Device context
typedef struct _nvme_device {
    nvme_controller_reg_t*  reg;        ///< register address map
    int                     dbstride;   ///< doorbell stride (in word size)
    int                     maxqsize;   ///< max queue size
    int                     pageshift;  ///< pagesize shift
    struct _nvme_queue      adminq;     ///< admin queue reference
} nvme_device_t;


// Export functions
nvme_device_t* nvme_create(int mapfd);
void nvme_delete(nvme_device_t* dev);

nvme_queue_t* nvme_setup_adminq(nvme_device_t* dev, int qsize,
                                void* sqbuf, u64 sqpa, void* cqbuf, u64 cqpa);

nvme_queue_t* nvme_create_ioq(nvme_device_t* dev, int id, int qsize,
                              void* sqbuf, u64 sqpa, void* cqbuf, u64 cqpa);
int nvme_delete_ioq(nvme_queue_t* ioq);

int nvme_cmd_identify(nvme_device_t* dev, int nsid, u64 prp1, u64 prp2);
int nvme_cmd_get_log_page(nvme_device_t* dev, int nsid,
                          int lid, int numd, u64 prp1, u64 prp2);
int nvme_cmd_create_cq(nvme_queue_t* ioq, u64 prp);
int nvme_cmd_create_sq(nvme_queue_t* ioq, u64 prp);
int nvme_cmd_delete_cq(nvme_queue_t* ioq);
int nvme_cmd_delete_sq(nvme_queue_t* ioq);

int nvme_cmd_rw(int opc, nvme_queue_t* ioq, int nsid, int cid,
                u64 lba, int nb, u64 prp1, u64 prp2);
int nvme_cmd_read(nvme_queue_t* ioq, int nsid, int cid,
                  u64 lba, int nb, u64 prp1, u64 prp2);
int nvme_cmd_write(nvme_queue_t* ioq, int nsid, int cid,
                   u64 lba, int nb, u64 prp1, u64 prp2);

int nvme_check_completion(nvme_queue_t* q, int* stat);
int nvme_wait_completion(nvme_queue_t* q, int cid, int timeout);

#endif  // _UNVME_NVME_H
