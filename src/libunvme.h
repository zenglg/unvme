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
 * @brief User space NVMe client header file.
 */

#ifndef _LIBUNVME_H
#define _LIBUNVME_H

#include <stdint.h>


#ifndef _UNVME_TYPE
#define _UNVME_TYPE                 ///< bit size data types
typedef uint8_t         u8;         ///< 8-bit unsigned
typedef uint16_t        u16;        ///< 16-bit unsigned
typedef uint32_t        u32;        ///< 32-bit unsigned
typedef uint64_t        u64;        ///< 64-bit unsigned
#endif // _UNVME_TYPE


//#define UNVME_MAXIOLEN  (128*1024)  ///< max io transfer length
#define UNVME_TIMEOUT   60          ///< io timeout in seconds


/// Namespace attributes structure
typedef struct _unvme_ns {
    int                 id;         ///< namespace id
    int                 vid;        ///< PCI vendor id
    char                model[8];   ///< compiled model name
    char                sn[20];     ///< serial number
    char                mn[40];     ///< namespace model number
    char                fr[8];      ///< namespace firmware revision
    int                 maxqsize;   ///< max queue size supported
    int                 pagesize;   ///< page size
    int                 blocksize;  ///< logical block size
    u64                 blockcount; ///< total number of logical blocks
    int                 nbpp;       ///< number of blocks per page
    int                 maxppio;    ///< max numer of pages per io
    int                 maxbpio;    ///< max number of blocks per io
    int                 maxppq;     ///< max number of pages per queue
    int                 maxiopq;    ///< max io submissions per queue
    void*               ses;        ///< client session
} unvme_ns_t;

/// Memory allocated page structure.
typedef struct _unvme_page {
    void*               buf;        ///< data buffer
    u64                 slba;       ///< starting lba
    u16                 nlb;        ///< number of logical blocks to read/write
    u16                 offset;     ///< first buffer offset
    int                 stat;       ///< page status
    u16                 id;         ///< page id
    u16                 qid;        ///< client queue id
    void*               data;       ///< application private data
} unvme_page_t;


// Export functions
const unvme_ns_t* unvme_open(const char* vfio, int nsid, int qcount, int qsize);
int unvme_close(const unvme_ns_t* ns);

unvme_page_t* unvme_alloc(const unvme_ns_t* ns, int qid, int numpages);
int unvme_free(const unvme_ns_t* ns, unvme_page_t* pa);

int unvme_read(const unvme_ns_t* ns, unvme_page_t* pa);
int unvme_write(const unvme_ns_t* ns, unvme_page_t* pa);
int unvme_aread(const unvme_ns_t* ns, unvme_page_t* pa);
int unvme_awrite(const unvme_ns_t* ns, unvme_page_t* pa);

unvme_page_t* unvme_poll(const unvme_ns_t* ns, unvme_page_t* pa, int sec);
unvme_page_t* unvme_apoll(const unvme_ns_t* ns, int qid, int sec);

#endif // _LIBUNVME_H

