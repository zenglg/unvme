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
 * @brief VFIO support routines.
 */

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/pci.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <dirent.h>
#include <errno.h>

#include "unvme_vfio.h"
#include "unvme_log.h"

/// Starting device DMA address
#define VFIO_IOVA           0x8800000000

/// Adjust to 4K page aligned size
#define VFIO_PASIZE(n)      (((n) + 0xfff) & ~0xfff)

struct _vfio_dev;

/// VFIO memory allocation entry
typedef struct _vfio_mem {
    struct _vfio_dev*       dev;        ///< device ownder
    int                     mmap;       ///< mmap indication flag
    vfio_dma_t              dma;        ///< dma mapped memory
    size_t                  nelems;     ///< number of elements
    size_t                  elsize;     ///< element size
    struct _vfio_mem*       prev;       ///< previous entry
    struct _vfio_mem*       next;       ///< next entry
} vfio_mem_t;

/// VFIO actual device context
typedef struct _vfio_dev {
    int                     id;         ///< device id
    int                     fd;         ///< device descriptor
    int                     groupfd;    ///< group file descriptor
    int                     contfd;     ///< container file descriptor
    __u64                   iova;       ///< next DMA (virtual IO) address
    vfio_mem_t*             memlist;    ///< memory allocated list
    pthread_spinlock_t      lock;       ///< multithreaded lock
} vfio_dev_t;


/**
 * Read a vfio device.
 * @param   dev         device context
 * @param   buf         buffer to read into
 * @param   len         read size
 * @param   off         offset
 * @return  0 if ok, else -1.
 */
static int vfio_read(vfio_dev_t* dev, void* buf, size_t len, off_t off)
{
    if (pread(dev->fd, buf, len, off) != len) {
        ERROR("pread(off=%#lx len=%lu)", off, len);
        return -1;
    }
    return 0;
}

/**
 * Write to vfio device.
 * @param   dev         device context
 * @param   buf         buffer to read into
 * @param   len         read size
 * @param   off         offset
 * @return  0 if ok, else -1.
 */
static int vfio_write(vfio_dev_t* dev, void* buf, size_t len, off_t off)
{
    if (pwrite(dev->fd, buf, len, off) != len) {
        ERROR("pwrite(off=%#lx len=%lu)", off, len);
        return -1;
    }
    return 0;
}

/**
 * Allocate a VFIO memory array of specified number of elements and size.
 * The pmb value, if non-zero, indicates that the array has already
 * been allocated (via mmap) and only need to be mapped to DMA address.
 * @param   vdev        device context
 * @param   nelems      number of array elements
 * @param   elsize      element size
 * @param   pmb         premapped buffer
 * @return  memory structure pointer or NULL if error.
 */
static vfio_mem_t* vfio_mem_alloc(vfio_device_t* vdev,
                                  size_t nelems, size_t elsize, void* pmb)
{
    vfio_mem_t* mem = zalloc(sizeof(*mem));
    mem->nelems = nelems;
    mem->elsize = elsize;
    size_t size = VFIO_PASIZE(nelems * elsize);

    if (pmb) {
        mem->dma.buf = pmb;
    } else {
        mem->dma.buf = mmap(0, size, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANON, -1, 0);
        if (mem->dma.buf == MAP_FAILED) {
            ERROR("mmap errno %d", errno);
            goto error;
        }
        mem->mmap = 1;
    }

    vfio_dev_t* dev = (vfio_dev_t*)vdev;

    pthread_spin_lock(&dev->lock);
    struct vfio_iommu_type1_dma_map map = {
        .argsz = sizeof(map),
        .flags = (VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE),
        .size = (__u64)size,
        .iova = dev->iova,
        .vaddr = (__u64)mem->dma.buf,
    };

    if (ioctl(dev->contfd, VFIO_IOMMU_MAP_DMA, &map) < 0) {
        ERROR("ioctl VFIO_IOMMU_MAP_DMA errno %d", errno);
        pthread_spin_unlock(&dev->lock);
        goto error;
    }
    mem->dma.size = size;
    mem->dma.addr = map.iova;
    mem->dma.id = mem;
    mem->dev = dev;

    // add node to the memory list
    if (!dev->memlist) {
        mem->prev = mem;
        mem->next = mem;
        dev->memlist = mem;
    } else {
        mem->prev = dev->memlist->prev;
        mem->next = dev->memlist;
        dev->memlist->prev->next = mem;
        dev->memlist->prev = mem;
    }
    dev->iova = map.iova + size;
    DEBUG_FN("%d: %#lx %ld %#lx", dev->id, map.iova, map.size, dev->iova);
    pthread_spin_unlock(&dev->lock);

    return mem;

error:
    if (mem->mmap) munmap(mem->dma.buf, size);
    free(mem);
    return NULL;
}

/**
 * Free up VFIO memory.
 * @param   mem         memory pointer
 * @return  0 if ok else -1.
 */
static int vfio_mem_free(vfio_mem_t* mem)
{
    vfio_dev_t* dev = mem->dev;

    struct vfio_iommu_type1_dma_unmap unmap = {
        .argsz = sizeof(unmap),
        .size = (__u64)mem->dma.size,
        .iova = mem->dma.addr,
    };

    // unmap and free dma memory
    if (mem->dma.buf) {
        if (ioctl(dev->contfd, VFIO_IOMMU_UNMAP_DMA, &unmap) < 0) {
            ERROR("ioctl VFIO_IOMMU_MAP_DMA errno %d", errno);
            return -1;
        }
    }
    if (mem->mmap) {
        if (munmap(mem->dma.buf, mem->dma.size) < 0) {
            ERROR("munmap errno %d", errno);
            return -1;
        }
    }

    // remove node from memory list
    pthread_spin_lock(&dev->lock);
    if (mem->next == dev->memlist) dev->iova -= mem->dma.size;
    if (mem->next == mem) {
        dev->memlist = NULL;
        dev->iova = VFIO_IOVA;
    } else {
        mem->next->prev = mem->prev;
        mem->prev->next = mem->next;
        if (dev->memlist == mem) dev->memlist = mem->next;
        dev->iova = dev->memlist->prev->dma.addr + dev->memlist->prev->dma.size;
    }
    DEBUG_FN("%d: %#lx %ld %#lx", dev->id, unmap.iova, unmap.size, dev->iova);
    pthread_spin_unlock(&dev->lock);

    free(mem);
    return 0;
}

/**
 * Create a VFIO device context.
 * @param   vfid        vfio device id
 * @return  device context or NULL if failure.
 */
vfio_device_t* vfio_create(int vfid)
{
    struct vfio_group_status group_status = { .argsz = sizeof(group_status) };
    struct vfio_iommu_type1_info iommu_info = { .argsz = sizeof(iommu_info) };
    struct vfio_device_info dev_info = { .argsz = sizeof(dev_info) };

    // allocate and initialize device context
    vfio_dev_t* dev = zalloc(sizeof(*dev));
    dev->id = vfid;
    dev->iova = VFIO_IOVA;
    if (pthread_spin_init(&dev->lock, PTHREAD_PROCESS_PRIVATE)) return NULL;

    // map vfio context
    char path[64];
    sprintf(path, "/dev/vfio/%d", vfid);
    if ((dev->groupfd = open(path, O_RDWR)) < 0) {
        ERROR("open %s failed", path);
        goto error;
    }
    if ((dev->contfd = open("/dev/vfio/vfio", O_RDWR)) < 0) {
        ERROR("open /dev/vfio/vfio");
        goto error;
    }
    if (ioctl(dev->contfd, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
        ERROR("ioctl VFIO_GET_API_VERSION");
        goto error;
    }
    if (ioctl(dev->contfd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU) == 0) {
        ERROR("ioctl VFIO_CHECK_EXTENSION");
        goto error;
    }
    if (ioctl(dev->groupfd, VFIO_GROUP_GET_STATUS, &group_status) < 0) {
        ERROR("ioctl VFIO_GROUP_GET_STATUS");
        goto error;
    }
    if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
        ERROR("group not viable %#x", group_status.flags);
        goto error;
    }
    if (ioctl(dev->groupfd, VFIO_GROUP_SET_CONTAINER, &dev->contfd) < 0) {
        ERROR("ioctl VFIO_GROUP_SET_CONTAINER");
        goto error;
    }
    if (ioctl(dev->contfd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU) < 0) {
        ERROR("ioctl VFIO_SET_IOMMU");
        goto error;
    }
    if (ioctl(dev->contfd, VFIO_IOMMU_GET_INFO, &iommu_info) < 0) {
        ERROR("ioctl VFIO_IOMMU_GET_INFO");
        goto error;
    }

    // Get the device file descriptor
    char s[64];
    struct dirent** des;
    int i;
    sprintf(s, "/sys/kernel/iommu_groups/%d/devices", dev->id);
    if ((i = scandir(s, &des, 0, 0)) != 3) {
        ERROR("scandir %s returned %d", s, i);
        goto error;
    }
    strncpy(s, des[i-1]->d_name, sizeof(s));
    while (i--) free(des[i]);
    DEBUG_FN("%d: pci=%s", dev->id, s);

    dev->fd = ioctl(dev->groupfd, VFIO_GROUP_GET_DEVICE_FD, s);
    if (dev->fd < 0) {
        ERROR("ioctl VFIO_GROUP_GET_DEVICE_FD");
        goto error;
    }

    /* Test and setup the device */
    if (ioctl(dev->fd, VFIO_DEVICE_GET_INFO, &dev_info) < 0) {
        ERROR("ioctl VFIO_DEVICE_GET_INFO");
    }
    DEBUG_FN("%d: flags=%u regions=%u irqs=%u",
             dev->id, dev_info.flags, dev_info.num_regions, dev_info.num_irqs);

    for (i = 0; i < dev_info.num_regions; i++) {
        struct vfio_region_info reg = { .argsz = sizeof(reg) };
        reg.index = i;

        if (ioctl(dev->fd, VFIO_DEVICE_GET_REGION_INFO, &reg)) continue;

        DEBUG_FN("%d: region=%d flags=%u resv=%u off=%#llx size=%#llx",
                 dev->id, reg.index, reg.flags, reg.resv, reg.offset, reg.size);

        if (i == VFIO_PCI_CONFIG_REGION_INDEX) {
            if (vfio_read(dev, s, sizeof(s), reg.offset)) goto error;

            __u16* vendor = (__u16*)(s + PCI_VENDOR_ID);
            __u16* cmd = (__u16*)(s + PCI_COMMAND);

            DEBUG_FN("%d: vendor=%#x device=%#x rev=%d", dev->id,
                     *vendor, (__u16*)(s + PCI_DEVICE_ID), s[PCI_REVISION_ID]);
            if (*vendor == 0xffff) {
                ERROR("device in bad state");
                goto error;
            }

            *cmd |= PCI_COMMAND_MASTER|PCI_COMMAND_MEMORY|PCI_COMMAND_INTX_DISABLE;
            vfio_write(dev, cmd, sizeof(*cmd), reg.offset + PCI_COMMAND);
            //vfio_read(dev, cmd, sizeof(*cmd), reg.offset + PCI_COMMAND);
        }
    }

    return (vfio_device_t*)dev;

error:
    vfio_delete((vfio_device_t*)dev);
    return NULL;
}

/**
 * Delete a VFIO device context.
 * @param   vdev        device context
 */
void vfio_delete(vfio_device_t* vdev)
{
    if (!vdev) return;
    vfio_dev_t* dev = (vfio_dev_t*)vdev;
    DEBUG_FN("%d", dev->id);

    // free all memory associated with the device
    while (dev->memlist) vfio_mem_free(dev->memlist);

    if (dev->fd) {
        close(dev->fd);
        dev->fd = 0;
    }
    if (dev->contfd) {
        close(dev->contfd);
        dev->contfd = 0;
    }
    if (dev->groupfd) {
        close(dev->groupfd);
        dev->groupfd = 0;
    }

    pthread_spin_destroy(&dev->lock);
    free(dev);
}

/**
 * Map a premapped buffer and return a DMA buffer.
 * @param   vdev        device context
 * @param   size        allocation size
 * @param   pmb         premapped buffer
 * @return  0 if ok else -1.
 */
vfio_dma_t* vfio_dma_map(vfio_device_t* vdev, size_t size, void* pmb)
{
    vfio_mem_t* mem = vfio_mem_alloc(vdev, 1, size, pmb);
    return mem ? &mem->dma : NULL;
}

/**
 * Free a DMA buffer (without unmapping dma->buf).
 * @param   dma         memory pointer
 * @return  0 if ok else -1.
 */
int vfio_dma_unmap(vfio_dma_t* dma)
{
    return vfio_mem_free((vfio_mem_t*)dma->id);
}

/**
 * Allocate and return a DMA buffer.
 * @param   vdev        device context
 * @param   size        allocation size
 * @return  0 if ok else -1.
 */
vfio_dma_t* vfio_dma_alloc(vfio_device_t* vdev, size_t size)
{
    vfio_mem_t* mem = vfio_mem_alloc(vdev, 1, size, 0);
    return mem ? &mem->dma : NULL;
}

/**
 * Free a DMA buffer.
 * @param   dma         memory pointer
 * @return  0 if ok else -1.
 */
int vfio_dma_free(vfio_dma_t* dma)
{
    return vfio_mem_free((vfio_mem_t*)dma->id);
}

