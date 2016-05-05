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
 * @brief Common NVMe setup functions.
 */

#include <stdio.h>
#include <error.h>

#include "unvme_vfio.h"
#include "unvme_nvme.h"

static vfio_device_t* vfiodev;
static nvme_device_t* nvmedev;
static vfio_dma_t* adminsq;
static vfio_dma_t* admincq;

static void no_progname(void) {}

/**
 * NVMe setup.
 */
static void nvme_setup(const char* pciname, int aqsize)
{
    int b, d, f;
    if (sscanf(pciname, "%02x:%02x.%1x", &b, &d, &f) != 3) {
        error(1, 0, "invalid PCI device %s (expect BB:DD.F format)", pciname);
    }
    int pci = (b << 16) + (d << 8) + f;
    vfiodev = vfio_create(pci);
    if (!vfiodev) error(1, 0, "vfio_create");
    nvmedev = nvme_create(vfiodev->fd);
    if (!nvmedev) error(1, 0, "nvme_create");

    adminsq = vfio_dma_alloc(vfiodev, aqsize * sizeof(nvme_sq_entry_t));
    if (!adminsq) error(1, 0, "vfio_dma_alloc");
    admincq = vfio_dma_alloc(vfiodev, aqsize * sizeof(nvme_sq_entry_t));
    if (!admincq) error(1, 0, "vfio_dma_alloc");

    if (!nvme_setup_adminq(nvmedev, aqsize, adminsq->buf, adminsq->addr,
                                            admincq->buf, admincq->addr)) {
        error(1, 0, "nvme_setup_adminq");
    }
}

/**
 * NVMe cleanup.
 */
static void nvme_cleanup()
{
    vfio_dma_free(adminsq);
    vfio_dma_free(admincq);
    nvme_delete(nvmedev);
    vfio_delete(vfiodev);
}

