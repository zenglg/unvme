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
 * @brief Shared memory support routines.
 */

#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include "unvme_log.h"
#include "unvme_shm.h"


/**
 * Create a shared memory file.
 * @param   filename    filename
 * @param   size        size
 * @return  a shared memory file reference.
 */
shm_file_t* shm_create(const char* filename, size_t size)
{
    shm_file_t* shm = zalloc(sizeof(*shm));
    strncpy(shm->name, filename, sizeof(shm->name) - 1);
    DEBUG_FN("%s", shm->name);
    shm->fd = shm_open(shm->name, O_CREAT|O_RDWR, S_IRUSR|S_IWUSR);
    if (shm->fd <= 0) {
        ERROR("%s errno %d", filename, errno);
        goto error;
    }
    if (ftruncate(shm->fd, size) < 0) {
        ERROR("ftruncate %s errno %d", filename, errno);
        goto error;
    }
    shm->buf = mmap(0, size, PROT_READ|PROT_WRITE, MAP_SHARED, shm->fd, 0);
    if (shm->buf == MAP_FAILED) {
        ERROR("mmap errno %d", errno);
        shm->buf = NULL;
        goto error;
    }
    memset(shm->buf, 0, size);
    shm->size = size;
    return shm;

error:
    shm_delete(shm);
    return NULL;
}

/**
 * Delete a shared memory file.
 * @param   shm         shared memory file
 * @return  0 if ok else -1.
 */
int shm_delete(shm_file_t* shm)
{
    if (!shm) return 0;
    DEBUG_FN("%s", shm->name);
    if (shm->buf) {
        if (munmap(shm->buf, shm->size) < 0) {
            ERROR("munmap errno %d", errno);
            return -1;
        }
    }
    if (shm->fd > 0) {
        close(shm->fd);
        if (shm_unlink(shm->name)) {
            ERROR("shm_unlink %s errno %d", shm->name, errno);
            return -1;
        }
    }
    free(shm);
    return 0;
}

/**
 * Map an existing shared memory file.
 * @param   filename        filename
 * @return  a shared memory file reference.
 */
shm_file_t* shm_map(const char* filename)
{
    shm_file_t* shm = zalloc(sizeof(*shm));
    strncpy(shm->name, filename, sizeof(shm->name) - 1);
    shm->fd = shm_open(shm->name, O_RDWR, S_IRUSR|S_IWUSR);
    if (shm->fd <= 0) {
        ERROR("%s errno %d", filename, errno);
        goto error;
    }
    struct stat fs;
    if (fstat(shm->fd, &fs)) {
        ERROR("fstat %s errno %d", filename, errno);
        goto error;
    }
    shm->buf = mmap(0, fs.st_size, PROT_READ|PROT_WRITE, MAP_SHARED, shm->fd, 0);
    if (shm->buf == MAP_FAILED) {
        ERROR("mmap errno %d", errno);
        shm->buf = NULL;
        goto error;
    }
    shm->size = fs.st_size;
    return shm;

error:
    shm_unmap(shm);
    return NULL;
}

/**
 * Unmap a shared memory file.
 * @param   shm         shared memory file
 * @return  0 if ok else -1.
 */
int shm_unmap(shm_file_t* shm)
{
    if (!shm) return 0;
    if (shm->buf) {
        if (munmap(shm->buf, shm->size) < 0) {
            ERROR("munmap errno %d", errno);
            return -1;
        }
    }
    if (shm->fd > 0) close(shm->fd);
    free(shm);
    return 0;
}

