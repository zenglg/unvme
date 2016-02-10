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
 * @brief UNVMe TPC model specific routines.
 */

#include "unvme.h"

/// APC library model
int unvme_model = UNVME_MODEL_TPC;


/**
 * Create IO queue model extended function.
 * @param   ioq         io queue
 */
void unvme_ioq_create_ext(unvme_queue_t* ioq)
{
    unvme_tpc_create(ioq);
}

/**
 * Delete IO queue model extended function.
 * @param   ioq         io queue
 */
void unvme_ioq_delete_ext(unvme_queue_t* ioq)
{
    unvme_tpc_delete(ioq);
}

/**
 * Poll and wait for a specific page io completion in a queue.
 * @param   ns          namespace handle
 * @param   pa          page array
 * @param   sec         number of seconds to wait before timeout
 * @return  pointer to the page array completed or NULL if timeout.
 */
unvme_page_t* unvme_poll(const unvme_ns_t* ns, unvme_page_t* pa, int sec)
{
    unvme_queue_t* q = ((client_session_t*)(ns->ses))->queues[pa->qid].ioq;
    return unvme_tpc_poll(&q->datapool, pa, sec);
}

/**
 * Poll and wait for any page io completion in the specified queue.
 * @param   ns          namespace handle
 * @param   qid         client queue id
 * @param   sec         number of seconds to poll before timeout
 * @return  pointer to the page array completed or NULL if timeout.
 */
unvme_page_t* unvme_apoll(const unvme_ns_t* ns, int qid, int sec)
{
    unvme_queue_t* q = ((client_session_t*)(ns->ses))->queues[qid].ioq;
    return unvme_tpc_apoll(&q->datapool, sec);
}

