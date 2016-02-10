/**
 * @file
 * @brief SPDK fio plugin engine.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <error.h>
#include <assert.h>
#include <pthread.h>
#include <pciaccess.h>

#include "rte_config.h"
#include "rte_mempool.h"
#include "rte_malloc.h"
#include "spdk/nvme.h"
#include "spdk/pci.h"

#include "config-host.h"
#include "fio.h"
#include "optgroup.h"       // since fio 2.4

#if 0
    #define DEBUG(fmt, arg...)  printf("#" fmt "\n", ##arg)
#else
    #define DEBUG(fmt, arg...)
#endif

#define TDEBUG(fmt, arg...) DEBUG("%s[%d] " fmt, __func__, td->thread_number, ##arg)

#define SPDK_PAGESIZE       4096

typedef struct {
    void*                   pad;
    unsigned int            nsid;
} spdk_options_t;

typedef struct {
    struct thread_data*     td;
    struct io_u*            io_u;
    void*                   rte_buf;
} spdk_iou_t;

typedef struct {
    struct io_u**           iocq;   // io completion queue
    int                     head;   // head of the io completion queue
    int                     tail;   // tail of the io completion queue
    int                     cbc;    // completion callback count
} spdk_data_t;

typedef struct {
    pthread_mutex_t         mutex;
    struct nvme_controller* ctrlr;
    struct nvme_namespace*  ns;
    uint64_t                ns_num_sectors;
    int                     ns_sector_size;
    int                     active;
} spdk_context_t;


// Static variables
static spdk_context_t       spdk = { .mutex = PTHREAD_MUTEX_INITIALIZER };

// Global request_mempool is used by libspdk_nvme.a and must be defined
struct rte_mempool*         request_mempool;


/*
 * Return number of CPUs available in system.
 */
static int get_num_cpus(void)
{
    const char* path_cpu_online = "/sys/devices/system/cpu/online";
    FILE* fp;
    char s[64];
    int ncpu = 1;

    if ((fp = fopen(path_cpu_online, "r")) > 0) {
        if (fgets(s, sizeof(s), fp)) {
            fclose(fp);
            ncpu = atoi(strchr(s, '-') + 1) + 1;
        }
    }
    return ncpu;
}

/*
 * Initialize SPDK controller and namespace.
 */
static int do_spdk_init(const char* pciname, struct thread_data *td)
{
    pthread_mutex_lock(&spdk.mutex);
    spdk.active++;
    if (spdk.ctrlr) {
        pthread_mutex_unlock(&spdk.mutex);
        return 0;
    }

    char coreargs[64];
    char* args[] = { "fio", coreargs, "-n4", "--log-level=6" };
    spdk_options_t* opt = td->eo;
    int nsid = opt->nsid ? opt->nsid : 1;
    int cores = get_num_cpus();
    if (cores > td->o.numjobs) cores = td->o.numjobs;

    struct pci_device_iterator* pci_dev_iter;
    struct pci_device* pci_dev;
    struct pci_id_match match;
    int bus, slot, func;

    sprintf(coreargs, "-c0x%lx", (1L << cores) - 1);
    if (rte_eal_init(4, args) < 0) error(1, 0, "rte_eal_init failed");

    request_mempool = rte_mempool_create("nvme_request", 8192,
                                         nvme_request_size(), 128, 0,
                                         NULL, NULL, NULL, NULL,
                                         SOCKET_ID_ANY, 0);
    if (!request_mempool) error(1, 0, "rte_mempool_create failed");

    pci_system_init();
    sscanf(pciname, "%d.%d.%d", &bus, &slot, &func);

    match.vendor_id = PCI_MATCH_ANY;
    match.subvendor_id = PCI_MATCH_ANY;
    match.subdevice_id = PCI_MATCH_ANY;
    match.device_id = PCI_MATCH_ANY;
    match.device_class = NVME_CLASS_CODE;
    match.device_class_mask = 0xFFFFFF;

    for (pci_dev_iter = pci_id_match_iterator_create(&match); ;) {
        pci_dev = pci_device_next(pci_dev_iter);
        if (!pci_dev) error(1, 0, "no NVMe device at pci %s", pciname);

        if ((pci_dev->bus == bus) && (pci_dev->dev == slot) &&
            (pci_dev->func == func)) {
            if (pci_device_has_non_null_driver(pci_dev)) {
                error(0, 0, "NVMe pci %s has driver", pciname);
                pthread_mutex_unlock(&spdk.mutex);
                return 1;
            }
            pci_device_probe(pci_dev);
            break;
        }
    }
    pci_iterator_destroy(pci_dev_iter);

    DEBUG("%s nvme_attach %d:%d.%d", __func__, bus, slot, func);
    spdk.ctrlr = nvme_attach(pci_dev);
    if (!spdk.ctrlr) {
        error(0, 0, "nvme_attach %s failed", pciname);
        pthread_mutex_unlock(&spdk.mutex);
        return 1;
    }

    spdk.ns = nvme_ctrlr_get_ns(spdk.ctrlr, nsid);
    if (!spdk.ns) {
        error(0, 0, "nvme_ctrlr_get_ns %d failed", nsid);
        pthread_mutex_unlock(&spdk.mutex);
        return 1;
    }
    spdk.ns_num_sectors = nvme_ns_get_num_sectors(spdk.ns);
    spdk.ns_sector_size = nvme_ns_get_sector_size(spdk.ns);

    pthread_mutex_unlock(&spdk.mutex);
    return 0;
}

/*
 * SPDK clean up.
 */
static void do_spdk_cleanup(void)
{
    pthread_mutex_lock(&spdk.mutex);
    spdk.active--;
    if (spdk.active == 0 && spdk.ctrlr) {
        DEBUG("%s nvme_detach", __func__);
        nvme_detach(spdk.ctrlr);
        spdk.ctrlr = NULL;
    }
    pthread_mutex_unlock(&spdk.mutex);
}


/*
 * The ->event() hook is called to match an event number with an io_u.
 * After the core has called ->getevents() and it has returned eg 3,
 * the ->event() hook must return the 3 events that have completed for
 * subsequent calls to ->event() with [0-2]. Required.
 */
static struct io_u* fio_spdk_event(struct thread_data *td, int event)
{
    spdk_data_t* sdata = td->io_ops->data;
    struct io_u* io_u = NULL;

    if (sdata->head != sdata->tail) {
        io_u = sdata->iocq[sdata->head];
        if (++sdata->head > td->o.iodepth) sdata->head = 0;
        TDEBUG("GET iou=%p head=%d", io_u, sdata->head);
    }
    return io_u;
}

/*
 * Completion callback function.
 */
static void spdk_completion_cb(void* data, const struct nvme_completion *nc)
{
    // add the completed entry to the iocq
    spdk_iou_t* siou = data;
    spdk_data_t* sdata = siou->td->io_ops->data;
    sdata->iocq[sdata->tail] = siou->io_u;
    if (++sdata->tail > siou->td->o.iodepth) sdata->tail = 0;
    sdata->cbc++;
    TDEBUG("PUT iou=%p tail=%d cbc=%d", siou->io_u, sdata->tail, sdata->cbc);
}

/*
 * The ->getevents() hook is used to reap completion events from an async
 * io engine. It returns the number of completed events since the last call,
 * which may then be retrieved by calling the ->event() hook with the event
 * numbers. Required.
 */
static int fio_spdk_getevents(struct thread_data *td, unsigned int min,
                              unsigned int max, const struct timespec *t)
{
    spdk_data_t* sdata = td->io_ops->data;
    int events = 0;
    struct timespec t0, t1;
    uint64_t timeout = 0;

    if (t) {
        timeout = t->tv_sec * 1000000000L + t->tv_nsec;
        clock_gettime(CLOCK_MONOTONIC_RAW, &t0);
    }

    for (;;) {
        nvme_ctrlr_process_io_completions(spdk.ctrlr, 0);

        // wait for completion
        while (sdata->cbc) {
            sdata->cbc--;
            events++;
            TDEBUG("events=%d cbc=%d", events, sdata->cbc);
            if (events >= min) return events;
        }

        if (t) {
            clock_gettime(CLOCK_MONOTONIC_RAW, &t1);
            uint64_t elapse = ((t1.tv_sec - t0.tv_sec) * 1000000000L)
                              + t1.tv_nsec - t0.tv_nsec;
            if (elapse > timeout) break;
        }
    }

    return events;
}

/*
 * The ->queue() hook is responsible for initiating io on the io_u
 * being passed in. If the io engine is a synchronous one, io may complete
 * before ->queue() returns. Required.
 *
 * The io engine must transfer in the direction noted by io_u->ddir
 * to the buffer pointed to by io_u->xfer_buf for as many bytes as
 * io_u->xfer_buflen. Residual data count may be set in io_u->resid
 * for a short read/write.
 */
static int fio_spdk_queue(struct thread_data *td, struct io_u *io_u)
{
    /*
     * Double sanity check to catch errant write on a readonly setup
     */
    fio_ro_check(td, io_u);

    int ret = 1;
    spdk_iou_t* siou = io_u->engine_data;
    uint64_t lba = io_u->offset / spdk.ns_sector_size;
    uint32_t count = io_u->xfer_buflen / spdk.ns_sector_size;

    switch (io_u->ddir) {
    case DDIR_READ:
        TDEBUG("READ iou=%p lba=%ld", io_u, lba);
        ret = nvme_ns_cmd_read(spdk.ns, siou->rte_buf, lba, count,
                               spdk_completion_cb, siou);
        break;
    case DDIR_WRITE:
        TDEBUG("WRITE iou=%p lba=%ld", io_u, lba);
        ret = nvme_ns_cmd_write(spdk.ns, siou->rte_buf, lba, count,
                                spdk_completion_cb, siou);
        break;
    default:
        break;
    }

    /*
     * Could return FIO_Q_QUEUED for a queued request,
     * FIO_Q_COMPLETED for a completed request, and FIO_Q_BUSY
     * if we could queue no more at this point (you'd have to
     * define ->commit() to handle that.
     */
    return ret ? FIO_Q_COMPLETED : FIO_Q_QUEUED;
}

/*
 * Hook for opening the given file. Unless the engine has special
 * needs, it usually just provides generic_file_open() as the handler.
 */
static int fio_spdk_open(struct thread_data *td, struct fio_file *f)
{
    return 0;
}

/*
 * Hook for closing a file. See fio_spdk_open().
 */
static int fio_spdk_close(struct thread_data *td, struct fio_file *f)
{
    return 0;
}

/*
 * The ->init() function is called once per thread/process, and should set up
 * any structures that this io engine requires to keep track of io. Not
 * required.
 */
static int fio_spdk_init(struct thread_data *td)
{
    spdk_data_t* sdata = calloc(1, sizeof(spdk_data_t));
    if (!sdata) return 1;

    sdata->iocq = calloc(td->o.iodepth + 1, sizeof(void*));
    if (!sdata->iocq) {
        free(sdata);
        return 1;
    }

    TDEBUG("register_io_thread %d", td->thread_number);
    nvme_register_io_thread();
    td->io_ops->data = sdata;
    return 0;
}

/*
 * This is paired with the ->init() function and is called when a thread is
 * done doing io. Should tear down anything setup by the ->init() function.
 * Not required.
 */
static void fio_spdk_cleanup(struct thread_data *td)
{
    spdk_data_t* sdata = td->io_ops->data;

    if (sdata) {
        if (sdata->iocq) free(sdata->iocq);
        free(sdata);
        TDEBUG("unregister_io_thread %d", td->thread_number);
        nvme_unregister_io_thread();
        do_spdk_cleanup();
    }
}

/*
 * The ->io_u_init() function is called once for each queue depth entry
 * (numjobs x iodepth) prior to .init and after .get_file_size.
 * It is needed if io_u buffer needs to be remapped.
 */
static int fio_spdk_io_u_init(struct thread_data *td, struct io_u *io_u)
{
    spdk_iou_t* siou = calloc(1, sizeof(spdk_iou_t));
    if (!siou) return 1;

    unsigned int maxlen = 0;
    if (td->o.bs[DDIR_READ] > maxlen) maxlen = td->o.bs[DDIR_READ];
    if (td->o.bs[DDIR_WRITE] > maxlen) maxlen = td->o.bs[DDIR_WRITE];
    if (td->o.max_bs[DDIR_READ] > maxlen) maxlen = td->o.max_bs[DDIR_READ];
    if (td->o.max_bs[DDIR_WRITE] > maxlen) maxlen = td->o.max_bs[DDIR_WRITE];

    siou->rte_buf = rte_malloc(NULL, maxlen, SPDK_PAGESIZE);
    if (!siou->rte_buf) {
        free(siou);
        return 1;
    }
    siou->td = td;
    siou->io_u = io_u;
    io_u->engine_data = siou;

    return 0;
}

/*
 * The ->io_u_free() function is called once for each queue depth entry
 * (numjobs x iodepth) prior to .init and after .get_file_size.
 * It is needed if io_u buffer needs to be remapped.
 */
static void fio_spdk_io_u_free(struct thread_data *td, struct io_u *io_u)
{
    spdk_iou_t* siou = io_u->engine_data;

    if (siou) {
        assert(siou->io_u == io_u);
        rte_free(siou->rte_buf);
        free(siou);
        io_u->engine_data = NULL;
    }
}

/*
 * The ->get_file_size() is called once for every job (i.e. numjobs)
 * before all other functions.  This is called after ->setup() but
 * is simpler to initialize here since we only care about the device name
 * (given as file_name) and just have to specify the device size.
 */
static int fio_spdk_get_file_size(struct thread_data *td, struct fio_file *f)
{
    TDEBUG("file=%s", f->file_name);
    if (!fio_file_size_known(f)) {
        if (do_spdk_init(f->file_name, td)) return 1;
        f->filetype = FIO_TYPE_CHAR;
        f->real_file_size = spdk.ns_num_sectors * spdk.ns_sector_size;
        fio_file_set_size_known(f);
    }
    return 0;
}


// SPDK options.
static struct fio_option fio_spdk_options[] = {
    {
        .name       = "nsid",
        .lname      = "NVMe nsid",
        .type       = FIO_OPT_INT,
        .off1       = offsetof(spdk_options_t, nsid),
        .minval     = 1,
        .maxval     = 0xffff,
        .help       = "NVMe namespace id",
        .category   = FIO_OPT_C_ENGINE,
    },
    {
        .name   = NULL,
    },
};


// Note that the structure is exported, so that fio can get it via
// dlsym(..., "ioengine");
struct ioengine_ops ioengine = {
    .name               = "spdk_fio",
    .version            = FIO_IOOPS_VERSION,
    .queue              = fio_spdk_queue,
    .getevents          = fio_spdk_getevents,
    .event              = fio_spdk_event,
    .init               = fio_spdk_init,
    .cleanup            = fio_spdk_cleanup,
    .open_file          = fio_spdk_open,
    .close_file         = fio_spdk_close,
    .get_file_size      = fio_spdk_get_file_size,
    .io_u_init          = fio_spdk_io_u_init,
    .io_u_free          = fio_spdk_io_u_free,
    .flags              = FIO_NOEXTEND | FIO_RAWIO,
    .options            = fio_spdk_options,
    .option_struct_size = sizeof(spdk_options_t),
};

