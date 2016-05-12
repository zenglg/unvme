UNVMe - A User Space NVMe Driver Project
========================================

UNVMe is a user space NVMe driver developed at Micron Technology for the
purpose of benchmarking different polling models in comparison to the
kernel space NVMe driver interupt driven model.

From the application perspective, UNVMe provides a single library (libunvme.a)
with a set of custom APIs to read and write to NVMe devices.

In this project, UNVMe is implemented as three distinct models (choose one):

    model_apc   -   Using this library model, an application will have
                    exclusive access to a given device, where the
                    application will process the completion queue upon
                    checking for I/O status.  In this model, there will be no
                    context switch within the submission and completion paths.

    model_tpc   -   Using this library model, an application will have
                    exclusive access to a given device, where a built-in
                    thread will process all I/O completions, and
                    the application can retrieve the saved results upon checking
                    for an I/O status.  In this model, there may be context
                    switch in the completion path between the built-in
                    and the application thread.

    model_cs    -   Using the client/server model, the driver will run
                    as a background service to manage a set of devices.
                    A client application which runs as a separate process can
                    send requests to the driver.  This model is similar to
                    the traditional driver model which allows one or more
                    applications to access a device.  In this model, there may
                    be context switch in both submission and completion paths
                    between the application and the driver processes.

A selected model can be built by specifying the named target.
All three models export the same set of APIs.

It should be noted that the UNVMe interface is designed to allow the user
to create a specified number of I/O submission-completion queue pairs.
The queues will be created exclusively for and when a session is opened.
For multi-threaded applications, an application thread can perform I/O on
one or more queues, but a queue must only be accessed by a single thread
in order to guarantee thread-safe operations.


System Requirements
===================

UNVMe depends on features provided by the VFIO module in the Linux kernel
(introduced since 3.6).  It has only been built and tested on CentOS 6 and 7
running on x86_64 CPU based systems.  

UNVMe requires the following hardware and software support:

    VT-d    -   System with CPU that supports VT-d
                (Virtualization Technology for Directed I/O).
                Check the <a href="http://ark.intel.com">Intel product specifications</a>

    VFIO    -   Linux kernel 3.6 or later compiled with the following configurations:

                    CONFIG_IOMMU_API=y
                    CONFIG_IOMMU_SUPPORT=y
                    CONFIG_INTEL_IOMMU=y
                    CONFIG_VFIO=m
                    CONFIG_VFIO_PCI=m
                    CONFIG_VFIO_IOMMU_TYPE1=m
                    CONFIG_INTEL_IOMMU_DEFAULT_ON=y

                If CONFIG_INTEL_IOMMU_DEFAULT_ON is not set then the boot
                command line must set the "intel_iommu=on" argument.  To
                support both UNVMe and SPDK, set also the "iommu=pt" parameter.

                To verify the system is correctly configured with VFIO support,
                check that /sys/kernel/iommu_groups directory is not empty but
                contains other subdirectories (i.e. group numbers).

                On CentOS 6, which comes with kernel version 2.6, the user must
                compile and boot a newer kernel that has the VFIO module.
                The user must also copy the header file from the kernel source
                directory include/uapi/linux/vfio.h to /usr/include/linux.

    libpciaccess-devel - If missing, run "yum install libpciaccess-devel".


Setup Script
============

Before running any UNVMe application, the user must run the unvme-setup
script once to enable the NVMe devices for UNVMe binding.  The user may
choose to enable specific, or by default, all NVMe devices in the system.

To enable all NVMe devices in the system for UNVMe:

    $ test/unvme-setup

To enable specific PCI device slots for UNVMe:

    $ test/unvme-setup 05:00.0 06:00.0

To reset all NVMe devices to the kernel space driver:

    $ test/unvme-setup reset

To list all NVMe devices binding status:

    $ test/unvme-setup show


Build and Run
=============

The Makefile.def file is provided for the user to configure the default
model to build, compilation flags, and path to external programs.

A specific model can also be built by explicitly specifying it as make target.
For example:

    $ make model_apc
    $ make model_tpc
    $ make model_cs

Make will keep track of the last built model and will default to remake that
model (until clean) before using the DEFAULT_MODEL setting in Makefile.def.


When building model_apc or model_tpc, make will produce a 'libunvme.a' library.

When building the model_cs, make will produce a 'libunvme.a' library and
the 'unvme' executable which is the driver.  The driver must be run first
prior to client applications.  It takes a list of PCI devices as argument.
For example:

    $ src/unvme 05:00.0 06:00.0

The UNVMe runtime log is saved in /dev/shm/unvme.log.


Applications and Tests
======================

Applications can be built using the libunvme.h and libunvme.a interfaces.

Examples of UNVMe test applications are found in the "test" directory.
They may be run as:

    $ test/unvme/unvme_api_test 00:05.0
    $ test/unvme/unvme_mts_test 00:05.0


Benchmarks
==========

UNVMe has been benchmarked with FIO (Flexible I/O Tester) using the test script
test/unvme-benchmark.  The ioengine/unvme_fio is provided for this purpose.
In order to build the unvme_fio engine, the user must set the FIO_DIR variable
pointing to the fully compiled FIO source path in Makefile.def.


To produce benchmark results for UNVMe model_apc, run:

    $ test/unvme-setup  # only need to run once
    $ make model_apc
    $ OUTDIR=out/apc test/unvme-benchmark 05:00.0

To produce benchmark results for UNVMe model_tpc, run:

    $ make model_tpc
    $ OUTDIR=out/tpc test/unvme-benchmark 05:00.0

To produce benchmark results for UNVMe model_cs, run:

    $ make model_cs
    $ src/unvme 05:00.0
    $ OUTDIR=out/cs test/unvme-benchmark 05:00.0


To produce benchmark results for the NVMe kernel space driver, run:

    $ test/unvme-setup reset
    $ OUTDIR=out/nvme test/unvme-benchmark /dev/nvme0n1


Notes:

    + The unvme-benchmark script will run FIO tests for random read and then
      random write, each using 1, 4, 8, and 16 jobs (threads) with iodepth of
      1, 4, 8, 16, 32, and 64.  The number of jobs will be translated to
      the number of NVMe queues, and each queue size will be iodepth + 1.
      
    + The unvme-benchmark test will take approximalte 6 hours.  For more
      consistent results comparison, it is better to turn off the CPU power
      saving feature when running tests, by adding these arguments
      "processor.max_cstate=1 intel_idle.max_cstate=0" to the boot command line.

    + The default output directory for unvme-benchmark will be "out" relative
      to where the unvme-benchmark script resides.  The output directory can
      be overriden by specifying OUTDIR on the shell command line.

    + If the tested device nsid is other than 1 then the variable NSID must
      be specified on the shell command line
      (e.g. NSID=2 test/unvme-benchmark 05:00.0).

    + The implemented unvme_fio engine does not support the FIO 'verify' option.


Documentation
=============

Doxygen formatted documents for the source code can be generated as:

    $ make doc

The output HTML based document can then be browsed from doc/html/index.html.


Questions and Comments:
=======================

See also the <a href="https://github.com/MicronSSD/unvme/wiki">Wiki FAQ</a> page,
and feel free to create new issue for questions and comments.

