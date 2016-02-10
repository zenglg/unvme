#
# Copyright (c) 2015-2016, Micron Technology, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#   2. Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#   3. Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

include ../Makefile.def

ifneq ($(wildcard $(FIO_DIR)/fio.h),)
	TARGETS = unvme_fio
ifneq ($(wildcard $(SPDK_ROOT_DIR)/Makefile),)
	TARGETS += spdk_fio
endif
endif

SPDK_CPPFLAGS = -I$(SPDK_ROOT_DIR)/include -I$(CONFIG_DPDK_DIR)/include
SPDK_LIBS = -L$(CONFIG_DPDK_DIR)/lib \
	    	$(SPDK_ROOT_DIR)/lib/nvme/libspdk_nvme.a \
		$(SPDK_ROOT_DIR)/lib/util/libspdk_util.a \
		$(SPDK_ROOT_DIR)/lib/memory/libspdk_memory.a \
		-lrte_eal -lrte_malloc -lrte_mempool -lrte_ring -ldl \
		-lpciaccess -lpthread -lrt

UNVME_SRC = ../src
UNVME_CPPFLAGS = -I$(UNVME_SRC)
UNVME_LIBS = $(UNVME_SRC)/libunvme.a -lpthread -lrt

OBJS = $(addsuffix .o, $(TARGETS))

CPPFLAGS += -D_GNU_SOURCE -I$(FIO_DIR)
CFLAGS += $(COPT) -Wall -fPIC
LDFLAGS += -shared -rdynamic

all: $(OBJS) $(TARGETS)

unvme_fio: $(UNVME_SRC)/libunvme.a
unvme_fio.o: CPPFLAGS += $(UNVME_CPPFLAGS)
unvme_fio: LDLIBS = $(UNVME_LIBS)

spdk_fio.o: CPPFLAGS += $(SPDK_CPPFLAGS)
spdk_fio: LDLIBS = $(SPDK_LIBS)

lint: COPT = -D_FORTIFY_SOURCE=2 -DUNVME_DEBUG -O2
lint: clean $(OBJS)
	@$(RM) *.o

clean:
	$(RM) unvme_fio spdk_fio *.o

.PHONY: all lint clean
