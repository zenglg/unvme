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

TARGET_SVC := unvme
TARGET_LIB := libunvme.a

CFLAGS += $(COPT) -Wall -fPIC
LDLIBS += -pthread -lrt

INCLUDES := unvme.h libunvme.h \
           unvme_nvme.h unvme_vfio.h unvme_shm.h unvme_log.h rdtsc.h

COMMON_SRCS := unvme_nvme.c unvme_vfio.c unvme_shm.c unvme_log.c

SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)

ifneq ($(wildcard .model),)
	LAST_MODEL := $(shell cat .model)
	MAKE_MODEL := $(LAST_MODEL)
else
	MAKE_MODEL := $(DEFAULT_MODEL)
	LAST_MODEL := $(MAKE_MODEL)
endif

all: $(MAKE_MODEL)

model:
	@if [ "$(LAST_MODEL)" != "$(MAKE_MODEL)" ]; then	\
		$(RM) *.o $(TARGET_LIB);			\
		echo -n $(MAKE_MODEL) > .model;			\
	fi
	@echo "Making $(MAKE_MODEL)"

model_apc: MAKE_MODEL := model_apc
model_tpc: MAKE_MODEL := model_tpc
model_cs:  MAKE_MODEL := model_cs

model_apc model_tpc model_cs: model
	$(MAKE) -f Makefile.$(MAKE_MODEL)

%.i: %.c
	$(CPP) $(CPPFLAGS) -o $@ $<

lint: COPT = -D_FORTIFY_SOURCE=2 -DUNVME_DEBUG -O3
lint: clean $(OBJS)
	@$(RM) *.o

clean:
	$(RM) $(TARGET_SVC) $(TARGET_LIB) .model *.o *.i /dev/shm/unvme*

.PHONY: all model_apc model_tpc model_cs lint clean

.EXPORT_ALL_VARIABLES:

