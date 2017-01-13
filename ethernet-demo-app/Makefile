#
# Copyright 2016, Data61
# Commonwealth Scientific and Industrial Research Organisation (CSIRO)
# ABN 41 687 119 230.
#
# This software may be distributed and modified according to the terms of
# the BSD 2-Clause license. Note that NO WARRANTY is provided.
# See "LICENSE_BSD2.txt" for details.
#
# @TAG(D61_BSD)
#

TARGETS := $(notdir ${SOURCE_DIR}).cdl
ADL := ethernet-demo.camkes

Client_CFILES := $(wildcard ${SOURCE_DIR}/*.c)
Client_HFILES := $(wildcard ${SOURCE_DIR}/*.h)

Client_LIBS := sel4camkes sel4utils sel4vka sel4allocman sel4vspace sel4simple sel4platsupport lwip ethdrivers

CAMKES_FLAGS += --cpp-flag=-I${SOURCE_DIR}

include ${PWD}/tools/camkes/camkes.mk
