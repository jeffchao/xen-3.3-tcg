# Software-Based Trusted Platform Module (TPM) Emulator for Linux
# Copyright (C) 2004 Mario Strasser <mast@gmx.net>
# Copyright (C) 2006 INTEL Corp.
#
# $Id: Makefile 115 2006-06-23 10:36:44Z mast $

COMPILE_ARCH    ?= $(shell uname -m | sed -e s/i.86/x86_32/)

# module settings
BIN            := vtpmd
VERSION_MAJOR  := 0
VERSION_MINOR  := 4
VERSION_BUILD  := $(shell date +"%s")

# Installation program and options
INSTALL         = install
INSTALL_PROG    = $(INSTALL) -m0755
INSTALL_DIR     = $(INSTALL) -d -m0755

# Xen tools installation directory
TOOLS_INSTALL_DIR = $(DESTDIR)/usr/bin

CC      := gcc
CFLAGS  += -g -Wall $(INCLUDE) -DDEBUG
CFLAGS  += -I. -Itpm -I../../vtpm_manager/manager

# Is the simulator running in it's own vm?
#CFLAGS += -DVTPM_MULTI_VM

ifeq ($(COMPILE_ARCH),x86_64)
LIBDIR = lib64
else
LIBDIR = lib
endif

# GNU MP configuration
GMP_LIB        := /usr/$(LIBDIR)/libgmp.a
GMP_HEADER     := /usr/include/gmp.h

# sources and objects
src            ?= .
DIRS           := . crypto tpm 
SRCS           := $(foreach dir, $(DIRS), $(wildcard $(src)/$(dir)/*.c))
OBJS           := $(patsubst %.c, %.o, $(SRCS))
SRCS           += $(foreach dir, $(DIRS), $(wildcard $(src)/$(dir)/*.h))

obj-m               := $(BIN)
$(BIN)-objs := $(patsubst $(src)/%.o, %.o, $(OBJS)) crypto/libgmp.a

EXTRA_CFLAGS   += -I$(src) -I$(src)/crypto -I$(src)/tpm 

# do not print "Entering directory ..."
MAKEFLAGS      += --no-print-directory

all: $(BIN)

$(BIN):	$(src)/crypto/gmp.h $(src)/crypto/libgmp.a version $(SRCS) $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) $(src)/crypto/libgmp.a -o $(BIN)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

install: $(BIN)
	$(INSTALL_PROG) $(BIN) $(TOOLS_INSTALL_DIR)

clean:
	rm -f $(src)/crypto/gmp.h $(src)/crypto/libgmp.a $(OBJS)

mrproper: clean
	rm -f $(BIN) tpm_version.h

$(src)/crypto/libgmp.a:
	test -f $(src)/crypto/libgmp.a || ln -s $(GMP_LIB) $(src)/crypto/libgmp.a

# Note: We have to make sure that we use stack-based calling conventions 
# when using GNU MP library functions
$(src)/crypto/gmp.h:
	test -r $(src)/crypto/gmp.h || cat $(GMP_HEADER) | \
		sed -e "s/\(__GMP_DECLSPEC [^e].*\);/\1 __attribute__ ((regparm(0)));/" | \
		sed -e "s/^int$$/int __attribute__ ((regparm(0)))/" | \
		sed -e "s/^void$$/void __attribute__ ((regparm(0)))/" | \
		sed -e "s/^size_t$$/size_t __attribute__ ((regparm(0)))/" | \
		sed -e "s/^mp_limb_t$$/mp_limb_t __attribute__ ((regparm(0)))/" | \
		sed -e "s/^__GMP_EXTERN_INLINE void$$/__GMP_EXTERN_INLINE void __attribute__ ((regparm(0)))/" | \
		sed -e "s/^unsigned long$$/unsigned long __attribute__ ((regparm(0)))/" | \
		sed -e "s/\(.* (\*__gmp_allocate_func) .*\);/\1 __attribute__ ((regparm(0)));/" | \
		sed -e "s/\(.* (\*__gmp_reallocate_func) .*\);/\1 __attribute__ ((regparm(0)));/" | \
		sed -e "s/\(.* (\*__gmp_free_func) .*\);/\1 __attribute__ ((regparm(0)));/" \
		> $(src)/crypto/gmp.h

version:
	@echo "#ifndef _TPM_VERSION_H_" > $(src)/tpm_version.h
	@echo "#define _TPM_VERSION_H_" >> $(src)/tpm_version.h
	@echo "#define VERSION_MAJOR $(VERSION_MAJOR)" >> $(src)/tpm_version.h
	@echo "#define VERSION_MINOR $(VERSION_MINOR)" >> $(src)/tpm_version.h
	@echo "#define VERSION_BUILD $(VERSION_BUILD)" >> $(src)/tpm_version.h
	@echo "#endif /* _TPM_VERSION_H_ */" >> $(src)/tpm_version.h

.PHONY: all install clean dist gmp version

