DLKM_DIR := motorola/kernel/modules
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := qpnp_adaptive_charge.ko
LOCAL_MODULE_TAGS := optional

ifeq ($(ADAPTIVE_TOLERANCE_OPTIMIZATION),true)
	KERNEL_CFLAGS += ADAPTIVE_TOLERANCE_OPTIMIZATION=y
endif

LOCAL_MODULE_PATH := $(KERNEL_MODULES_OUT)
include $(DLKM_DIR)/AndroidKernelModule.mk

