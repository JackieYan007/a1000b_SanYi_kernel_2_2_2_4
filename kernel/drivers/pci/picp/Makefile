obj-$(CONFIG_BST_PICP)	+= picpm.o
ifeq ($(CONFIG_ARCH_BSTA1000A),y)
picpm-objs := picp.o #pcie_test.o
else ifeq ($(CONFIG_ARCH_BSTA1000B),y)
picpm-objs := picp.o #pcie_test_a1000b.o
endif

obj-y		+= pcie_save_conf/