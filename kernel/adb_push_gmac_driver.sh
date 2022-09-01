export BUILD_PATH=/home/bst/kernel_src/sdk_2_2_2_4/01_SanYi/a1000b_SanYi_kernel_2_2_2_4/kernel/build/drivers/net/ethernet/bst/dwmac

adb push ${BUILD_PATH}/bst-gmac-platform.ko /home/root/userdata/bst-gmac-platform.ko
adb push ${BUILD_PATH}/bstmac.ko /home/root/userdata/bstmac.ko

adb shell sync


