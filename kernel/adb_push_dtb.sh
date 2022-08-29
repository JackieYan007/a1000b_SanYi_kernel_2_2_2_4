export BUILD_PATH=/home/bst/kernel_src/sdk_2_2_2_4/01_SanYi/a1000b_SanYi_kernel_2_2_2_4/kernel/build/arch/arm64/boot/dts/bst

adb shell mount /dev/mmcblk0p1 /mnt/boot/
adb shell ls -l /mnt/boot/

adb push ${BUILD_PATH}/bsta1000b-pat.dtb /mnt/boot/bsta1000b-pat.dtb

adb shell sync
adb shell umount /mnt/boot/
adb shell reboot

