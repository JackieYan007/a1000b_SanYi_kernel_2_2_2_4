#!/bin/sh

CROSS_COMPILETOOL=aarch64-bst-linux-

make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build bsta1000b_pat_defconfig
make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build -j32 
make CROSS_COMPILE=$CROSS_COMPILETOOL ARCH=arm64 O=build modules -j32

