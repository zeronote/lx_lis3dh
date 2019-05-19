#!/bin/bash

#check root
#check write priviledges and exec

CORES=$(grep -c ^processor /proc/cpuinfo)
TOOLS_DIR="../lis3dh_tools/"
TOOLCHAIN="gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux"
KERNEL_TAG="4.1.15-ti-rt-r43"
KERNEL_DIR="${TOOLS_DIR}/linux"

if [ ! -f "/usr/bin/dtc" ]; then
	echo "WARNING: it seems that dtc is not installed, you won't be able to generate the dtbo"
fi

mkdir -p ${TOOLS_DIR}

echo "Downloading linaro toolchain"
wget https://releases.linaro.org/archive/14.09/components/toolchain/binaries/${TOOLCHAIN}.tar.xz || { echo "download failed, check your internet connection"; exit 1; }
echo "Archive extraction..."
tar -xf ${TOOLCHAIN}.tar.xz -C ${TOOLS_DIR} 

echo "Download kernel sources"
git clone --depth 1 --branch ${KERNEL_TAG} https://github.com/beagleboard/linux ${TOOLS_DIR}/linux

cp lxbeaglebone_defconfig $KERNEL_DIR/.config 
cp Module.symvers_beaglebone $KERNEL_DIR/Module.symvers_ext

ARM_CROSS_COMPILER="$(readlink -f ${TOOLS_DIR})/${TOOLCHAIN}/bin/arm-linux-gnueabihf-"

pushd $KERNEL_DIR

echo "Prepare linux sources"
make ARCH=arm CROSS_COMPILE=${ARM_CROSS_COMPILER} oldconfig
make ARCH=arm CROSS_COMPILE=${ARM_CROSS_COMPILER} prepare -j${CORES} 
make ARCH=arm CROSS_COMPILE=${ARM_CROSS_COMPILER} scripts -j${CORES}  
mv Module.symvers_ext Module.symvers

popd

echo "Compile dtb overlay"
dtc -O dtb -o BB-LX-ACCEL-00A0.dtbo -b 0 -@ lis3dh.dts

echo "OK. Now use \"make\" to CC the kernel module."
