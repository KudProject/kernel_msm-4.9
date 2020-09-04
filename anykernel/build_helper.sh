#!/bin/bash

configdir=$(pwd)/arch/arm64/configs

CFG=holland1_defconfig

tcdir=${HOME}/android/TOOLS/proton-clang

[ -d $tcdir ] \
&& echo -e "\nProton-Clang Present.\n" \
|| echo -e "\nProton-Clang Not Present. Downloading Around 500MB...\n" \
| mkdir -p $tcdir \
| git clone --depth=1 https://github.com/kdrag0n/proton-clang $tcdir \
| echo "Done."

echo -e "\nChecking Clang Version...\n"
PATH="$tcdir/bin:${PATH}" \
clang --version
echo -e "\n"

[ -f ~/.bashrc ] && source ~/.bashrc
[ -f ~/.bash_profile ] && source ~/.bash_profile
[ -f ~/.profile ] && source ~/.profile

export LC_ALL=C && export USE_CCACHE=1
ccache -M 100G
echo -e "\nStarting Build...\n"

[ -d out ] && rm -rf out || mkdir -p out

treble() {
cp $configdir/$CFG $configdir/treble_defconfig
}

nontreble() {
cp $configdir/$CFG $configdir/nontreble_defconfig
echo "CONFIG_MACH_NONTREBLE_DTS=y" >> $configdir/nontreble_defconfig
echo "CONFIG_PRONTO_WLAN=m" >> $configdir/nontreble_defconfig
}

rmconf() {
[ -f $configdir/treble_defconfig ] && rm -rf $configdir/treble_defconfig
[ -f $configdir/nontreble_defconfig ] && rm -rf $configdir/nontreble_defconfig
}

pcmake() {
PATH="$tcdir/bin:${PATH}" \
make	\
	O=out \
	ARCH=arm64 \
	CC="ccache clang" \
	AR=llvm-ar \
	NM=llvm-nm \
	STRIP=llvm-strip \
	OBJCOPY=llvm-objcopy \
	OBJDUMP=llvm-objdump \
	OBJSIZE=llvm-size \
	READELF=llvm-readelf \
	HOSTCC=clang \
	HOSTCXX=clang++ \
	HOSTAR=llvm-ar \
	CROSS_COMPILE=aarch64-linux-gnu- \
	CROSS_COMPILE_ARM32=arm-linux-gnueabi- \
	CONFIG_DEBUG_SECTION_MISMATCH=y \
	CONFIG_NO_ERROR_ON_MISMATCH=y \
	$1 $2 $3 || exit
}

pcmod() {
[ -d "modules" ] && rm -rf modules || mkdir -p modules

pcmake INSTALL_MOD_PATH=../modules \
       INSTALL_MOD_STRIP=1 \
       modules_install || exit
}
