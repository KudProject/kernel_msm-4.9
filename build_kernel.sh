#!/bin/bash

# Do not touch this.

source anykernel/build_helper.sh

[[ $1 == 't' || $1 == 'dt' ]] && treble
[[ $1 == 'n' || $1 == 'dn' ]] && nontreble

[[ $1 == 't' || $1 == 'dt' ]] && make O=out ARCH=arm64 treble_defconfig
[[ $1 == 'n' || $1 == 'dn' ]] && make O=out ARCH=arm64 nontreble_defconfig

[[ $1 == 'n' || $1 == 't' ]] && pcmake -j$(nproc --all)

[ $1 == 'n' ] && pcmod

[[ $1 == 'dt' || $1 == 'dn' ]] && pcmake dtbs

rmconf

[ $1 == 'n' ] && ./anykernel/build_nontreble_zip.sh
[ $1 == 't' ] && ./anykernel/build_treble_zip.sh
