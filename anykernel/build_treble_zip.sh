#!/bin/bash

kernel_dir=.
ak_dir=anykernel/anykernel3
image=$kernel_dir/out/arch/arm64/boot/Image.gz-dtb
pre=KUD

[ -d $ak_dir ] && echo -e "\nAnykernel 3 Present.\n" \
|| mkdir -p $ak_dir \
| git clone --depth=1 https://github.com/osm0sis/AnyKernel3 $ak_dir
rm $ak_dir/anykernel.sh
cp $ak_dir/../anykernel.sh $ak_dir

cp -r $ak_dir ./ak_dir_working

cp $image ./Image.gz_treble

mv Image.gz_treble ./ak_dir_working/Image.gz-dtb && cd ak_dir_working
zip -r ${pre}_BOOT_MOCHI_TREBLE_`date +%d\.%m\.%Y_%H\:%M\:%S`.zip . -x '*.git*'
mv *.zip ../out
cd ..
rm -rf ak_dir_working
