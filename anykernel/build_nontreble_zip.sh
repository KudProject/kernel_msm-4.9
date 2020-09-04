#!/bin/bash

kernel_dir=.
ak_dir=anykernel/anykernel3
pronto_dir=anykernel/pronto_wlan
image=$kernel_dir/out/arch/arm64/boot/Image.gz-dtb
pre=KUD

[ -d $ak_dir ] && echo -e "\nAnykernel 3 Present.\n" \
|| mkdir -p $ak_dir \
| git clone --depth=1 https://github.com/osm0sis/AnyKernel3 $ak_dir
rm $ak_dir/anykernel.sh
cp $ak_dir/../anykernel.sh $ak_dir

cp -r $ak_dir ./ak_dir_working

cp $image ./Image.gz_nontreble

mv Image.gz_nontreble ./ak_dir_working/Image.gz-dtb && cd ak_dir_working
sed -i 's/patch_cmdline firmware/\# patch_cmdline firmware/g' anykernel.sh

zip -r ${pre}_BOOT_MOCHI_NON_TREBLE_`date +%d\.%m\.%Y_%H\:%M\:%S`.zip . -x '*.git*'
mv *.zip ../out
cd ..
rm -rf ak_dir_working

[ -d $pronto_dir ] && rm -rf $pronto_dir
unzip $ak_dir/../pronto_wlan_template.zip -d $pronto_dir

cd $kernel_dir/modules
find -iname "wlan.ko" -exec cp {} ../$pronto_dir/vendor/lib/modules/wlan.ko \;
cd ../$pronto_dir

zip -r ${pre}_PRONTO_WLAN_NON_TREBLE_`date +%d\.%m\.%Y_%H\:%M\:%S`.zip . -x '*.git*'
mv *.zip ../../out
