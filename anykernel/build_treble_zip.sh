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

dtbdir=$kernel_dir/out/arch/arm64/boot/dts/qcom

dtc -I dtb -O dts -o $dtbdir/nontreble-inline-prima.dts $dtbdir/msm8937-pmi8937-qrd-sku2-holland1.dtb
echo "/ {firmware{android{fstab{vendor{status = \"disabled\";};};};};};" >> $dtbdir/nontreble-inline-prima.dts
dtc -I dts -O dtb -o $dtbdir/nontreble-inline-prima.dtb $dtbdir/nontreble-inline-prima.dts

rm $dtbdir/../../Image.gz-dtb
cat $dtbdir/nontreble-inline-prima.dtb >> $dtbdir/../../Image.gz
mv $dtbdir/../../Image.gz $dtbdir/../../Image.gz-dtb
cp $image ./Image.gz_nontreble-inline-prima
mv Image.gz_nontreble-inline-prima ./ak_dir_working/Image.gz-dtb && cd ak_dir_working
zip -r ${pre}_BOOT_MOCHI_NON_TREBLE_PRIMA_INLINE_`date +%d\.%m\.%Y_%H\:%M\:%S`.zip . -x '*.git*'
mv *.zip ../out
cd ..

rm -rf ak_dir_working
