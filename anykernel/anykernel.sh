# AnyKernel3 Ramdisk Mod Script
# osm0sis @ xda-developers
properties() { '
kernel.string=MOCHI
do.devicecheck=0
do.modules=0
do.systemless=0
do.cleanup=1
do.cleanuponabort=0
device.name1=tenor_g
device.name2=10or
device.name3=G
device.name4=
device.name5=
supported.versions=
'; }

block=/dev/block/bootdevice/by-name/boot;
is_slot_device=0;
ramdisk_compression=auto;

. tools/ak3-core.sh;

chmod -R 750 $ramdisk/*;
chown -R root:root $ramdisk/*;

dump_boot;

ui_print "*******************************************"
ui_print "Updating Kernel and Patching cmdline..."
ui_print "*******************************************"


ui_print "*******************************************"
ui_print "Brought to you by MOCHI (TG: @mochi_wwww)"
ui_print "*******************************************"

patch_cmdline firmware_class.path firmware_class.path=/vendor/firmware_mnt/image
patch_cmdline androidboot.selinux androidboot.selinux=permissive

write_boot;
