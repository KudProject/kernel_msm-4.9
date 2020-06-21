#!/bin/sh
KO_PATH=platform
KO_FILE=ilitek

make clean ; make
#adb root
#adb push $KO_FILE.ko /data/
