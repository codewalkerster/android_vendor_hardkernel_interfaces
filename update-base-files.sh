#!/bin/bash

if [ ! -d $ANDROID_BUILD_TOP/hardware/interfaces ] ; then
  echo "Where is hardware/interfaces?";
  exit 1;
fi

if [ ! -d $ANDROID_BUILD_TOP/system/libhidl/transport ] ; then
  echo "Where is system/libhidl/transport?";
  exit 1;
fi

echo "WARNING: This script changes file in many places."

# These files only exist to facilitate the easy transition to hidl.

options="-Lexport-header \
        -rvendor.hardkernel.hardware:vendor/hardkernel/interfaces \
        -randroid.hidl:system/libhidl/transport"

# hardware/libhardware
hidl-gen $options \
         -o $ANDROID_BUILD_TOP/hardware/libhardware/include/hardware/odroidthings-base.h \
         vendor.hardkernel.hardware.odroidthings@1.0
