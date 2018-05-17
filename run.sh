#!/bin/sh
# set -exo pipefail
ndk-build.cmd -j4 NDK_DEBUG=0 1>&2

abi=$(adb shell getprop ro.product.cpu.abi | tr -d '\r')
sdk=$(adb shell getprop ro.build.version.sdk | tr -d '\r')
pre=$(adb shell getprop ro.build.version.preview_sdk | tr -d '\r')
rel=$(adb shell getprop ro.build.version.release | tr -d '\r')

echo 'abi:' $abi
echo 'sdk:' $sdk
echo 'pre:' $pre
echo 'release:' $rel

dir=/data/local/tmp/minicap-devel
adb shell "mkdir $dir 2>/dev/null || true"

adb push libs/${abi}/minicap ${dir}

if [ -e jni/minicap-shared/aosp/libs/android-$rel/$abi/minicap.so ]; then
  adb push jni/minicap-shared/aosp/libs/android-$rel/$abi/minicap.so $dir
else 
  adb push jni/minicap-shared/aosp/libs/android-$sdk/$abi/minicap.so $dir
fi

# size=$(adb shell dumpsys window | grep -Eo 'init=[0-9]+x[0-9]+' | head -1 | cut -d= -f 2)
args="-P 480x850"
echo mobilesize: $args
adb shell chmod 777 $dir/minicap
adb shell LD_LIBRARY_PATH=$dir $dir/minicap $args "$@"

c=`adb shell 'ps | grep minicap'`
pid=`echo $c | awk '{print $2}'`
adb shell kill ${pid}

adb shell rm -r $dir