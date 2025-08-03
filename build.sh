#!/bin/bash
set -e

# MMI-UHAS34.29-3 For moto penang 4G 4G+
# Kernel Modules:
# ---------------
# Download prebuilts folder from Android distribution. Move it to $my_top_dir
# For example:
# my_top_dir=$PWD
# mkdir -vp $my_top_dir/kernel
# cd $my_top_dir/kernel
# git clone https://android.googlesource.com/kernel/build
# mkdir -vp  $my_top_dir/kernel/prebuilts-master/clang/host
# cd $my_top_dir/kernel/prebuilts-master/clang/host
# git clone https://android.googlesource.com/platform/prebuilts/clang/host/linux-x86
# mkdir -vp  $my_top_dir/kernel/prebuilts/
# cd $my_top_dir/kernel/prebuilts/
# git clone https://android.googlesource.com/kernel/prebuilts/build-tools
# mkdir -vp $my_top_dir/vendor/aosp_gki/kernel/aarch64
# Download gki from:
# https://source.android.com/docs/core/architecture/kernel/gki-android13-5_10-release-builds
# cd -
#
# actually you can:
# my_top_dir=$PWD && mkdir -vp $my_top_dir/kernel/{prebuilts-master/clang/host,prebuilts} $my_top_dir/vendor/aosp_gki/kernel/aarch64 && git -C $my_top_dir/kernel clone --depth=1 https://android.googlesource.com/kernel/build && git -C $my_top_dir/kernel/prebuilts-master/clang/host clone --depth=1 https://android.googlesource.com/platform/prebuilts/clang/host/linux-x86 && git -C $my_top_dir/kernel/prebuilts clone --depth=1 https://android.googlesource.com/kernel/prebuilts/build-tools && curl -L -o /tmp/gki.tar.gz https://dl.google.com/android/gki/gki-android13-5.10-lts-release.tar.gz && tar -xf /tmp/gki.tar.gz -C $my_top_dir/vendor/aosp_gki/kernel/aarch64 --strip-components=1 && echo "✅ GKI downloaded and extracted to vendor/aosp_gki/kernel/aarch64"


# ========== CONFIGURATION ==========
my_top_dir="$PWD"
KERNEL_DIR="$my_top_dir/kernel-5.10"
OUT_BASE="$my_top_dir/out/target/product/penangf/obj/KERNEL_OBJ"
kernel_out_dir="$OUT_BASE/kernel-5.10"
MODULES_STAGING_DIR="$OUT_BASE/staging"
REL_KERNEL_OUT="$OUT_BASE"
TARGET_KERNEL_CONFIG="$kernel_out_dir/.config"

export PATH="$my_top_dir/prebuilts/build-tools/path/linux-x86:$my_top_dir/prebuilts/clang/host/linux-x86/clang-r416183b/bin:$my_top_dir/kernel/prebuilts/kernel-build-tools/linux-x86/bin:$PATH"

export CLANG_TRIPLE= CROSS_COMPILE=aarch64-linux-gnu- CROSS_COMPILE_COMPAT=arm-linux-gnueabi- \
CROSS_COMPILE_ARM32= ARCH=arm64 SUBARCH= MAKE_GOALS=all \
HOSTCC=clang HOSTCXX=clang++ CC=clang LD=ld.lld \
NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump \
OBJSIZE=llvm-size STRIP=llvm-strip

# ========== PREBUILTS SETUP ==========
echo "[*] Cloning toolchains..."
mkdir -p $my_top_dir/prebuilts/clang/host
#git clone https://android.googlesource.com/platform/prebuilts/clang/host/linux-x86 -b aml_tz3_314012010 $my_top_dir/prebuilts/clang/host/linux-x86

git clone https://android.googlesource.com/kernel/prebuilts/build-tools $my_top_dir/prebuilts/build-tools

# ========== CONFIGURE KERNEL ==========
echo "[*] Generating build config..."
mkdir -p "$kernel_out_dir"
python kernel-5.10/scripts/gen_build_config.py --kernel-defconfig penangf_defconfig --kernel-defconfig-overlays "" --kernel-build-config-overlays "" -m user -o $REL_KERNEL_OUT/build.config
cp -p kernel-5.10/arch/arm64/configs/penangf_defconfig "$REL_KERNEL_OUT/penangf.config"

# ========== BUILD KERNEL ==========
echo "[*] Building kernel..."
cd $KERNEL_DIR
make LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=dtc O=$kernel_out_dir gki_defconfig $REL_KERNEL_OUT/penangf.config
make -j$(nproc) O=$kernel_out_dir LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=dtc all vmlinux
make -j$(nproc) O=$kernel_out_dir LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=dtc INSTALL_MOD_PATH=$MODULES_STAGING_DIR modules_install

# ========== MODULE BUILD FUNCTION ==========
build_module() {
    local mod_path=$1
    local install_path=$2

    echo "[*] Building $mod_path"
    mkdir -p "$install_path"
    make -C $mod_path M=$mod_path KERNEL_SRC=$KERNEL_DIR O=$kernel_out_dir LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=dtc
    make -C $mod_path M=$mod_path KERNEL_SRC=$KERNEL_DIR O=$kernel_out_dir LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=dtc INSTALL_MOD_PATH=$install_path modules_install
}

# ========== STANDARD MODULES ==========
build_module ../vendor/mediatek/kernel_modules/met_drv_v3 "$REL_KERNEL_OUT/vendor/mediatek/kernel_modules/met_drv_v3"
build_module ../vendor/mediatek/kernel_modules/gpu/platform/mt6768 "$REL_KERNEL_OUT/vendor/mediatek/kernel_modules/gpu/platform/mt6768"
build_module ../vendor/mediatek/kernel_modules/connectivity/common "$REL_KERNEL_OUT/vendor/mediatek/kernel_modules/connectivity/common"
build_module ../vendor/mediatek/kernel_modules/connectivity/fmradio "$REL_KERNEL_OUT/vendor/mediatek/kernel_modules/connectivity/fmradio"
build_module ../vendor/mediatek/kernel_modules/connectivity/gps/gps_stp "$REL_KERNEL_OUT/vendor/mediatek/kernel_modules/connectivity/gps/gps_stp"

# ========== ETC SPECIAL MODULES ==========
cd "$kernel_out_dir"

declare -A etc_modules=(
    [udc]="../../ETC/udc_lib.ko_intermediates/LINKED"
    [connfem]="../../ETC/connfem.ko_intermediates/LINKED"
    [wmt_drv]="../../ETC/wmt_drv.ko_intermediates/LINKED"
    [bt_drv]="../../ETC/bt_drv_connac1x.ko_intermediates/LINKED"
    [wmt_chrdev_wifi]="../../ETC/wmt_chrdev_wifi.ko_intermediates/LINKED"
    [wlan_drv_gen4m]="../../ETC/wlan_drv_gen4m.ko_intermediates/LINKED"
)

for mod in "${!etc_modules[@]}"; do
    path="${etc_modules[$mod]}"
    echo "[*] Building ETC module $mod"
    make O=$kernel_out_dir LLVM=1 LLVM_IAS=1 DEPMOD=depmod DTC=dtc M=$path AUTOCONF_H=$kernel_out_dir/include/generated/autoconf.h \
         KBUILD_EXTRA_SYMBOLS="$kernel_out_dir/Module.symvers" \
         src="$my_top_dir/vendor/mediatek/kernel_modules/connectivity/${mod//_/\/}"
done

echo "[✔] Kernel and modules built successfully."

# ========== ASK USER ABOUT ANYKERNEL3 ==========
read -p "Do you want to create a TWRP-flashable AnyKernel3 ZIP? (y/n): " choice

if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
    echo "[*] Creating AnyKernel3 flashable zip..."

    # Paths
    ANYKERNEL_DIR="$my_top_dir/AnyKernel3"
    ANYKERNEL_OUT="$my_top_dir/AnyKernel3-penang-kernel.zip"

    # Clone AnyKernel3 if not exists
    if [ ! -d "$ANYKERNEL_DIR" ]; then
        git clone https://github.com/osm0sis/AnyKernel3.git "$ANYKERNEL_DIR"
    fi

    # Clean old files
    rm -f "$ANYKERNEL_DIR/Image.gz-dtb" "$ANYKERNEL_DIR/Image.gz"
    rm -rf "$ANYKERNEL_DIR/modules"
    mkdir -p "$ANYKERNEL_DIR/modules"

    # Copy kernel Image.gz-dtb (adjust if Image.gz or zImage is used)
    if [ -f "$kernel_out_dir/arch/arm64/boot/Image.gz-dtb" ]; then
        cp "$kernel_out_dir/arch/arm64/boot/Image.gz-dtb" "$ANYKERNEL_DIR/"
    else
        echo "[!] Kernel image not found! Check build output."
        exit 1
    fi

    # Copy modules
    find "$MODULES_STAGING_DIR" -name '*.ko' -exec cp {} "$ANYKERNEL_DIR/modules/" \;

    # Optional: strip modules to reduce size
    if command -v llvm-strip &> /dev/null; then
        find "$ANYKERNEL_DIR/modules" -name '*.ko' -exec llvm-strip --strip-unneeded {} \;
    fi

    # Customize anykernel.sh for your device
    sed -i 's|^block=.*|block=/dev/block/bootdevice/by-name/boot|' "$ANYKERNEL_DIR/anykernel.sh"
    sed -i 's|^is_slot_device=.*|is_slot_device=1|' "$ANYKERNEL_DIR/anykernel.sh"

    # Create ZIP
    cd "$ANYKERNEL_DIR"
    zip -r9 "$ANYKERNEL_OUT" ./* > /dev/null

    echo "[✔] Flashable zip created at: $ANYKERNEL_OUT"
else
    echo "[*] Skipping AnyKernel3 zip creation."
fi
