# Helpers for running HolyC as /init of a Linux guest in QEMU, so it can
# drive emulated hardware (e1000, 16550 UARTs) itself. Sourced by
# tools/e1000test/run.sh, tools/hwtest/run.sh and tools/nettest/run.sh.
#   guest_ok          root + $QEMU (default qemu-system-x86_64)? else returns 1
#   guest_kernel DIR  sets VMLINUZ (given, or an Ubuntu kernel via apt-get download)
#   guest_initrd FILE.HC MAIN DIR   builds DIR/initrd.cpio with the HolyC as /init
QEMU=${QEMU:-qemu-system-x86_64}

guest_ok() {
  [ "$(id -u)" = 0 ] && command -v "$QEMU" >/dev/null
}

guest_kernel() {
  [ -n "$VMLINUZ" ] && return 0
  local pkg
  pkg=$(apt-cache search --names-only '^linux-image-unsigned-[0-9.]+-[0-9]+-generic$' | sort -V | tail -1 | cut -d' ' -f1)
  (cd "$1" && apt-get download -q "$pkg" >/dev/null 2>&1 && dpkg-deb -x ./*.deb k) || return 1
  VMLINUZ=$(ls "$1"/k/boot/vmlinuz-*)
}

guest_initrd() {
  python3 tools/hctest/hc2c.py c "$1" --main "$2" -o "$3/guest.c" || return 1
  gcc -std=gnu11 -O1 -fwrapv -fno-strict-aliasing -fms-extensions -w -static "$3/guest.c" \
    -o "$3/init" -lm -lpthread || return 1
  python3 tools/e1000test/initramfs.py "$3/init" "$3/initrd.cpio"
}
