#!/usr/bin/env python3
"""Make a hard disk image with the repo on it, for getting ROS2-Holy-C
into TempleOS: an MBR with one FAT32 partition (TempleOS reads FAT32),
long file names included. Standard library only.

  python3 tools/mkdisk/mkdisk.py ros2holyc.img        (the repo -> /ROS2-Holy-C)
  python3 tools/mkdisk/mkdisk.py out.img DIR [DIR...]

Attach it as a second IDE disk, e.g.
  qemu-system-x86_64 -m 512 -cdrom TempleOS.ISO -drive file=ros2holyc.img,format=raw,if=ide,index=1
and in TempleOS copy it over (see docs/TEMPLEOS.md).
"""
import os
import struct
import sys

SECTOR = 512
SPC = 1                  # sectors per cluster: small files, and FAT32 needs >= 65525 clusters
RESERVED = 32
NFATS = 2
PART_START = 2048        # LBA of the partition
SKIP_DIRS = {".git", "__pycache__", ".github"}
SKIP_EXT = {".pyc", ".img", ".iso", ".ISO"}
DATE = (2026 - 1980) << 9 | 1 << 5 | 1      # FAT date: 2026-01-01
TIME = 0


class Fat32:
    def __init__(self, total_sectors):
        self.total = total_sectors
        clusters_guess = total_sectors // SPC
        self.fat_sectors = (clusters_guess * 4 + SECTOR - 1) // SECTOR + 1
        self.data_start = RESERVED + NFATS * self.fat_sectors
        self.nclusters = (total_sectors - self.data_start) // SPC
        if self.nclusters < 65525:
            raise SystemExit("image too small for FAT32 (make it at least 40 MB)")
        self.fat = [0] * (self.nclusters + 2)
        self.fat[0] = 0x0FFFFFF8
        self.fat[1] = 0x0FFFFFFF
        self.next_free = 2
        self.data = {}        # cluster -> bytes

    def alloc_chain(self, nbytes):
        n = max(1, (nbytes + SPC * SECTOR - 1) // (SPC * SECTOR))
        first = self.next_free
        if first + n > self.nclusters + 2:
            raise SystemExit("image full: use a bigger size")
        for i in range(n):
            c = first + i
            self.fat[c] = c + 1 if i < n - 1 else 0x0FFFFFFF
        self.next_free += n
        return first, n

    def write_chain(self, first, n, payload):
        size = SPC * SECTOR
        for i in range(n):
            chunk = payload[i * size:(i + 1) * size]
            if chunk.strip(b"\0"):
                self.data[first + i] = chunk


def short_name(name, used):
    """8.3 name (11 bytes) and whether a long name entry is needed."""
    base, ext = os.path.splitext(name)
    ext = ext[1:]
    ok = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-~!#$%&'()@^{}")
    if (1 <= len(base) <= 8 and len(ext) <= 3 and base == base.upper() and ext == ext.upper()
            and all(c in ok for c in base + ext) and name.count(".") <= 1):
        sn = base.ljust(8) + ext.ljust(3)
        if sn not in used:
            used.add(sn)
            return sn.encode(), False
    clean = lambda s: "".join(c if c in ok else "_" for c in s.upper().replace(".", ""))
    b, e = clean(base), clean(ext)[:3]
    for i in range(1, 100000):
        tail = "~%d" % i
        sn = (b[:8 - len(tail)] + tail).ljust(8) + e.ljust(3)
        if sn not in used:
            used.add(sn)
            return sn.encode(), True
    raise SystemExit("too many similar names in one folder")


def lfn_checksum(sn):
    s = 0
    for c in sn:
        s = (((s & 1) << 7) + (s >> 1) + c) & 0xFF
    return s


def lfn_entries(name, sn):
    u = name.encode("utf-16-le")
    chars = [u[i:i + 2] for i in range(0, len(u), 2)]
    chars.append(b"\0\0")
    while len(chars) % 13:
        chars.append(b"\xff\xff")
    parts = [chars[i:i + 13] for i in range(0, len(chars), 13)]
    if chars[-13:] == [b"\xff\xff"] * 13:      # name filled the last entry exactly
        parts = parts[:-1]
    out = []
    ck = lfn_checksum(sn)
    for i, p in enumerate(parts):
        seq = i + 1
        if i == len(parts) - 1:
            seq |= 0x40
        e = bytes([seq]) + b"".join(p[0:5]) + bytes([0x0F, 0, ck]) + b"".join(p[5:11]) + \
            b"\0\0" + b"".join(p[11:13])
        out.append(e)
    return list(reversed(out))


def dir_entry(sn, attr, cluster, size):
    return struct.pack("<11sBBBHHHHHHHI", sn, attr, 0, 0, TIME, DATE, DATE,
                       cluster >> 16, TIME, DATE, cluster & 0xFFFF, size)


def build(fs, src, cluster, parent):
    """Write directory src (already given cluster) and everything under it."""
    entries = []
    if parent is not None:
        entries.append(dir_entry(b".          ", 0x10, cluster, 0))
        entries.append(dir_entry(b"..         ", 0x10, parent, 0))
    used = set()
    names = sorted(os.listdir(src))
    for name in names:
        path = os.path.join(src, name)
        if name in SKIP_DIRS or os.path.splitext(name)[1] in SKIP_EXT or os.path.islink(path):
            continue
        sn, need_lfn = short_name(name, used)
        if need_lfn:
            entries.extend(lfn_entries(name, sn))
        if os.path.isdir(path):
            sub_entries = count_entries(path) + 2
            first, n = fs.alloc_chain(sub_entries * 32)
            entries.append(dir_entry(sn, 0x10, first, 0))
            build(fs, path, first, cluster)
        else:
            with open(path, "rb") as fh:
                data = fh.read()
            if data:
                first, n = fs.alloc_chain(len(data))
                fs.write_chain(first, n, data)
            else:
                first = 0
            entries.append(dir_entry(sn, 0x20, first, len(data)))
    payload = b"".join(entries)
    n = max(1, (len(payload) + SPC * SECTOR - 1) // (SPC * SECTOR))
    fs.write_chain(cluster, n, payload.ljust(n * SPC * SECTOR, b"\0"))


def count_entries(src):
    """Directory entries src needs (with long names), to size its cluster chain."""
    n, used = 0, set()
    for name in sorted(os.listdir(src)):
        path = os.path.join(src, name)
        if name in SKIP_DIRS or os.path.splitext(name)[1] in SKIP_EXT or os.path.islink(path):
            continue
        sn, need_lfn = short_name(name, used)
        n += 1 + (len(lfn_entries(name, sn)) if need_lfn else 0)
    return n


def make(out, dirs, size_mb):
    total = size_mb * 1024 * 1024 // SECTOR
    psize = total - PART_START
    fs = Fat32(psize)
    # the root holds one folder per source directory
    root_names = [os.path.basename(os.path.abspath(d)) for d in dirs]
    used = set()
    root_entries = [struct.pack("<11sB20x", b"ROS2HOLYC  ", 0x08)]  # volume label
    root_first, root_n = fs.alloc_chain(32 * 64)
    for d, name in zip(dirs, root_names):
        sn, need_lfn = short_name(name, used)
        if need_lfn:
            root_entries.extend(lfn_entries(name, sn))
        first, n = fs.alloc_chain((count_entries(d) + 2) * 32)
        root_entries.append(dir_entry(sn, 0x10, first, 0))
        build(fs, d, first, 0)
    payload = b"".join(root_entries)
    fs.write_chain(root_first, root_n, payload.ljust(root_n * SPC * SECTOR, b"\0"))

    with open(out, "wb") as img:
        img.truncate(total * SECTOR)
        # MBR: one FAT32 (LBA) partition
        mbr = bytearray(SECTOR)
        mbr[446:462] = struct.pack("<B3sB3sII", 0x00, b"\xfe\xff\xff", 0x0C, b"\xfe\xff\xff",
                                   PART_START, psize)
        mbr[510:512] = b"\x55\xaa"
        img.seek(0)
        img.write(mbr)
        # boot sector (BPB)
        bs = bytearray(SECTOR)
        bs[0:3] = b"\xeb\x58\x90"
        bs[3:11] = b"ROS2HC  "
        struct.pack_into("<HBHBHHBHHHII", bs, 11, SECTOR, SPC, RESERVED, NFATS, 0, 0, 0xF8, 0,
                         63, 255, PART_START, psize)
        struct.pack_into("<IHHIHH12xBBBI11s8s", bs, 36, fs.fat_sectors, 0, 0, root_first, 1, 6,
                         0x80, 0, 0x29, 0x20260101, b"ROS2HOLYC  ", b"FAT32   ")
        bs[510:512] = b"\x55\xaa"
        # FSInfo
        fsi = bytearray(SECTOR)
        struct.pack_into("<I", fsi, 0, 0x41615252)
        struct.pack_into("<III", fsi, 484, 0x61417272, fs.nclusters + 2 - fs.next_free, fs.next_free)
        struct.pack_into("<I", fsi, 508, 0xAA550000)
        base = PART_START * SECTOR
        for at in (0, 6):     # boot sector + backup
            img.seek(base + at * SECTOR)
            img.write(bs)
            img.seek(base + (at + 1) * SECTOR)
            img.write(fsi)
        fat = struct.pack("<%dI" % len(fs.fat), *fs.fat)
        for i in range(NFATS):
            img.seek(base + (RESERVED + i * fs.fat_sectors) * SECTOR)
            img.write(fat)
        for c, chunk in fs.data.items():
            img.seek(base + (fs.data_start + (c - 2) * SPC) * SECTOR)
            img.write(chunk)
    used_mb = (fs.next_free - 2) * SPC * SECTOR / 1e6
    print("%s: %d MB disk, FAT32 partition, %.1f MB used" % (out, size_mb, used_mb))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    srcs = sys.argv[2:] or [repo]
    make(sys.argv[1], srcs, int(os.environ.get("DISK_MB", "128")))
