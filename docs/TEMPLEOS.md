# Running it in TempleOS

## 1. Get the files in
TempleOS reads FAT32 disks, so the easiest way in is a disk image with the repo on it:
```
python3 tools/mkdisk/mkdisk.py ros2holyc.img          # 128 MB, FAT32, the repo in /ROS2-Holy-C
```
It's plain Python (no tools to install) and keeps the long file names. `tools/check.sh`
reads every file back with an independent FAT implementation (pyfatfs) to make sure.

Attach it next to TempleOS in QEMU:
```
qemu-system-x86_64 -m 1024 -smp 2 -enable-kvm \
  -drive file=TempleOS.qcow2,if=ide,index=0 \
  -drive file=ros2holyc.img,format=raw,if=ide,index=1 \
  -netdev user,id=n0 -device e1000,netdev=n0
```
(or add it as a second IDE disk in VirtualBox). In TempleOS, mount it and copy the
folder home (VERIFY: the drive letter TempleOS gives it):
```
Mount;                                            //answer the questions: the 2nd IDE disk
CopyTree("D:/ROS2-Holy-C","C:/Home/ROS2-Holy-C");
```

## 2. Load and test
```
#include "~/ROS2-Holy-C/Load.HC"
#include "~/ROS2-Holy-C/tests/Run.HC"
RosTestsRun;
```
Then try things from [TOOLS.md](TOOLS.md): `TurtleSim;`, `RvizLite("map");`, the CLI functions.

## 3. Network
With QEMU's e1000 (above) and a DHCP server on the network (QEMU's user networking has one):
```
RosNetworkDhcp(0);      //address from DHCP, DDS on domain 0
```
or a fixed address: `RosNetwork("10.0.2.15","255.255.255.0","10.0.2.2",0);`.
For ROS 2 on the Linux side to see TempleOS, use a TAP or bridged network instead of user
networking (user networking doesn't pass multicast); see [NETWORK.md](NETWORK.md).

## What hasn't been checked on a real TempleOS yet
Everything here is compiled and tested on Linux through `tools/hctest` (HolyC translated to
C), plus a linter for HolyC's differences. The real HolyC compiler hasn't seen it yet. Lines
marked `VERIFY` in the source are the TempleOS-only parts (drawing, keyboard, disk, PCI,
the e1000 driver, Spawn on other cores). `tools/templeos/vm.py` drives TempleOS in QEMU
with no screen (screenshots, typing) for testing it automatically once a TempleOS ISO is
available.
