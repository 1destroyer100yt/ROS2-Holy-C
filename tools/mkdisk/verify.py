"""Check a mkdisk image with an independent FAT reader (pyfatfs):
every file of the source tree must read back byte for byte, with its
long name.   python3 verify.py out.img SRC_DIR    (pip install pyfatfs)
"""
import os
import sys

from pyfatfs.PyFatFS import PyFatFS

sys.path.insert(0, os.path.dirname(__file__))
from mkdisk import SKIP_DIRS, SKIP_EXT, PART_START, SECTOR  # noqa: E402

img, src = sys.argv[1], os.path.abspath(sys.argv[2])
top = os.path.basename(src)
fs = PyFatFS(img, offset=PART_START * SECTOR, read_only=True)
n, bad = 0, 0
for dirpath, dirnames, files in os.walk(src):
    dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS and not os.path.islink(os.path.join(dirpath, d))]
    rel = os.path.relpath(dirpath, src)
    for f in files:
        p = os.path.join(dirpath, f)
        if os.path.splitext(f)[1] in SKIP_EXT or os.path.islink(p):
            continue
        fat_path = "/" + top + ("" if rel == "." else "/" + rel) + "/" + f
        try:
            with fs.openbin(fat_path) as fh:
                same = fh.read() == open(p, "rb").read()
        except Exception as e:  # noqa: BLE001
            same = False
            print("missing:", fat_path, e)
        n += 1
        if not same:
            bad += 1
            print("differs:", fat_path)
fs.close()
print("verify: %d files, %d bad" % (n, bad))
sys.exit(1 if bad or not n else 0)
