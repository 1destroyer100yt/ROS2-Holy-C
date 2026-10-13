#!/bin/bash
# The RoboClaw driver and DiffDrive (src/Hw) against a RoboClaw simulator
# that the official Basicmicro library is checked against first.
#   tools/hwtest/run.sh        over a pty (any Linux)
#   tools/hwtest/run.sh com    over the 16550 UART (COM2) of a QEMU guest:
#                              the HolyC drives the UART's I/O ports itself
#                              (needs root, qemu-system-x86_64 = $QEMU, a
#                              kernel = $VMLINUZ or apt-get download)
# needs: pip install basicmicro pyserial   (PYTHON=... to pick a python)
cd "$(dirname "$0")/../.."
PY=${PYTHON:-python3}
if ! $PY -c "import basicmicro" 2>/dev/null; then echo "hwtest: basicmicro not installed for $PY, skipping"; exit 0; fi
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
$PY tools/hwtest/roboclaw_sim.py --selfcheck || { echo "hwtest: FAIL (simulator vs the official library)"; exit 1; }
if [ "$1" = "com" ]; then
  source tools/guest/lib.sh
  guest_ok || { echo "hwtest com: needs root and $QEMU, skipping"; exit 0; }
  guest_kernel "$tmp" || { echo "hwtest com: no kernel, skipping"; exit 0; }
  guest_initrd tests/hw/RoboClaw.HC HwRoboClawCom "$tmp" || exit 1
  "$QEMU" -m 256 -display none -no-reboot -serial file:"$tmp/console.txt" \
    -chardev socket,id=c2,path="$tmp/com2.sock",server=on,wait=on -serial chardev:c2 \
    -kernel "$VMLINUZ" -initrd "$tmp/initrd.cpio" -append "console=ttyS0 panic=-1 quiet" &
  qemu=$!
  $PY tools/hwtest/roboclaw_sim.py --unix "$tmp/com2.sock" --seconds 90 > "$tmp/sim.txt" &
  sim=$!
  for i in $(seq 1 900); do kill -0 $qemu 2>/dev/null || break; sleep 0.1; done
  kill $qemu 2>/dev/null; wait $sim; wait 2>/dev/null
  out="$tmp/console.txt"
else
  $PY tools/hwtest/roboclaw_sim.py --pty "$tmp/tty" --seconds 30 > "$tmp/sim.txt" &
  sim=$!
  for i in $(seq 1 100); do [ -s "$tmp/tty" ] && break; sleep 0.1; done
  ROBOCLAW_TTY=$(cat "$tmp/tty") python3 tools/hctest/hc2c.py run tests/hw/RoboClaw.HC \
    --main HwRoboClawPty > "$tmp/holyc.txt" 2>&1
  kill -INT $sim 2>/dev/null; wait $sim 2>/dev/null
  out="$tmp/holyc.txt"
fi
grep -a RESULT "$out" "$tmp/sim.txt" | sed 's/^[^:]*://'
grep -aq "hw_errors=0 .*-> 1" "$out" && echo "hwtest ${1:-pty}: OK" || { tail -5 "$out"; echo "hwtest ${1:-pty}: FAIL"; exit 1; }
