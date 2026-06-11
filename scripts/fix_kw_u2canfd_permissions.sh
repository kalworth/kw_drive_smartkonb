#!/usr/bin/env bash
set -euo pipefail

vendor="2e88"
product="4603"
found=0

sudo modprobe cdc_acm 2>/dev/null || true

for dev in /sys/bus/usb/devices/*; do
  [[ -f "$dev/idVendor" && -f "$dev/idProduct" ]] || continue
  [[ "$(<"$dev/idVendor")" == "$vendor" ]] || continue
  [[ "$(<"$dev/idProduct")" == "$product" ]] || continue

  found=1
  bus="$(printf '%03d' "$(<"$dev/busnum")")"
  num="$(printf '%03d' "$(<"$dev/devnum")")"
  node="/dev/bus/usb/$bus/$num"

  sudo chmod a+rw "$node"
  ls -l "$node"
done

for tty in /dev/ttyACM*; do
  [[ -e "$tty" ]] || continue
  sudo chmod a+rw "$tty"
  ls -l "$tty"
done

if [[ "$found" == "0" ]]; then
  echo "KW U2CANFD device $vendor:$product not found in WSL." >&2
  exit 1
fi
