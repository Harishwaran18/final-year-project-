#!/usr/bin/env bash
# Creates and brings up the virtual CAN interface the SIL gateway writes to.
#
#   ./scripts/setup_vcan.sh            # create vcan0
#   ./scripts/setup_vcan.sh vcan1      # create vcan1 instead
#   ./scripts/setup_vcan.sh --teardown # remove the interface
#
# Once vcan0 is up, watch the frames the gateway produces with:
#   candump vcan0

set -euo pipefail

TEARDOWN=0
IFACE="vcan0"
for arg in "$@"; do
  case "${arg}" in
    --teardown) TEARDOWN=1 ;;
    -h|--help)
      sed -n '2,9p' "$0"
      exit 0
      ;;
    -*) echo "unknown option: ${arg}" >&2; exit 2 ;;
    *) IFACE="${arg}" ;;
  esac
done

if [[ "$(id -u)" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

if [[ "${TEARDOWN}" -eq 1 ]]; then
  echo "==> Removing ${IFACE}"
  ${SUDO} ip link delete "${IFACE}" 2>/dev/null || echo "    ${IFACE} did not exist"
  exit 0
fi

echo "==> Loading the vcan kernel module"
${SUDO} modprobe vcan

if ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "==> ${IFACE} already exists"
else
  echo "==> Creating ${IFACE}"
  ${SUDO} ip link add dev "${IFACE}" type vcan
fi

echo "==> Bringing ${IFACE} up"
${SUDO} ip link set up "${IFACE}"

echo
ip -details link show "${IFACE}"
echo
echo "The interface is ready. Run the gateway with dry_run:=false, then:"
echo "  candump ${IFACE}"
