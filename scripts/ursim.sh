#!/usr/bin/env bash
# URSim bring-up/tear-down for the M1 test rig.
#
#   scripts/ursim.sh up      start container, wait for dashboard, power on
#   scripts/ursim.sh down    stop + remove container (image kept)
#   scripts/ursim.sh status  robot/safety mode via dashboard
#
# Ports: 30001-30004 (URControl/RTDE), 29999 (dashboard), 50001-50003
# (urcl reverse interface), 6080 (noVNC web UI: http://localhost:6080).
#
# Note for Apple-silicon hosts: the image is amd64-only; PolyScope boots
# under qemu emulation and can take 15-40 min before the dashboard (and the
# RTDE safety-confirmation it performs) is available. URControl itself is up
# within ~1 min — RTDE answers, but sends no data until PolyScope confirms
# the safety setup. On an x86 Linux host the whole thing is up in <1 min.
set -euo pipefail

IMG=universalrobots/ursim_e-series
NAME=ursim

dash() { python3 - "$@" <<'EOF'
import socket, sys, time
cmds = sys.argv[1:]
s = socket.create_connection(("127.0.0.1", 29999), timeout=10); s.settimeout(10)
print(s.recv(4096).decode(errors="replace").strip())
for c in cmds:
    s.sendall((c + "\n").encode()); time.sleep(2)
    try: print(c, "->", s.recv(4096).decode(errors="replace").strip())
    except socket.timeout: print(c, "-> <timeout>")
s.close()
EOF
}

case "${1:-}" in
  up)
    docker rm -f "$NAME" 2>/dev/null || true
    docker run -d --name "$NAME" -e ROBOT_MODEL=UR10 \
      -p 30001-30004:30001-30004 -p 29999:29999 -p 50001-50003:50001-50003 \
      -p 6080:6080 "$IMG"
    echo "waiting for dashboard (PolyScope) ..."
    for _ in $(seq 1 120); do
      if banner=$(dash 2>/dev/null | head -1) && [ -n "$banner" ]; then
        echo "dashboard: $banner"
        dash "power on" "brake release" "robotmode" "close safety popup" || true
        exit 0
      fi
      sleep 30
    done
    echo "dashboard did not come up — check: docker logs $NAME / http://localhost:6080" >&2
    exit 1
    ;;
  down)   docker rm -f "$NAME" ;;
  status) dash "robotmode" "safetystatus" ;;
  *) echo "usage: $0 up|down|status" >&2; exit 2 ;;
esac
