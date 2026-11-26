#!/usr/bin/env bash
# Configures a Raspberry Pi 5 for the hexapod's real-time control loop.
#
#   sudo bash scripts/setup_pi.sh       apply every setting (safe to run again)
#   bash scripts/setup_pi.sh --check    only show the current state, changes nothing
#
# What it changes, and why:
#   1. CPU governor -> performance, now and at every boot. The clock stays at full
#      speed, so the loop never waits for it to ramp up after sleeping.
#   2. Kernel command line (after a reboot): isolcpus keeps ordinary processes off the
#      loop's core, irqaffinity keeps hardware interrupts off it.
#      The original file is saved next to it as cmdline.txt.orig.
#   3. Real-time limits for your user (at next login): SCHED_FIFO up to priority 90 and
#      unlimited locked memory, so ros2_control's loop runs in real time without sudo.
#   4. dialout group for your user (at next login), to open the servo adapter's port.
# Written for Ubuntu 24.04 on the Pi; ROS 2 itself is installed by setup_ros.sh.
# How to undo each step: see docs/pages/software/raspberry-pi.md.
set -euo pipefail

LOOP_CORE=3       # reserved for the control loop (h3215_bringup's controllers.yaml)
OTHER_CORES=0-2   # everything else: OS, ROS 2, perception
RT_PRIORITY=90    # highest SCHED_FIFO priority your user may request

CMDLINE=/boot/firmware/cmdline.txt
[ -f "$CMDLINE" ] || CMDLINE=/boot/cmdline.txt   # older Raspberry Pi OS releases
RT_LIMITS=/etc/security/limits.d/hexapod-rt.conf


apply_governor() {
    echo performance | tee /sys/devices/system/cpu/cpufreq/policy*/scaling_governor > /dev/null
    # Some images set their own governor at boot: this service runs after it.
    cat > /etc/systemd/system/cpu-performance.service <<'EOF'
[Unit]
Description=CPU governor: performance (hexapod control loop)
After=raspi-config.service

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo performance | tee /sys/devices/system/cpu/cpufreq/policy*/scaling_governor'

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable --quiet cpu-performance.service
}

apply_cmdline() {
    [ -f "$CMDLINE.orig" ] || cp "$CMDLINE" "$CMDLINE.orig"
    # The file is a single line: remove any old values, then append ours at the end.
    sed -i -E \
        -e 's/ *(isolcpus|irqaffinity)=[^ ]*//g' \
        -e 's/^ +//' \
        -e "1s/\$/ isolcpus=$LOOP_CORE irqaffinity=$OTHER_CORES/" \
        "$CMDLINE"
}

apply_rt_limit() {
    printf '%s - rtprio %s\n%s - memlock unlimited\n' "$SUDO_USER" "$RT_PRIORITY" "$SUDO_USER" > "$RT_LIMITS"
}

apply_serial_access() {
    usermod -aG dialout "$SUDO_USER"
}

check() {
    echo "model:          $(tr -d '\0' 2> /dev/null < /proc/device-tree/model || echo unknown)"
    echo "governor:       $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2> /dev/null)"
    echo "isolated cores: $(cat /sys/devices/system/cpu/isolated)   (empty until you reboot)"
    echo "boot cmdline:   $(grep -oE '(isolcpus|irqaffinity)=[^ ]*' "$CMDLINE" | tr '\n' ' ')"
    echo "rt limit file:  $(tr '\n' ';' < "$RT_LIMITS" 2> /dev/null || echo missing)"
    echo "your rtprio:    $(ulimit -r)   (needs a new login; under sudo it shows root's)"
    echo "your memlock:   $(ulimit -l)"
    echo "groups:         $(id -nG "${SUDO_USER:-$USER}")"
    if command -v vcgencmd > /dev/null; then
        echo "temperature:    $(vcgencmd measure_temp)"
        echo "throttling:     $(vcgencmd get_throttled)   (0x0 = never throttled since boot)"
    fi
}


if [ "${1:-}" = "--check" ]; then
    check
    exit 0
fi
if [ "$(id -u)" -ne 0 ] || [ -z "${SUDO_USER:-}" ]; then
    echo "Run it with sudo from your normal user: sudo bash $0" >&2
    exit 1
fi
grep -q "Raspberry Pi 5" /proc/device-tree/model 2> /dev/null ||
    echo "Warning: not a Raspberry Pi 5, check that the core numbers above fit this CPU." >&2

echo "1/4 CPU governor -> performance";                          apply_governor
echo "2/4 Kernel: isolcpus=$LOOP_CORE irqaffinity=$OTHER_CORES"; apply_cmdline
echo "3/4 Real-time limits for $SUDO_USER";                      apply_rt_limit
echo "4/4 Serial port access for $SUDO_USER";                    apply_serial_access
echo
check
echo
echo "Done. Reboot to apply the kernel settings: sudo reboot"
