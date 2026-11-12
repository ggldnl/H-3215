# Raspberry Pi setup

The control loop runs at 50 Hz on one CPU core reserved for it, with real-time priority. Two scripts prepare the Pi: `setup_pi.sh` configures the system, `setup_env.sh` installs the Python environment.

Needs Raspberry Pi OS **64-bit** (conda has no 32-bit ARM builds).

## Setup

From the repository folder on the Pi:

1. Configure the system, then reboot:
   ```bash
   sudo bash scripts/setup_pi.sh
   sudo reboot
   ```
2. Check that it took effect: governor `performance`, isolated core `3`, rtprio `90`, `dialout` in your groups.
   ```bash
   bash scripts/setup_pi.sh --check
   ```
3. Create the Python environment, then open a new shell:
   ```bash
   bash scripts/setup_env.sh
   ```
4. In every new shell:
   ```bash
   conda activate hexapod
   ```

The first `setup_env.sh` run on a Pi writes `hexapod/conda-Linux-aarch64.lock`. Commit it: every later run, on any Pi, installs exactly the same packages.

## What `setup_pi.sh` changes

| Setting | Why |
|---|---|
| CPU governor `performance`, also at boot | No frequency ramp-up delay when the loop wakes |
| `isolcpus=3 irqaffinity=0-2` in `cmdline.txt` | Core 3 runs only the control loop; interrupts go to the other cores |
| `rtprio 90` for your user | The loop can use `SCHED_FIFO` without sudo |
| `dialout` group | Access to the servo adapter's serial port |

It backs up `cmdline.txt` once, and running it again changes nothing.

## Changing the environment

Edit `hexapod/environment.yml`, run `bash scripts/setup_env.sh --relock`, then commit `environment.yml` and the lock file. Other machines run `bash scripts/setup_env.sh` and get the same packages.

## Undoing the setup

```bash
sudo cp /boot/firmware/cmdline.txt.orig /boot/firmware/cmdline.txt   # kernel settings
sudo systemctl disable cpu-performance.service                       # governor at boot
sudo rm /etc/security/limits.d/hexapod-rt.conf                       # real-time limit
```

Reboot afterwards.

!!! tip "If the Pi doesn't boot after a kernel change"
    Put the SD card in a PC: `cmdline.txt.orig` is on the boot partition, which Windows can read. Copy it over `cmdline.txt`.
