# Raspberry Pi setup

The control loop runs on one CPU core reserved for it, with real-time priority (see [ROS 2](ros.md)). Two scripts prepare the Pi: `setup_pi.sh` configures the system, `setup_ros.sh` installs ROS 2 and builds the robot's packages. Everything that runs on the Pi is a ROS package, C++ or Python, so there is no other environment to set up.

Needs **Ubuntu Server 24.04 (64-bit)**, the system ROS 2 Jazzy has packages for. Write it to the SD card with Raspberry Pi Imager (*Other general-purpose OS → Ubuntu*).

## Setup

From the repository folder on the Pi:

1. Configure the system, then reboot:
   ```bash
   sudo bash scripts/setup_pi.sh
   sudo reboot
   ```
2. Check that it took effect: governor `performance`, isolated core `3`, rtprio `90`, memlock `unlimited`, `dialout` in your groups.
   ```bash
   bash scripts/setup_pi.sh --check
   ```
3. Install ROS 2 Jazzy and build the workspace, `~/h3215_ws`, with this repository linked into it (it asks for sudo for the system packages):
   ```bash
   bash scripts/setup_ros.sh
   ```
4. In every new shell (or add it to `~/.bashrc`):
   ```bash
   source ~/h3215_ws/install/setup.bash
   ```
5. Set up the servos (see [Servos](servos.md)), then start the robot (see [ROS 2](ros.md#running)):
   ```bash
   ros2 launch h3215_bringup robot.launch.py
   ```

After pulling changes, rebuild with `bash scripts/setup_ros.sh --build`. The workspace is built with `--symlink-install`, so changes to Python nodes, launch files and configuration apply without a rebuild; C++ changes and new files need one.

## What `setup_pi.sh` changes

| Setting | Why |
|---|---|
| CPU governor `performance`, also at boot | No frequency ramp-up delay when the loop wakes |
| `isolcpus=3 irqaffinity=0-2` in `cmdline.txt` | Core 3 runs only the control loop; interrupts go to the other cores |
| `rtprio 90` and `memlock unlimited` for your user | The controller manager can use `SCHED_FIFO` and lock its memory without sudo |
| `dialout` group | Access to the servo adapter's serial port |

It backs up `cmdline.txt` once, and running it again changes nothing.

## Dependencies

Every dependency of the Pi's packages is listed in their `package.xml` files, and `setup_ros.sh` installs them with rosdep from Ubuntu's and ROS's apt repositories, at the versions of the Ubuntu and ROS releases. To add one, add it to the `package.xml` of the package that needs it, with its [rosdep key](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html), and run `setup_ros.sh` again.

## Undoing the setup

```bash
sudo cp /boot/firmware/cmdline.txt.orig /boot/firmware/cmdline.txt   # kernel settings
sudo systemctl disable cpu-performance.service                       # governor at boot
sudo rm /etc/security/limits.d/hexapod-rt.conf                       # real-time limits
```

Reboot afterwards.

!!! tip "If the Pi doesn't boot after a kernel change"
    Put the SD card in a PC: `cmdline.txt.orig` is on the boot partition, which Windows can read. Copy it over `cmdline.txt`.
