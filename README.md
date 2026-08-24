# AR0822 kernel driver for Raspberry Pi

[![Build](https://github.com/Kurokesu/ar0822-rpi-driver/actions/workflows/build.yml/badge.svg)](https://github.com/Kurokesu/ar0822-rpi-driver/actions/workflows/build.yml)
[![Code style](https://github.com/Kurokesu/ar0822-rpi-driver/actions/workflows/code-style.yml/badge.svg)](https://github.com/Kurokesu/ar0822-rpi-driver/actions/workflows/code-style.yml)
[![Release](https://img.shields.io/github/v/release/Kurokesu/ar0822-rpi-driver)](https://github.com/Kurokesu/ar0822-rpi-driver/releases/latest)
[![Kurokesu apt archive](https://img.shields.io/badge/apt-apt.kurokesu.com-D70A53?logo=debian)](https://apt.kurokesu.com)
[![RPi OS Bookworm | Trixie](https://img.shields.io/badge/RPi_OS-Bookworm_%7C_Trixie-blue?logo=raspberrypi)](https://www.raspberrypi.com/software/operating-systems/)
[![Kernel 6.12+](https://img.shields.io/badge/kernel-6.12%2B-blue?logo=raspberrypi)](https://github.com/raspberrypi/linux/tree/rpi-6.12.y)

Raspberry Pi kernel driver for Onsemi AR0822, an 8 MP rolling shutter 1/1.8" back side illuminated CMOS sensor.

- 2-lane and 4-lane MIPI CSI-2 (up to 960 Mbps/lane)
- 10-bit and 12-bit RAW output
- 3840×2160 @ 40 fps (full resolution)
- 1920×1080 @ 120 fps (2×2 binning)

![Kurokesu camera modules connected to a Raspberry Pi 5](https://raw.githubusercontent.com/Kurokesu/ar0822-rpi-driver/main/docs/kurokesu-on-pi.jpg)

*AR0822 camera modules are available at [kurokesu.com](https://www.kurokesu.com/item/822C-CSI)*

> [!NOTE]
> This driver supports an experimental eHDR mode, modeled after IMX708
> implementation, by exposing the standard `V4L2_CID_WIDE_DYNAMIC_RANGE` control.
> Read more in [eHDR (experimental)](#ehdr-experimental).

## Install

Connect camera to CSI port with Pi powered off.

Update OS and reboot:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

> [!IMPORTANT]
> If driver or camera stack was previously built from source, run one-time cleanup before first apt install. See [migrating from a source install](#migrating-from-a-source-install).

Enable Kurokesu apt archive (skip if already enabled):

```bash
curl -fsSLO https://apt.kurokesu.com/setup.sh
sudo sh setup.sh
```

Install driver and camera stack:

```bash
sudo apt update
sudo apt install -y ar0822-rpi-dkms rpicam-apps
```

*With archive enabled, apt resolves Kurokesu `rpicam-apps` and `libcamera` forks with AR0822 support as updates to stock packages. Later updates arrive with regular `apt upgrade`.*

Edit boot configuration:

```bash
sudo nano /boot/firmware/config.txt
```

Make two changes:

1. Find `camera_auto_detect` near the top and set it to `0`:

```ini
camera_auto_detect=0
```

2. Add `dtoverlay=ar0822` under the `[all]` section at the bottom of the file:

```ini
[all]
dtoverlay=ar0822
```

*If camera is connected to cam0 port, use `dtoverlay=ar0822,cam0` instead. See [cam0](#cam0).*

Save and exit.

`config.txt` changes take effect after reboot:

```bash
sudo reboot
```

Verify camera is detected:

```bash
rpicam-hello --list-cameras
```

Expected output (varies by lane configuration):

```
Available cameras
-----------------
0 : ar0822 [3840x2160 12-bit GRBG] (/base/axi/pcie@1000120000/rp1/i2c@88000/ar0822@10)
    Modes: 'SGRBG10_CSI2P' : 1920x1080 [120.15 fps - (0, 0)/3840x2160 crop]
                             3840x2160 [40.03 fps - (0, 0)/3840x2160 crop]
           'SGRBG12_CSI2P' : 1920x1080 [120.21 fps - (0, 0)/3840x2160 crop]
                             3840x2160 [33.89 fps - (0, 0)/3840x2160 crop]
```

Start live preview:

```bash
rpicam-hello -t 0
```

On headless systems, capture a still image instead:

```bash
rpicam-still -o test.jpg
```

## dtoverlay options

`ar0822` overlay supports comma-separated options to override defaults:

| option | description | default |
|--------|-------------|----------|
| `cam0` | Use cam0 port instead of cam1 | cam1 |
| `4lane` | Use 4-lane MIPI CSI-2 (if wired) | 2 lanes |

### cam0

If camera is connected to cam0 port, append `,cam0`:

```ini
dtoverlay=ar0822,cam0
```

### 4lane

To enable 4-lane MIPI CSI-2, append `,4lane`:

```ini
dtoverlay=ar0822,4lane
```

> [!WARNING]
> Before using `4lane`, confirm your camera port actually supports 4-lane MIPI CSI. Not all Raspberry Pi models and carrier boards provide 4-lane MIPI CSI on both ports.

> [!TIP]
> Options can be combined. Example (cam0, 4-lane):
> ```ini
> dtoverlay=ar0822,cam0,4lane
> ```

## eHDR (experimental)

AR0822 features an on-sensor HDR mode that expands dynamic range up to 120 dB by combining three exposures within sensor using the MEC algorithm. To reduce bandwidth requirements, linearized 20-bit HDR signal is companded to a 12-bit output.

> [!IMPORTANT]
> libcamera pipeline is designed for linear image data from sensor. Kurokesu's fork HDR implementation is experimental. Companded data may show color shifts due to compression.

Due to exposure range limitations, running at maximum fps with current PIXCLK configuration reduces maximum exposure drastically.

For instance, running 4K @ 30 fps results in maximum exposure T1 ≈ 10.26 ms, while running 4K @ 28.8 fps results in T1 ≈ 30.4 ms (right at internal delay buffer limit).

Consider reducing framerate slightly when larger exposure range is desired. This will be addressed in future driver revisions.

eHDR mode is enabled by appending `--hdr` to `rpicam` commands.

### List eHDR modes

```bash
rpicam-hello --list-cameras --hdr
```

```
Available cameras
-----------------
0 : ar0822 [3840x2160 12-bit GRBG] (/base/axi/pcie@1000120000/rp1/i2c@88000/ar0822@10)
    Modes: 'SGRBG12_CSI2P' : 1920x1080 [48.04 fps - (0, 0)/3840x2160 crop]
                             3840x2160 [30.01 fps - (0, 0)/3840x2160 crop]
```

## Build from source

Install required tools:

```bash
sudo apt install -y git
sudo apt install -y --no-install-recommends dkms
```

Clone this repository:

```bash
cd ~
git clone https://github.com/Kurokesu/ar0822-rpi-driver.git
cd ar0822-rpi-driver/
```

If driver was installed from apt archive previously, remove it first:

```bash
sudo apt remove ar0822-rpi-dkms
```

Run setup script:

```bash
sudo ./setup.sh
```

Camera stack, boot configuration and verification follow [Install](#install). Skip `ar0822-rpi-dkms` there, only `rpicam-apps` is needed. To build `libcamera` and `rpicam-apps` from source as well, see [libcamera/BUILDING.md](https://github.com/Kurokesu/libcamera/blob/kurokesu/BUILDING.md).

## Migrating from a source install

One-time cleanup before first apt install.

Remove `ar0822` driver modules installed by `setup.sh`:

```bash
dkms status | grep ar0822 | cut -d, -f1 | sort -u | xargs -rI{} sudo dkms remove {} --all
```

Source-built `libcamera` and `rpicam-apps` install to `/usr/local` and shadow packaged binaries. Remove them:

> [!WARNING]
> Command below deletes everything under `/usr/local` with `libcamera`, `rpicam` or `libpisp` in its name, including custom scripts or files named after them.

```bash
sudo find /usr/local -depth \( -name '*libcamera*' -o -name '*rpicam*' -o -name '*libpisp*' \) -exec rm -rf {} +
```

Cleanup complete. Continue with [install steps](#install).

## Special thanks

- [Will Whang](https://github.com/will127534) for [imx585-v4l2-driver](https://github.com/will127534/imx585-v4l2-driver), used as basis for structuring this driver.
- Sasha Shturma's Raspberry Pi CM4 carrier with Hi-Res MIPI Display project. Install script adapted from [cm4-panel-jdi-lt070me05000](https://github.com/renetec-io/cm4-panel-jdi-lt070me05000).
