# Kernel Driver for AR0822

[![code formatting](https://github.com/Kurokesu/ar0822-v4l2-driver/actions/workflows/clang-format.yml/badge.svg)](https://github.com/Kurokesu/ar0822-v4l2-driver/actions/workflows/clang-format.yml)
[![Raspberry Pi OS Bookworm](https://img.shields.io/badge/Raspberry_Pi_OS-Bookworm-blue?logo=raspberrypi)](https://www.debian.org/releases/bookworm/)
[![Raspberry Pi OS Trixie](https://img.shields.io/badge/Raspberry_Pi_OS-Trixie-blue?logo=raspberrypi)](https://www.debian.org/releases/trixie/)

Raspberry Pi kernel driver for the Onsemi AR0822 — an 8MP rolling shutter 1/1.8" back side illuminated CMOS sensor.

- 2-lane and 4-lane MIPI CSI-2 (up to 960 Mbps/lane)
- 10-bit and 12-bit RAW output
- 3840×2160 @ 40 fps (full resolution)
- 1920×1080 @ 120 fps (2×2 binning)

> [!NOTE]
> This driver supports an experimental eHDR mode, modeled after the IMX708
> implementation, by exposing the standard `V4L2_CID_WIDE_DYNAMIC_RANGE` control.
> Read more about it in [eHDR (experimental)](#ehdr-experimental).

## Prerequisites

**Kernel version**: You should be running on a Linux kernel version 6.1 or newer. You can verify your kernel version by executing `uname -r` in your terminal.
   
## Installation Steps

### Development Tools

Required tools: `git`, `dkms`. If not already installed, install with:

```bash
sudo apt install -y git
sudo apt install -y --no-install-recommends dkms
```

### Fetching the Source Code

Clone the repository to your local machine and navigate to the cloned directory:

```bash
cd ~
git clone https://github.com/Kurokesu/ar0822-v4l2-driver.git
cd ar0822-v4l2-driver/
```

### Compiling and Installing the Kernel Driver

To compile and install the kernel driver, execute the provided installation script:

```bash
sudo ./setup.sh
```

### Updating the Boot Configuration

Edit the boot configuration file using the following command:

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

Save the file and exit the editor.

Remember to reboot your system for the changes to take effect after editing `config.txt`.

> [!IMPORTANT]
> The stock `libcamera` does not support the AR0822 sensor — you must build a patched version for the camera to function properly. See [libcamera](#libcamera) below.

## dtoverlay options

The `ar0822` overlay supports comma-separated options to override defaults:

| option | description | default |
|--------|-------------|----------|
| `cam0` | Use cam0 port instead of cam1 | cam1 |
| `4lane` | Enable 4-lane MIPI CSI support | 2-lane |

### cam0

If the camera is connected to the cam0 port, append `,cam0`:

```ini
dtoverlay=ar0822,cam0
```

### 4lane

To enable 4-lane MIPI CSI-2, append `,4lane`:

```ini
dtoverlay=ar0822,4lane
```

> [!WARNING]
> Before using `4lane`, confirm your camera port actually supports 4 lanes. Not all Raspberry Pi models and carrier boards provide 4-lane CSI on both ports.


> [!TIP]
> You can combine options. Example `cam0 + 4 lanes`:
> ```ini
> dtoverlay=ar0822,cam0,4lane
> ```

## libcamera

Currently, the main `libcamera` repository does not support the `ar0822` sensor. To enable support, a fork has been created with the necessary modifications.

On Raspberry Pi devices, `libcamera` and `rpicam-apps` must be rebuilt together. Detailed instructions can be found [here](https://www.raspberrypi.com/documentation/computers/camera_software.html#advanced-rpicam-apps), but for convenience, this is the shorter version:

### Build libcamera and rpicam-apps

#### Remove Pre-installed rpicam-apps

```bash
sudo apt remove --purge rpicam-apps
```

#### Install libcamera Dependencies

```bash
sudo apt install -y libboost-dev
sudo apt install -y libgnutls28-dev openssl libtiff5-dev pybind11-dev
sudo apt install -y qtbase5-dev libqt5core5a libqt5gui5 libqt5widgets5
sudo apt install -y meson cmake
sudo apt install -y python3-yaml python3-ply
sudo apt install -y libglib2.0-dev libgstreamer-plugins-base1.0-dev
```

#### Clone the Forked libcamera Repository

Clone Kurokesu's fork of `libcamera` with `ar0822` modifications:

```bash
cd ~
git clone https://github.com/Kurokesu/libcamera.git --branch ar0822
cd libcamera/
```

#### Configure the Build Environment

Run `meson` to configure the build environment:

```bash
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
```

#### Build and Install libcamera

Build `libcamera`:

```bash
ninja -C build
```

Then install it:

```bash
sudo ninja -C build install
```

> [!TIP]
> On devices with 1GB of memory or less, the build may exceed available memory. Append the `-j 1` flag to meson commands to limit the build to a single process.

> [!WARNING]
> `libcamera` does not yet have a stable binary interface. Always build `rpicam-apps` after you build `libcamera`.

#### Install rpicam-apps Dependencies

```bash
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y libavcodec-dev libavdevice-dev libavformat-dev libswresample-dev
sudo apt install -y libepoxy-dev libpng-dev
```

#### Clone the rpicam-apps Repository

Clone Kurokesu’s `rpicam-apps` fork, which contains HDR modifications:

```bash
cd ~
git clone https://github.com/Kurokesu/rpicam-apps.git --branch hdr-ar0822
cd rpicam-apps
```

#### Configure the rpicam-apps Build

Run the following `meson` command to configure the build (libav enabled by default):

```bash
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
```

> [!IMPORTANT]
> `-Denable_libav` enables optional video encode/decode support (FFmpeg / `libavcodec`).
> 
> - **Debian Bookworm**: the packaged `libav*` version is **too old** to build `rpicam-apps` releases **newer than v1.9.0** with libav enabled.
>   - On Bookworm this typically shows up as build errors like “libavcodec API version is too old”, because Bookworm ships `libavcodec` **59.x** while newer `rpicam-apps` expects **libavcodec >= 60** (see [Raspberry Pi forum thread](https://forums.raspberrypi.com/viewtopic.php?t=392649)).
>   - If you want libav support on Bookworm, check out the `rpicam-apps` **v1.9.0** before running `meson setup`:
> 
>     ```bash
>     git checkout v1.9.0
>     ```
>   - If you are building **`rpicam-apps` > v1.9.0** on Bookworm, you must disable libav:
>
>     ```bash
>     meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
>     ```
> - **Debian Trixie**: build `rpicam-apps` as usual with `-Denable_libav=enabled` (no need to check out an older version).

#### Build rpicam-apps

Run the following command to build:

```bash
meson compile -C build
```

#### Install rpicam-apps

Run the following command to install `rpicam-apps`:

```bash
sudo meson install -C build
```

> [!TIP]  
> The command above should automatically update the `ldconfig` cache. If you have trouble accessing your new `rpicam-apps` build, run the following command to update the cache:
> 
> ```bash
> sudo ldconfig
> ```

#### Verify the rpicam-apps Build

Verify that `rpicam-apps` was rebuilt correctly by checking the version:

```bash
rpicam-hello --version
```

You should get output similar to this, with your build date:

```
rpicam-apps build: v1.10.0 19-11-2025 (12:25:28)
rpicam-apps capabilites: egl:1 qt:1 drm:1 libav:1
libcamera build: v0.5.2
```

### Verify that `ar0822` is detected

Do not forget to reboot!

```bash
sudo reboot
```

Run the following command to list available cameras:

```bash
rpicam-hello --list-cameras
```

You should see output similar to this (depending on your link-frequency and lane count):

```
Available cameras
-----------------
0 : ar0822 [3840x2160 12-bit GRBG] (/base/axi/pcie@1000120000/rp1/i2c@88000/ar0822@10)
    Modes: 'SGRBG10_CSI2P' : 1920x1080 [120.15 fps - (0, 0)/3840x2160 crop]
                             3840x2160 [40.03 fps - (0, 0)/3840x2160 crop]
           'SGRBG12_CSI2P' : 1920x1080 [120.21 fps - (0, 0)/3840x2160 crop]
                             3840x2160 [33.89 fps - (0, 0)/3840x2160 crop]
```

## eHDR (experimental)

AR0822 features an on‑sensor HDR mode that expands dynamic range up to 120 dB by combining three exposures within the sensor using the MEC algorithm. To reduce bandwidth requirements even further, the linearized 20‑bit HDR signal is companded to a 12‑bit output.

> [!IMPORTANT] 
> libcamera pipeline is designed to work with linear image data from sensor,
> so while Kurokesu's libcamera fork HDR implementation is in experimental
> stage, companded data may have some color shifts due to compression.

Because of the way exposure range limitations work in sensor, running at maximum FPS with current PIXCLK configuration will reduce maximum exposure drastically.

For instance, running 4k @ 30fps results in maximum exposure T1 ≈ 10.26ms, while running 4k @ 28.8fps results in T1 ≈ 30.4ms (right at the internal delay buffer limit).

Consider reducing framerate slightly when larger exposure range is desired, this will be addressed in future revisions of the driver.

eHDR mode is enabled by appending `--hdr` to `rpicam` commands.

### List supported eHDR mode formats

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

## Special Thanks

Special thanks to:

- [Will Whang](https://github.com/will127534) for [imx585-v4l2-driver](https://github.com/will127534/imx585-v4l2-driver) repository which was used as the basis for structuring this driver.
- Sasha Shturma's Raspberry Pi CM4 carrier with Hi-Res MIPI Display project. The install script is adapted from [cm4-panel-jdi-lt070me05000](https://github.com/renetec-io/cm4-panel-jdi-lt070me05000).
