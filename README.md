# Kernel Driver for AR0822

The AR0822 is a high-resolution CMOS image sensor that supports up to 4K video capture. This driver enables the sensor to work with the Raspberry Pi's MIPI CSI interface, supporting both 2-lane and 4-lane configurations.

This guide provides detailed instructions on how to install the AR0822 kernel driver on a Linux system, specifically Raspbian.

> [!NOTE]
> This driver supports an experimental eHDR mode, modeled after the IMX708
> implementation, by exposing the standard `V4L2_CID_WIDE_DYNAMIC_RANGE` control.
> Read more about it in [eHDR (experimental)](#ehdr-experimental).

## Prerequisites

**Kernel version**: You should be running on a Linux kernel version 6.1 or newer. You can verify your kernel version by executing `uname -r` in your terminal.
   
## Installation Steps

### Development tools

Required tools: `gcc`, `dkms`, `linux-headers`. If not already installed, install with:

```bash 
sudo apt install -y linux-headers dkms git
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

In the opened editor, locate the line containing `camera_auto_detect` and change its value to `0`. Then, add the line `dtoverlay=ar0822`. So, it will look like this:

```
camera_auto_detect=0
dtoverlay=ar0822
```

After making these changes, save the file and exit the editor.

Remember to reboot your system for the changes to take effect.

## dtoverlay options

The driver supports several configuration options that can be combined as needed:

| Option | Description | Default |
|--------|-------------|----------|
| `cam0` | Use cam0 port instead of cam1 | cam1 |
| `4lane` | Enable 4-lane MIPI CSI support | 2-lane |

### cam0

By default, the driver uses cam1 port. If the camera is attached to cam0 port instead, append the dtoverlay with `,cam0` to override the default:

```
camera_auto_detect=0
dtoverlay=ar0822,cam0
```

### 4-lane support

To enable 4-lane MIPI CSI support, append the dtoverlay with `,4lane` like this:

```
camera_auto_detect=0
dtoverlay=ar0822,4lane
```

You can also combine options, for example to use cam0 with 4-lane support:

```
camera_auto_detect=0
dtoverlay=ar0822,cam0,4lane
```

> [!WARNING]
> Before using the 4-lane option, double-check that your selected camera port (cam0 or cam1) actually has 4 lanes wired on your Raspberry Pi and carrier board combination. Not all carrier boards support 4-lane MIPI CSI on both ports.

## libcamera Support

Currently, the main `libcamera` repository does not support the `ar0822` sensor. To enable support, a fork has been created with the necessary modifications.

On Raspberry Pi devices, `libcamera` and `rpicam-apps` must be rebuilt together. Detailed instructions can be found [here](https://www.raspberrypi.com/documentation/computers/camera_software.html#advanced-rpicam-apps), but for convenience, this is the shorter version:

### Build libcamera and rpicam-apps

#### Remove Pre-installed rpicam-apps

```bash
sudo apt remove --purge rpicam-apps
```

#### Install rpicam-apps Dependencies

```bash
sudo apt install -y libepoxy-dev libjpeg-dev libtiff5-dev libpng-dev
```

```bash
sudo apt install -y libavcodec-dev libavdevice-dev
```

```bash
sudo apt install -y cmake libboost-program-options-dev libdrm-dev libexif-dev
sudo apt install -y meson ninja-build
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

Download a local copy of Kurokesu's fork of `libcamera` with `ar0822` modifications from GitHub:

```bash
cd ~
git clone --single-branch --branch ar0822 https://github.com/Kurokesu/libcamera.git
cd libcamera/
```

#### Configure the Build Environment

Run `meson` to configure the build environment:

```bash
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=enabled -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=disabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
```

#### Build and Install libcamera

Finally, run the following command to build and install `libcamera`:

```bash
sudo ninja -C build install
```

> [!TIP]
> On devices with 1GB of memory or less, the build may exceed available memory. Append the `-j 1` flag to meson commands to limit the build to a single process.

> [!WARNING]
> `libcamera` does not yet have a stable binary interface. Always build `rpicam-apps` after you build `libcamera`.

#### Clone the rpicam-apps Repository

Download a local copy of Kurokesu’s `rpicam-apps` fork, which contains HDR modifications:

```bash
cd ~
git clone https://github.com/Kurokesu/rpicam-apps.git --branch hdr-ar0822
cd rpicam-apps
```

#### Configure the rpicam-apps Build

Run the following `meson` command to configure the build:

```bash
meson setup build -Denable_libav=disabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
```

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

Verify that `rpicam-apps` has been rebuilt correctly by checking the version:

```bash
rpicam-hello --version
```

You should get output similar to this, with your build date:

```
rpicam-apps build: v1.10.0 19-11-2025 (12:25:28)
rpicam-apps capabilites: egl:1 qt:1 drm:1 libav:1
libcamera build: v0.5.2
```

### Verify that ar0822 is Being Detected Correctly

Do not forget to reboot!

```bash
sudo reboot
```

Run the following command to list available cameras:

```bash
rpicam-hello --list-cameras
```

You should see output similar to this (depending on your link-frequency and lane amount selection):

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
- Sasha Shturma's Raspberry Pi CM4 Сarrier with Hi-Res MIPI Display project, the install script is adapted from the github project page: https://github.com/renetec-io/cm4-panel-jdi-lt070me05000
