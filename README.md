# Kernel Driver for AR0822

The AR0822 is a high-resolution CMOS image sensor that supports up to 4K video capture. This driver enables the sensor to work with the Raspberry Pi's MIPI CSI interface, supporting both 2-lane and 4-lane configurations.

This guide provides detailed instructions on how to install the AR0822 kernel driver on a Linux system, specifically Raspbian.

## Prerequisites

Before you begin the installation process, please ensure the following prerequisites are met:

- **Kernel version**: You should be running on a Linux kernel version 6.1 or newer. You can verify your kernel version by executing `uname -r` in your terminal.

- **Development tools**: Essential tools such as `gcc`, `dkms`, and `linux-headers` are required for compiling a kernel module. If not already installed, these can be installed using the package manager with the following command:
  
   ```bash 
   sudo apt install linux-headers dkms git
   ```
   
## Installation Steps

### Fetching the Source Code

Clone the repository to your local machine and navigate to the cloned directory:

```bash
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

#### Install rpicam-apps Dependencies
```bash
sudo apt install libavcodec-dev libavdevice-dev -y
```

#### Clone the rpicam-apps Repository
Download a local copy of Raspberry Pi’s `rpicam-apps` GitHub repository:
```bash
cd ~
git clone https://github.com/raspberrypi/rpicam-apps.git
cd rpicam-apps
```

#### Configure the rpicam-apps Build
Run the following `meson` command to configure the build:
```bash
meson setup build -Denable_libav=enabled -Denable_drm=enabled -Denable_egl=enabled -Denable_qt=enabled -Denable_opencv=disabled -Denable_tflite=disabled -Denable_hailo=disabled
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
rpicam-apps build: v1.6.0 000000000000-invalid 08-05-2025 (16:08:14)
rpicam-apps capabilites: egl:1 qt:1 drm:1 libav:1
libcamera build: v0.4.0
```

### Verify that ar0822 is Being Detected Correctly
Run the following command to list available cameras:
```bash
rpicam-hello --list-cameras
```

You should see output similar to this (depending on your link-frequency and lane amount selection):
```
Available cameras
-----------------
0 : ar0822 [3840x2160 12-bit GRBG] (/base/soc/i2c0mux/i2c@0/ar0822@10)
    Modes: 'SGRBG10_CSI2P' : 1920x1080 [60.08 fps - (0, 0)/1920x1080 crop]
                             3840x2160 [41.11 fps - (0, 0)/3840x2160 crop]
           'SGRBG12_CSI2P' : 1920x1080 [60.18 fps - (0, 0)/1920x1080 crop]
                             3840x2160 [34.79 fps - (0, 0)/3840x2160 crop]
```

## Special Thanks

Special thanks to:
- [Will Whang](https://github.com/will127534) for [imx585-v4l2-driver](https://github.com/will127534/imx585-v4l2-driver) repository which was used as the basis for structuring this driver.
- Sasha Shturma's Raspberry Pi CM4 Сarrier with Hi-Res MIPI Display project, the install script is adapted from the github project page: https://github.com/renetec-io/cm4-panel-jdi-lt070me05000
