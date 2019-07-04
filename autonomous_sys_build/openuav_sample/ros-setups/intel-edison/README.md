# Intel Edison Setup

# Disclaimer

We do not support this install script and are unable to respond to all queries. Installation is at your own risk and may involve unstable packages. Having said that though, if you figure out any bugs, enhancements or suggestions, you're encouraged to submit a pull request or contact us.

# Before you Start

Before starting with the installation it's a good idea to boot the Edison straight out of the box to make sure it's working. This way we can make sure we have a functional board before proceeding and we won't be mistakenly blaming setup issues if something is wrong here.

Connect one USB cable to the cosole port and then start your temrminal app (see next section for more information on this). Once you are connected plug in the second USB cable for power and after 15 seconds you should see the system booting. If you want to login the user name is root (no password).

# Flash Ubilinux

To flash Ubilinux carefully follow the instruction here http://www.emutexlabs.com/ubilinux/29-ubilinux/218-ubilinux-installation-instructions-for-intel-edison

The Ubilinux image can be downloaded from this page: http://www.emutexlabs.com/ubilinux

Make sure you have the console USB cable in place and use it so you know when the installation has finished. You MUST NOT remove power before it’s done or it could be bricked. If you don't have a console connection make sure you wait 2 minutes at the end of the installation as it instructs. During this time it is completing the installion which shoudln't be interrupted. If you don't get any update on your console after this message is displayed restart your console terminal connection.

Connect to the console with 115000 8N1, for example: 

`screen /dev/USB0 115200 8N1` 

and login as root (password: edison)


## Post Ubilinux Install
After Ubilinux has been installed you will end up with the following partitions:

```
Filesystem       Size  Used Avail Use% Mounted on
rootfs           1.4G  813M  503M  62% /
/dev/root        1.4G  813M  503M  62% /
devtmpfs         480M     0  480M   0% /dev
tmpfs             97M  292K   96M   1% /run
tmpfs            5.0M     0  5.0M   0% /run/lock
tmpfs            193M     0  193M   0% /run/shm
tmpfs            481M     0  481M   0% /tmp
/dev/mmcblk0p7    32M  5.3M   27M  17% /boot
/dev/mmcblk0p10  1.3G  2.0M  1.3G   1% /home
```

## Post ROS Install
Once ROS is installed there won't be much space left on the root partition. TODO: Add howto on freeing up space.

```
Filesystem       Size  Used Avail Use% Mounted on
rootfs           1.4G  1.1G  194M  86% /
/dev/root        1.4G  1.1G  194M  86% /
devtmpfs         480M     0  480M   0% /dev
tmpfs             97M  304K   96M   1% /run
tmpfs            5.0M     0  5.0M   0% /run/lock
tmpfs            193M     0  193M   0% /run/shm
tmpfs            481M  6.6M  474M   2% /tmp
/dev/mmcblk0p7    32M  5.3M   27M  17% /boot
/dev/mmcblk0p10  1.3G  381M  910M  30% /home
```

# Pre-Installation Steps

## Freeing up Space on the Root Partition

You will need more space on the root partition. Run the following commands:

`mv /var/cache /home/`
`ln -s /home/cache /var/cache`

Add the following to the file `/etc/dpkg/dpkg.cfg` to prevent installing docs, locales and man pages. You can also delete the contents of the folders `sudo rm -rf /usr/share/locale/*`, `sudo rm -rf /usr/share/man/*` and `sudo rm -rf /usr/share/doc/*`.
```
# /etc/dpkg/dpkg.conf.d/01_nodoc

# Delete locales
path-exclude=/usr/share/locale/*

# Delete man pages
path-exclude=/usr/share/man/*

# Delete docs
path-exclude=/usr/share/doc/*
path-include=/usr/share/doc/*/copyright
```

## Wifi
Run `wpa_passphrase your-ssid your-wifi-password` to generate pka.
`cd /etc/network`
Edit /etc/network/interfaces
- Change wpa-ssid
- Change wpa-pka
- Comment out `auto usb0` plus the three lines that follow it (interface definition)
- Uncomment `auto wlan0`
- Save
Run: `ifup wlan0`

If you want to use a static IP then your config will look something like this:
```
auto wlan0
iface wlan0 inet static
    # For WPA
    wpa-ssid <your-ssid>
    wpa-psk <your-ssid-psk>

    address 192.168.43.101
    netmask 255.255.255.0
```

For the remaining steps you may wish to login via ssh instead.

## Update
```
apt-get -y update
apt-get -y upgrade
```

## Locales
```
dpkg-reconfigure locales # Select only en_US.UTF8 and select None as the default on the confirmation page that follows.
update-locale
```
Update the `/etc/default/locale` file an ensure `LANG=en_US.UTF-8` then reboot.

Note that if you receive warning messages about missing or wrong languages this is likely to be due to the locale being forwarded when using SSH. Either ignore them or complete this step via the serial console by commenting out the SendEnv LANG LC_* line in the local /etc/ssh/ssh_config file on your machine (not the Edison).

## Timezone
`dpkg-reconfigure tzdata`

## Tools
```
apt-get -y install git
apt-get -y install sudo less
```

## Add User
`adduser px4`
`passwd px4` (set the password to px4)
`usermod -aG sudo px4`
`usermod -aG dialout px4`

Login as px4 to continue.

## Hosts file
`vim /etc/hosts`
Add this line:
`127.0.0.1 ubilinux`

# Upgrade Debian 7 Wheezy to 8 Jessie

To avoid build issues, these install script depend on Debian Jessie. You can use Jubilinux (at your own risk), or, to upgrade the Debian base in Ubilinux to Jessie, do the following:

1. `sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade`
2. `vim /etc/apt/sources.list` Replace every `Wheezy` with `Jessie`. Comment out the line `#deb http://ubilinux.org/edison wheezy main`. Your sources.list file should look like this:
```
deb http://http.debian.net/debian jessie main contrib non-free
#deb-src http://http.debian.net/debian jessie main contrib non-free

deb http://http.debian.net/debian jessie-updates main contrib non-free
#deb-src http://http.debian.net/debian jessie-updates main contrib non-free

deb http://security.debian.org/ jessie/updates main contrib non-free
#deb-src http://security.debian.org/ jessie/updates main contrib non-free

#deb http://ubilinux.org/edison jessie main

deb http://http.debian.net/debian jessie-backports main

#deb http://http.debian.net/debian stretch main contrib non-free
```
3. `sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade`
4. To save space:
```
sudo apt-get clean
sudo rm -r /var/lib/apt/lists/*
sudo apt-get update
```
5. Make sure you have a serial connection through USB and reboot.

6. On reboot, press enter to stop the usual boot process and set the following settings in U-Boot:
```
setenv bootargs_target multi-user
saveenv
```

# ROS/MAVROS Installation

As ROS packages for the Edison/Ubilinux don't exist we will have to build it from source. This process will take about 1.5 hours but most of it is just waiting for it to build.

A script has been writen to automate the building and installation of ROS, MAVROS and MAVROS_EXTRAS. Current testing has been copy-pasting line by line to the console. Willing testers are encouraged to try out running the script:

```
git clone https://github.com/pennaerial/ros-setups
cd ros-setups/intel-edison/
./install_kinetic_mavros.sh
```

If all went well you should have a ROS installtion. Hook your Edison up to the Pixhawk and run a test. See this page for instructions: https://pixhawk.org/peripherals/onboard_computers/intel_edison

## Known issues

Currently only the install_kinetic_mavros.sh script has had success, installing indigo works on a clean image of Ubilinux Wheezy, but without mavros_extras due to compilation errors in cv_bridge.

# Python Flight App

Once you have a functional ROS setup you can *very carefully* perform an offboard flight using the setpoint_demo.py script. This script assumes that you have already successfully run `roslaunch mavros px4.launch`.

WARNING WARNING: Make sure you can take control via RC transmitter at any time, things can go quite wrong. Also be aware that there isn't any velocity control currently and the multirotor will use max velocity at times. Read the code before you fly so you know what to expect.

Launch the demo by running:

`./setpoint_demo.py`

and once it is running activate offboard control on your RC transmitter.

# Post-Installation Tips

## Mounting the Edison Mass Storage Partition within Ubilinux
If you need to transfer files between your computer and the Intel Edison, you can also mount the mass storage partition within Ubilinux. It will be accessible by both systems concurrently. Run the following:

```
sudo mkdir /edison
sudo mount -t vfat /dev/mmcblk0p9 /edison
```

## Freeing up Space on the Root Partition

Once again we will remove unneeded files from the root partition. You can delete the files in root's home directory (that's /root) or move them to the home partition.
