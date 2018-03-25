# Perception & Interface Design

**VERY IMPORTANT (the following information may save your ass (well, tons of hours of installation))**: if you are using ubuntu, be careful with anaconda because conda Interference might prevent the required packages to be installed into the system python (if your don't define your `PATH` to search for packages clearly in the environment, and each time you try to install a package, the computer will automatically find the installed package which might not in the required directory e.g. Anaconda). Disable the conda python and force the program to use the system default python instead is a secured choice. Then use `pip install` (or with a `sudo` in the front in some cases) to install all the packages required to run the project.

### Network Configuration

#### (Optional) Ubuntu Ethernet Network Manipulations

In order to run the project smoothly, all computers involved in the system have to be under the same network. For this specific project, all computers were connected via ethernet. The following session is just for the purpose to make sure Ubuntu connect to ethernet properly.

Firstly, `ctrl` + `alt` + `t` to open the terminal and register as root:

```bash
$ sudo bash
```
Then navigate to the network directory for further manipulations:

```bash
$ cd /etc/network
```
After that, edit the `interfaces` file:

```bash
$ less interfaces
$ gedit interfaces
```
Within the `interfaces` file, you should be seeing contents like the following:

***need to check on Ubuntu***

Then simply comment out everything below the line of: `source /etc/network/interfaces.d/*`

After all the above has been done, reintall and update the network drivers and managers in case it has not been properly installed:

```bash
$ apt-get install network-manager-pptp network-manager-pptp-gnome
```
After that, plug in the wired connection and disable the WiFi connection left only wired connection avaible by click ***need to check on Ubuntu***

Finally, restart the wired connection by:

```bash
$ systemctl restart network-manager.service
```
and the wired connection should be working properly for future project developments.

#### Ckecking Network Configuration

In order to make sure the computer running "perception" process is in the same network environment with the ROS master, check the network configuration of the "perception" computer by: 

```bash
$ ifconfig -a
```



### Graphic Card Configuration (for Vive & Kinect Camera)

For the purpose of setting up a proper environment for the Vive Headset and Kinext Camera Sensor, certain steps of driver package configurations have to been processed.

#### Kinect Configuration



#### Vive Configuration

Firstly, check the Nvdia version on the perception host computer by:

```bash
$ nvidia-smi
```
***check the usage of the following line***

```bash
$ apt-get install dnsmasq
```

For this particular project, the Vive headset is compatible and running in maximum efficiency with `cuda-8.0`, thus need to make sure installing cuda by downloading the package from ***check the website***

`$ ls -tr` to set the list as time order to find the latest download in the directory.

***check the usage of `chmod +x`***

```bash
$ chmod +x cuda_8.0.61_375.26_linux.run
$ sudo ./cuda_8.0.61_375.26_linux.run
```

#### (Optional) might not work on all computers

Graphic card installation for the Vive

```bash
sudo apt-add-repository ppa:graphics-drivers/ppa
```