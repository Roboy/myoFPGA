# myoFPGA
This repo contains design files for altera de0 nano fpga implementing control for myoMuscles via SPI. 

## Installation 
### Get the source:
```
#!/bin/bash
git clone https://github.com/Roboy/myoFPGA
cd myoFPGA/
git checkout openPowerlink
```
### Install Quartus 16.1 and SoC-EDS
* Download [Quartus 16.1](http://dl.altera.com/?edition=lite), the updated version and install to ~/altera
* Download [SoC-EDS](https://dl.altera.com/soceds/) and install to ~/altera/16.1
* Get the [free license key](https://developer.arm.com/products/software-development-tools/ds-5-development-studio/editions/customized-editions/altera/community-edition/)
* The installation of ds-5 will fail because of missing permissions. 
* Simply run the the following script as root after installation: ~/altera/16.1/embedded/ds-5_installer/install.sh. 
* If you have missing dependencies run the script: ds-deps-ubuntu_64.sh
* Install to: ~/altera/16.1/embedded/ds-5

You now have a full development suite for fpga and arm compilation.

### Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
### Intall OpenPowerLink
* Download [openPowerLink](https://sourceforge.net/projects/openpowerlink/)
* install pcap: sudo apt install libpcap-dev
* [Build the libs](http://openpowerlink.sourceforge.net/doc/2.5/2.5.1/page_build_stack.html) (both debug and release)
* edit myoFPGA/myoFPGA/src/roboy_managing_node/cmake/options.cmake according to your path to openPowerLink
  * SET(OPLK_BASE_DIR /home/roboy/Downloads/openPowerLink)

## Build
### Build arm controlled node and host managing node
```
#!/bin/bash
cd myoFPGA/myoFPGA
~/altera/16.1/embedded/embedded_command_shell.sh
catkin_make
```
### Build host managing node only
```
#!/bin/bash
cd myoFPGA/myoFPGA
~/altera/16.1/embedded/embedded_command_shell.sh
catkin_make --pkg 
```
## Run controlled node
The controlled node runs on the SoC arm core of the fpga. You probably already have a running fpga so login to it via:
```
#!/bin/bash
ssh root@192.168.0.102
```
If you want to copy the generated roboy_controlled_node_fpga binary to the fpga you can use scp:
```
#!/bin/bash
ssh path/to/roboy_controlled_node_fpga root@192.168.0.102:/root/
```
Run the controlled node:
```
#!/bin/bash
root@cyclone5-soc:~ $ ./roboy_controlled_node_fpga
```
