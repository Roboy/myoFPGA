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
### Install Quartus 17.0 and SoC-EDS 17.0
* Download [Quartus 17.0](http://dl.altera.com/?edition=lite), the updated version and install to ~/intelFPGA
* Download [SoC-EDS](https://dl.altera.com/soceds/) and install to ~/intelFPGA
* Get the [free license key](https://developer.arm.com/products/software-development-tools/ds-5-development-studio/editions/customized-editions/altera/community-edition/)
* The installation of ds-5 will fail because of missing permissions. 
* Simply run the the following script as root after installation: ~/altera/16.1/embedded/ds-5_installer/install.sh. 
* If you have missing dependencies run the script: ds-deps-ubuntu_64.sh
* Install to: ~/intelFPGA/17.0/embedded/ds-5

You now have a full development suite for fpga and arm compilation.

### Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
### Intall OpenPowerLink
* install pcap: sudo apt install libpcap-dev
* cd /path/to/myoFPGA/src/roboy_powerlink/stack/build/linux && cmake ../.. -DCMAKE_BUILD_TYPE=Debug && sudo make install && cmake ../.. -DCMAKE_BUILD_TYPE=Release && sudo make install
* edit myoFPGA/myoFPGA/src/roboy_managing_node/cmake/options.cmake according to your path to openPowerLink
  * SET(OPLK_BASE_DIR /home/roboy/Downloads/openPowerLink)

## Build
### Build arm controlled node and host managing node
```
#!/bin/bash
cd myoFPGA/myoFPGA
~/intelFPGA/17.0/embedded/embedded_command_shell.sh
catkin_make
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
