# Teensy Setup

1. Download and apply Teensy udev rules:
    ```bash
    cd ~
    curl -O https://www.pjrc.com/teensy/00-teensy.rules
    sudo cp 00-teensy.rules /etc/udev/rules.d/
    rm 00-teensy.rules
    ```

2. Download the latest verion from https://www.arduino.cc/en/software. Make sure to download the ZIP file, not the AppImage, since we need access to the internal files.

3. Launch by running:
    ```bash
    ./arduino-ide_2.3.2_Linux_64bit/arduino-ide
    ```

4. In IDE, In File > Preferences > "Additional boards manager URLs" copy the URL "https://www.pjrc.com/teensy/package_teensy_index.json" (reference [1]):

5. In IDE, Open Boards Manager by clicking the left-side board icon, search for "Teensy", and click "Install". 

6. Install pre-compiled micro-ros library release "v2.0.7-humble" (reference [2])
    ```bash
    cd ~/Downloads
    curl -0 https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.7-humble.zip
    ```

7. In IDE, in Sketch > Include Library > Add .ZIP Library.. find and select `micro_ros_arduino-2.0.7-humble.zip`

8. Patch `platform.txt` in `.arduino/` folder to llow pre-compiled libraries
    ```bash
    cd ~/.arduino15/packages/teensy/hardware/avr/1.59.0/
    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/iron/extras/patching_boards/platform_teensy.txt > platform.txt
    ```
9. Download `libmicroros.a` from MARIAM Sharepoint and replace the file `/Arduino/libraries/micro_ros_arduino/src/imxrt1062/fpv5-d16-hard/libmicroros.a`

10. Install libraries for MARIAM: Copy all folders in `~/MARIAM/arduino/micro_ros_MARIAM/libraries` to `~/Arduino/libraries`
    ```bash
    cp -r ~/MARIAM/arduino/micro_ros_MARIAM/libraries/* ~/Arduino/libraries/
    ```

11. Validate installation. Restart IDE, File > Open > "~/MARIAM/arduino/micro_ros_MARIAM/micro_ros_MARIAM.ino, press the check mark to compile.


# References
- [1] https://www.pjrc.com/teensy/td_download.html: Instructions for downloading the teensyduino integration
- [2] https://github.com/micro-ROS/micro_ros_arduino/tree/humble: Instructions for integrating micro-ros library

# Teensy Notes

Well lots of hacking around finally got both sides working, I need to figure out what changes I did are necessary and not, I'll attempt to reproduce a working setup on a lab system and create a readme so we don't have just my personal laptop working.
The 4 big things of note are:

- Obvious stuff https://github.com/micro-ROS/micro_ros_arduino/blob/humble/README.md#patch-teensyduino
- The latest version supported arduino ide by the TeensyduinoInstall.linux64 is 1.8.15 (not 2) teensy_post_compile is broken on 1.8.15 for any ROS stuff when anything else is linked it cant find the binary
- The micro_ros_arduino-2.0.7-humble.zip  is missing a critical binary ie imxrt1062 fpv5-d16-hard the folder is empty in the provided zip so I built it from the micro_ros_arduino  repo and copied over the /home/carter/MARIAM/src/micro_ros_arduino/src/imxrt1062/fpv5-d16-hard/libmicroros.a and that seemed to work
- Could only build and load on the teensy on arduino-ide_2.2.1_Linux_64bit 

So yeah it was a lot of jumping around installing things in one place and jumping to another and uninstalling stuff ect. but I need to find the simplest path to working something to note the micro_ros stuff they say This software is not ready for production use. It has neither been developed nor tested for a specific use case. and it looks like they are not maintaining it well some crazy divergent trees
Humble: This branch is 121 commits ahead of, 201 commits behind iron.
Galactic: This branch is 216 commits ahead of, 450 commits behind iron. (edited) 

![Teensy Setup Image](images/teensySetupImage.png?raw=true "Teensy Setup Image")

# OLD Teensy Setup
1. Install Arduino 1.8.15 from (https://www.arduino.cc/en/software/OldSoftwareReleases)
2. Download and apply Teensy udev rules 
```bash
curl -O https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
```
3. Run the Teensy installer for Arduino 1.8.x from https://www.pjrc.com/teensy/td_download.html
```bash
curl -0 https://www.pjrc.com/teensy/td_159/TeensyduinoInstall.linux64
chmod 755 TeensyduinoInstall.linux64
./TeensyduinoInstall.linux64
```
4. Add Micro-ROS library from https://github.com/micro-ROS/micro_ros_arduino