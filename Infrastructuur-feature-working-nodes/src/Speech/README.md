# Speech
Voor de codestandaard, klik [hier](https://github.com/R2D2KLASB/Info/blob/main/CodeStandaard.md)

# Table of Contents
1. [Installation](#Installation)
2. [Preparation](#Preparation)
3. [Build & Run](#Build&Run)

# Installation
## Ubuntu 20.04 LTS (RPi4)
1. Make sure that you run ubuntu linux focal fossa (20.04).
2. To install ros2 on ubuntu follow the installation guide. [Click here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html )
3. To install ros2 build tools and ssl for cmake run: 
    > sudo apt install python3-colcon-common-extensions libssl-dev
4. Clone the latest portaudio source files with:
    > git clone https://github.com/PortAudio/portaudio.git
5. In the portaudio repo make a folder named "install".
6. Run the following command to generate the build files from the portaudio repo:
    > cmake . -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/ubuntu/portaudio/install
7. Now build and install portaudio with:
    > make && make install

# Preparation
Before you can run the speech node you have to set your mic as the default input device and give the use permissions to use gpio.

1. Firstly run the command:
    > arecord -1
2. You will be presented with a list of devices, remember the **card number** and the **device number** of the device you want to use.
3. Open a new file with:
    > nano ~/.asoundrc
4. Insert this into the file with **your** card number and device number:

        pcm.!default {
            type asym
            capture.pcm "mic"
        }
        pcm.mic {
            type plug
            slave {
                pcm "hw:<card number>,<device number>"
            }
        }
5. Give the user acces to GPIO.
    > sudo chown ubuntu /dev/gpiomem
    
    > sudo chmod g+rw /dev/gpiomem

# Build & Run <a name="Build&Run"></a>
1. Clone this repository.
2. Move to speech_ros2 and source the ros2 environment with:
    > ~/ros2_foxy/ros2-linux/setup.bash
3. Now build the speech node with: 
    > colcon build --merge-install --packages-select speech_ros2
4. Run the setup files:
    > call install/setup.bash
5. Now you can run the speech node with:
    > ros2 run speech_ros2 talker