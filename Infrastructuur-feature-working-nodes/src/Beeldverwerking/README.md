# Beeldverwerking

Voor de codestandaard, klik [hier](https://github.com/R2D2KLASB/Info/blob/main/CodeStandaard.md)

# Install

The install.sh script should download automatically the dependencies and build the ros2 package.

```
cd "PATH-TO-ROS2-ENVIROMENT/src"
git clone https://github.com/R2D2KLASB/Beeldverwerking.git -b dev
cd Beeldverwerking
./install.sh
```


# Run

Talker( Web Interface ):
```
ros2 run beeldverwerking talker
```
Talker( Command-Line ):
```
ros2 run beeldverwerking talker "path-to-image"
```

Subscriber:
```
ros2 run beeldverwerking subscriber
```

If package 'beeldverwerking' not found:
```
cd "PATH-TO-ROS2-ENVIROMENT"
. install/setup.bash
```

Don't forget to source your ROS 2 installation before running an package!
