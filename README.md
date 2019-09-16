### Library for V2X based on ROS2

***Concept***

ROS2 -> libv2x -> libv2x_msgs -> v2x


***Prerequisites***

| Item      | Version                                |
| -         | -                                      |
| Linux     | Ubuntu 18.04 LTS                       |
| ROS2      | Dashing Diademata                      |
| asn1c     | https://github.com/vlm/asn1c           |
| asn1 file | SAE J2735 May-2016, J2735_201603DA.asn |                          |


***Build Instruction***

> Install ROS2 on Linux

> Install asn1c

https://github.com/vlm/asn1c

> Create ROS2 workspace directory and change directory
```
mkdir -p ws/src
cd ws
```

> Clone source codes
```
git clone https://github.com/libv2x/v2x src
```

> Build ASN1 codec
```
cp J2735_201603DA.asn libv2x/asn1/J2735_201603DA.asn
cd libv2x && ./codec.sh && cd ..
```

> Build
```
colcon build  --symlink-install
```

***Run Example***

> Terminal 1
```
source install/local_setup.bash
ros2 run v2x v2x_ieee1609dot3_devemu __log_level:=debug
```
> Terminal 2
```
source install/local_setup.bash
ros2 run v2x v2x_ieee1609dot3 __log_level:=debug
```
```
[DEBUG] [rcl]: Subscription in wait set is ready
[DEBUG] [rcl]: Subscription taking message
[DEBUG] [rcl]: Subscription take succeeded: true
[DEBUG] [ieee1609dot3]: 1568364003.396481301
[DEBUG] [ieee1609dot3]: 3 32 510
[DEBUG] [rcl]: Waiting without timeout
[DEBUG] [rcl]: Timeout calculated based on next scheduled timer: false
[DEBUG] [rcl]: Guard condition in wait set is ready
[DEBUG] [rcl]: Waiting without timeout
[DEBUG] [rcl]: Timeout calculated based on next scheduled timer: false
[DEBUG] [rcl]: Subscription in wait set is ready
[DEBUG] [rcl]: Subscription taking message
[DEBUG] [rcl]: Subscription take succeeded: true
[DEBUG] [ieee1609dot3]: 1568364004.396473876
[DEBUG] [ieee1609dot3]: 3 32 510
```
