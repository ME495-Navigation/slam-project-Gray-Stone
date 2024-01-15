# ME495 Sensing, Navigation and Machine Learning For Robotics
* Leo Chen
* Winter 2024
# Package List
This repository consists of several ROS packages
- nuturtle_description - robot description for turtle bot 



## Package: nuturtle_description

A custom turtlebot description package. Contains the launch file and urdfs for turtlebot with custom modification. 

* turtle bot collision geometry have some parameters 
    * when given a positive body radius, body collision geometry is changed to cylinder (with given radius)

### URDF options

URDF takes color argument to set the robot color to red, green, blue, or purple.

URDF also reads in the `config/diff_params.yaml` file for configuration on it.   

### Launch file

`load_one.launch.py` launches one turtlebot with it's urdf.

arguments: 
```
Arguments (pass arguments as '<name>:=<value>'):

    'use_jsp':
        select weather joint state publisher should be used, true or false. default true
        (default: 'true')

    'use_rviz':
        select weather to use rviz, true or false. default true
        (default: 'true')

    'color':
        select the color of the turtlebot. Valid choices are: ['red', 'green', 'blue', 'purple']
        (default: 'purple')
```
