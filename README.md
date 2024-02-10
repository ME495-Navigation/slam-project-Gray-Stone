#ME495 Sensing, Navigation and Machine Learning For Robotics
* Leo Chen
* Winter 2024
#Package List
This repository consists of several ROS packages
- nuturtle_description - robot description for turtle bot
- turtlelib - Pure Cmake Project library for computation
- nusim - simulator for turtlebot.


## Odomatry working with real turltebot. 

Here is a video where the odomatry and turtle control running on the actual robot. Tracking the robot's position while robotis drived around in circles.

[](https://github.com/ME495-Navigation/slam-project-me495/assets/7969697/81fa220b-b19b-435d-8e55-745e553d9f54)

The initial odom reading is 
```
x=0.005 y=0 theta = -0.00715
```

After drinving a few circles, and parked robot back to roughtly where it started, the odom reading is 

```
x= 0.102  y=0.0358  theta= 0.396
```

This reading is certainly no where near accurate. The main source of error is likely from multiple acceleration and deceleration, where in odom, we assumed constant velocity. Specially when the robot reverse and driven manually. the error increased dramateclly. 