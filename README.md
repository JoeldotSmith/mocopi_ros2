# Mocopi to Ros2
This ros2 package connects the mocopi motion capture system from sony seen [here](https://electronics.sony.com/more/mocopi/all-mocopi/p/qmss1-uscx?srsltid=AfmBOopi66GOcrqdH1cDTxvq5f5IxXbZ4eLk3l5UIrA47Kxz0TRwDVpK)

## Ros2 version
This package was built and only tested on **ros2 foxy**

## How to use
Starting node with 
```bash 
ros2 run mocopi_ros2 mocopi_receiver

```
After starting the node, to view the information from the mocopi phone app, run
```bash
rviz2 -d .../mocopi_ros2/config/mocopi_view.rviz
```

This will open rviz and if everything is running and connenct a skeleton will appear and track the movements of the mocopi sensors.

## Acknowledgment

A big thanks to the team from the people who wrote the [original ros1 package](https://github.com/hello-world-lab/mocopi_ros.git) which this code is originally based and has been converted.

And another big thanks to the people who wrote the [orginal plugin](https://github.com/seagetch/mcp-receiver.git) in which the ros1 package was based on