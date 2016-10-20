# my_ps6

Your description goes here

## Example usage
Because then executing time I set for addressing callback funcion is only 30s, a bit longer than the duration time of bag file, it is desirable to run the ps6_lidar_transofrm node prior to the bag file.
So the recommended step is (in order):
rosparam set use_sim_time true
rosrun my_ps6 ps6_lidar_transformer
rosbag play block_scan.bag --clock
## Running tests/demos
    
