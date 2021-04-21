# slam_karto_g2o
A ROS package for 2-D pose graph SLAM using open karto package for the front-end and g2o solver for the back-end. This package has been tested on Ubuntu 18.04. The version of g2o solver we tested is [Release Version 20200410](https://github.com/RainerKuemmerle/g2o/releases/tag/20200410_git). We start installing g2o solver by installing all the dependencies:
```
$ sudo apt-get install cmake libeigen3-dev libsuitesparse-dev
```
We are now ready to build and install g2o solver:
```
$ tar zxf g2o-20200410_git.tar.gz
$ mkdir g2o-bin
$ cd g2o-bin
$ cmake ../g2o-20200410_git
$ make -j4
$ sudo make install
```

After installing g2o solver, please create and initialize a ROS workspace. We assume that your workspace is named catkin_ws. Then, run the following commands to clone and build open karto package:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ros-perception/open_karto.git
$ cd ..
$ catkin_make
```

After the above preparation, clone and build this package:
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/nkuwenjian/slam_karto_g2o.git
$ cd ..
$ catkin_make
```

Finally, run the following commands to launch Karto SLAM:
```
$ source ~/catkin_ws/devel/setup.bash
$ roslaunch slam_karto_g2o slam_karto_g2o.launch
```

Open a new terminal and play your rosbag:
```
$ rosbag play <rosbagfile> --clock
```

## Remarks
The source code of the SLAM back-end in this package refers to the [tutorial_slam2d](https://github.com/RainerKuemmerle/g2o/blob/master/g2o/examples/tutorial_slam2d/tutorial_slam2d.cpp) in g2o examples. It is recommended to read this tutorial to learn more about the use of g2o.