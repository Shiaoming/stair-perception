
# Introduction

This is a stair perception kit programming with PCL and Qt. It can model the stair and measure the stair's height, width, steps and so on. Tht input data can either be Kinect V2 video stream or saved images and point clouds from disk.

![screenshot](https://raw.githubusercontent.com/Psunshine/stair-perception/master/pic/screenshot1.png)
![screenshot](https://raw.githubusercontent.com/Psunshine/stair-perception/master/pic/screenshot2.png)

<div  align="center">    
<img src="https://raw.githubusercontent.com/Psunshine/stair-perception/master/pic/stairmodel.png" width = "350" alt="stairmodel" align=center />
</div>


For more details about this algorithm, please refer to paper **Real-Time Stairs Geometric Parameters Estimation for Lower Limb Rehabilitation Exoskeleton** on *30th Chinese Control and Decision Conference (2018 CCDC)*.

My algorithm is far from perfect and not stable in some conditions. I will be very glad that if you can push to improve it. :)

# Dependencies

* [libfreenect2](https://github.com/OpenKinect/libfreenect2)
* [libfreenect2pclgrabber](https://github.com/giacomodabisias/libfreenect2pclgrabber)
* opencv
* [pcl](https://github.com/PointCloudLibrary/pcl)
* Qt-5

# Build

1. install ```libfreenect2``` according to their [doc](https://github.com/OpenKinect/libfreenect2).
1. install opencv
    ```sudo apt-get install libopencv-dev```
1. install pcl
    - one can get pcl libaray by [installing ros environment](http://www.ros.org/install/)
    - or compiling form source code (refer to this [link](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php))
1. install qt5
    ```sudo apt-get install cmake qt5-default qtcreator```
1. finally, make and test!
    ```bash
    git clone https://github.com/Psunshine/stair-perception.git
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
