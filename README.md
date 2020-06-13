
**Note**: Upgraded ros version stair-perception can be found [here](https://github.com/Shiaoming/stair-perception-ROS).

# Introduction

This is a stair perception kit programming with PCL and Qt which can model the stair and measure the stair's height, width, steps and so on. Tht input data can either be Kinect V2 video stream or saved images and point clouds from disk.

You can run it in terminal after build scince some useful information not shown in the gui window is printed. The main window is shown below, as you can seen, the visualization of original image and estimated stair model are included. Of course, for the convenience of tuning parameters, some of them can be adjust in the main window (notice some paramerers are not used at last, please refer to the original code). The status bar show some basic infomation including the fist stair parameters, IMU pose and running time.
![](https://raw.githubusercontent.com/Psunshine/stair-perception/master/pic/screenshot1.png)

You can get more detail stair information in the terminal output. For example, the staircase shown above have **four** steps, each step's geometric parameters such as height and depth are printed.
<div  align="center">    
<img src="https://raw.githubusercontent.com/Psunshine/stair-perception/master/pic/screenshot2.png"  alt="terminal" align=center />
</div>

The meaning of **H**eight, **D**epth, **V_D**istance and **H_D**istance are shown in the picture below.
<div  align="center">    
<img src="https://raw.githubusercontent.com/Psunshine/stair-perception/master/pic/stairmodel.png"  alt="stairmodel" align=center />
</div>

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
    git clone https://github.com/Shiaoming/stair-perception.git
    mkdir build
    cd build
    cmake ..
    make -j4
    ```

For more details about this algorithm, please refer to paper **Real-Time Stairs Geometric Parameters Estimation for Lower Limb Rehabilitation Exoskeleton** on *30th Chinese Control and Decision Conference (2018 CCDC)*.

```
@article{zhao_adaptive_2019,
	title = {An adaptive stair-ascending gait generation approach based on depth camera for lower limb exoskeleton},
	volume = {90},
	issn = {0034-6748},
	url = {https://doi.org/10.1063/1.5109741 http://aip.scitation.org/doi/10.1063/1.5109741},
	doi = {10/ggsxh2},
	number = {12},
	journal = {Review of Scientific Instruments},
	author = {Zhao, Xiaoming and Chen, Wei-Hai and Li, Bing and Wu, Xingming and Wang, Jianhua},
	month = dec,
	year = {2019},
	note = {Publisher: AIP Publishing, LLC},
	pages = {125112}
}
@inproceedings{zhao2018real,
	title={Real-Time Stairs Geometric Parameters Estimation for Lower Limb Rehabilitation Exoskeleton},
	author={Zhao, Xiaoming and Chen, Weihai and Yan, Xing and Wang, Jianhua and Wu, Xingming},
	booktitle={2018 Chinese Control And Decision Conference (CCDC)},
	pages={5018--5023},
	year={2018},
	organization={IEEE}
}
```
