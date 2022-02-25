# sel_map (Semantic ELvation Map)
**Authors:** Parker Ewen (pewen@umich.edu), Adam Li (adamli@umich.edu), Yuxin Chen (chyuxin@umich.edu), Steven Hong (hongsn@umich.edu), and Ram Vasudevan (ramv@umich.edu). 

- All authors affiliated with the Robotics Institute of the University of Michigan, 2505 Hayward Street, Ann Arbor, Michigan, USA.
- This work is supported by the Ford Motor Company via the Ford-UM Alliance under award N022977, by the Office of Naval Research under award number N00014-18-1-2575, and in part by the National Science Foundation under Grant 1751093.
- `sel_map` was developed in [Robotics and Optimization for Analysis of Human Motion (ROAHM) Lab](http://www.roahmlab.com/) at University of Michigan - Ann Arbor.

## Introduction
<img align="right" height="230" src="/figures/main.png"/>
Semantic ELevation (SEL) map is a semantic Bayesian inferencing framework for real-time elevation mapping and terrain property estimation. The package takes the inputs from RGB-D cameras and robot poses, and recursively estimates both the terrain surface profile and a probability distribution for terrain properties. The package can be deplolyed on a physical legged robotic platform in both indoor and outdoor environments. The semantic networks used in this package are modular and interchangeable, better performance can be achieved using the specific trained networks for the corresponding applications. This package provides several examples such as ResNet-50.


<img height="270" src="/figures/flow_diagram.png"/>

<img height="230" src="/figures/terrain_class.png"/> <img height="230" src="/figures/terrain_property.png"/>

## Dependencies
The package is built on Ubuntu 20.04 with ROS Noetic Distribution, and the algorithms are compiled with C++11 and Python3.

`sel_map` has the following required dependencies
* mesh-msgs (version 1.1.0 or higher)
* opencl-headers
* hdf5-map-io
* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html)
* [Python3](https://www.python.org/download/releases/3.0/)
* [cv-bridge](http://wiki.ros.org/cv_bridge)
* [CUDA 11](https://developer.nvidia.com/cuda-downloads)
* [PyTorch](https://pytorch.org)

The following commands can be used to install the dependencies, and follow the [linked instruction](https://catkin-tools.readthedocs.io/en/latest/installing.html) to install `catkin_tools`.
```
sudo apt install ros-noetic-mesh-msgs opencl-headers ros-noetic-hdf5-map-io python3-pip ros-noetic-cv-bridge
```

Make sure the system is setup with CUDA 11, and install [pytorch](https://pytorch.org) with CUDA 11. One example is shown below.
```
pip3 install torch==1.10.2+cu113 torchvision==0.11.3+cu113 torchaudio==0.10.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
```

Install the required python packages using the following command in `sel_map` folder.
```
pip install -r requirements.txt
```
(Note: if you want to install them globally, add `sudo` previledge.)

## Building

1. Create a catkin workspace.
```
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
```

2. Download the package in src folder, make sure in main branch.
```
git clone git@github.com:roahmlab/sel_map.git
```

3. (Optional) For Spot RViZ visualization, download the package from Clearpath Robotics in src folder.
```
git clone git@github.com:clearpathrobotics/spot_ros.git
```
(Note: This step is not required if you don't need Spot visualization or work on a different robot.)

4. Add pretrained network models to semseg folder.
For example, download the 2 files from [CSAIL link](http://sceneparsing.csail.mit.edu/model/pytorch/), and put them in the folder ~/catkin_ws/src/sel_map/sel_map_segmentation/mit_semseg_wrapper/ckpt/ade20k-resnet50dilated-ppm_deepsup.

5. Build the package using either `catkin_make` or `catkin build` in catkin_ws folder.
```
catkin build
```

6. Source the workspace.
```
source devel/setup.bash
```


## Usage
Before running the package, make sure you build the package successfully and source the workspace.

In one terminal, launch the package:
```
roslaunch sel_map spot_sel.launch
```

In another terminal, run the rosbag data:
```
rosbag play spot_comp8.data --clock --pause
```
(Note: When doing rosbag playbacks, make sure to use `--clock`)

In order to run different network model, specific the `semseg_config` argument. The available network models are located in [sel_map/sel_map/config/semseg](https://github.com/roahmlab/sel_map/tree/main/sel_map/config/semseg). One example is shown below.
```
roslaunch sel_map spot_sel.launch semseg_config:=Encoding_FCN_ResNeSt50_PContext_full.yaml
```

If you have `CUDA out of memory` error, you can either try a more powerful laptop, or just run the elevation mapping without sementic segmentation.
```
roslaunch sel_map spot_sel.launch semseg_config:=Bypass.yaml
```

## License

`sel_map` is released under a [MIT license](https://github.com/roahmlab/sel_map/blob/main/LICENSE). For a list of all code/library dependencies, please check dependency section.

For a closed-source version of `sel_map` for commercial purpose, please contact the authors.

An overview of the theoretical and implementation details has been published in [to_be_added] If you use `sel_map` in an academic work, please cite using the following BibTex entry:

  @article{to_be_added,
          title={to_be_added}
  }


