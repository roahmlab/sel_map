# sel_map (Semantic ELevation Map)
**Authors:** Parker Ewen (pewen@umich.edu), Adam Li (adamli@umich.edu), Yuxin Chen (chyuxin@umich.edu), Steven Hong (hongsn@umich.edu), and Ram Vasudevan (ramv@umich.edu). 

- All authors are affiliated with the Robotics Institute and department of Mechanical Engineering of the University of Michigan, 2505 Hayward Street, Ann Arbor, Michigan, USA.
- This work is supported by the Ford Motor Company via the Ford-UM Alliance under award N022977, by the Office of Naval Research under award number N00014-18-1-2575, and in part by the National Science Foundation under Grant 1751093.
- `sel_map` was developed in [Robotics and Optimization for Analysis of Human Motion (ROAHM) Lab](http://www.roahmlab.com/) at University of Michigan - Ann Arbor.

## Introduction
<img align="right" height="230" src="/figures/main.png"/>

Semantic ELevation (SEL) map is a semantic Bayesian inferencing framework for real-time elevation mapping and terrain property estimation. The package takes the inputs from RGB-D cameras and robot poses, and recursively estimates both the terrain surface profile and a probability distribution for terrain properties. The package can be deployed on a physical legged robotic platform in both indoor and outdoor environments. The semantic networks used in this package are modular and interchangeable, better performance can be achieved using the specific trained networks for the corresponding applications. This package provides several examples such as ResNet-50. The dataset for terrain friction can be found in [terrain_friction_dataset](https://github.com/roahmlab/terrain_friction_dataset) repository. The link to the project website is [here](https://roahmlab.github.io/sel_map/).


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
* [CUDA 11](https://developer.nvidia.com/cuda-toolkit)
* [PyTorch](https://pytorch.org)

Make sure the ROS Noetic is installed before downloading the dependencies.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

The following commands can be used to install the dependencies, and follow the [linked instruction](https://catkin-tools.readthedocs.io/en/latest/installing.html) to install `catkin_tools`.
```
sudo apt install ros-noetic-mesh-msgs opencl-headers ros-noetic-hdf5-map-io python3-pip ros-noetic-cv-bridge git ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz
```

Make sure the system is setup with an appropriate version of CUDA 11+ following your preferred method. One example is shown below.
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.0-1_all.deb
sudo apt install ./cuda-keyring_1.0-1_all.deb
sudo apt update
sudo apt install cuda
```

Install [pytorch](https://pytorch.org) with CUDA 11. One example is shown below.
```
pip3 install torch==1.10.2+cu113 torchvision==0.11.3+cu113 torchaudio==0.10.2+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html
```

Install the required python packages using the following command in `sel_map` folder.
```
pip install -r requirements.txt
```
(Note: if you want to install them globally, add `sudo` privilege; however, this is not recommended.)

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
For example, download the 2 files from [CSAIL link](http://sceneparsing.csail.mit.edu/model/pytorch/ade20k-resnet50dilated-ppm_deepsup/), and put them in the folder ~/catkin_ws/src/sel_map/sel_map_segmentation/mit_semseg_wrapper/ckpt/ade20k-resnet50dilated-ppm_deepsup.

5. Build the package using either `catkin_make` or `catkin build` in catkin_ws folder.
```
catkin build
```

6. Source the workspace.
```
source devel/setup.bash
```

**NOTE:** The included [pytorch-encoding](https://github.com/zhanghang1989/PyTorch-Encoding) wrapper takes a while to compile. If you do not want to compile this, or if for some reason you cannot, you can also disable the package by adding a `CATKIN_IGNORE` file to the wrapper package directory. For example:
```
touch src/sel_map/sel_map_segmentation/pytorch_encoding_wrapper/CATKIN_IGNORE
```

## Usage
Before running the package, make sure you build the package successfully and source the workspace.

- Launch the package with Spot demo using the following code.
```
roslaunch sel_map spot_sel.launch
```

- In another terminal, run the rosbag, which provides recorded data to the package.
```
rosbag play spot_comp8.data --clock --pause
```
(Note: When doing rosbag playbacks, make sure to use `--clock`)

- To run different network model, specific the `semseg_config` argument. The available network models are located in [sel_map/sel_map/config/semseg](https://github.com/roahmlab/sel_map/tree/main/sel_map/config/semseg). One example is shown below.
```
roslaunch sel_map spot_sel.launch semseg_config:=Encoding_ResNet50_PContext_full.yaml terrain_properties:=pascal_context.yaml
```
(Note: terrain_properties argument should agree with the network models. PContext with pascal_context and ADE with csail_semseg_properties.)

- If you have `CUDA out of memory` error, you can either try a more powerful laptop, or just run the elevation mapping without semantic segmentation.
```
roslaunch sel_map spot_sel.launch semseg_config:=Bypass.yaml
```

- If you want to show the terrain class map instead of terrain properties, specify the `colorscale` argument.
```
roslaunch sel_map spot_sel.launch colorscale:=use_properties.yaml
```
- The CSAIL network color scheme is shown [here](https://docs.google.com/spreadsheets/d/1se8YEtb2detS7OuPE86fXGyD269pMycAWe2mtKUj2W8/edit#gid=0).
- The pcontext network color scheme is shown [here](https://docs.google.com/spreadsheets/d/1a-73_0Xi4L3U7m5KLqtBJww-4YzNyWlXWooiONRWW98/edit#gid=1446720304).

## License

`sel_map` is released under a [MIT license](https://github.com/roahmlab/sel_map/blob/main/LICENSE). For a list of all code/library dependencies, please check dependency section. For a closed-source version of `sel_map` for commercial purpose, please contact the authors.

An overview of the theoretical and implementation details has been published in [IEEE Robotics and Automation Letters](https://ieeexplore.ieee.org/document/9792203) and IEEE International Conference on Intelligent Robots and Systems (IROS 2022). If you use `sel_map` in an academic work, please cite using the following BibTex entry:


      @article{9792203,
            author={Ewen, Parker and Li, Adam and Chen, Yuxin and Hong, Steven and Vasudevan, Ram},
            journal={IEEE Robotics and Automation Letters}, 
            title={These Maps are Made for Walking: Real-Time Terrain Property Estimation for Mobile Robots}, 
            year={2022},
            volume={7},
            number={3},
            pages={7083-7090},
            doi={10.1109/LRA.2022.3180439}}


