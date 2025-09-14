# ROS2 导航试点
## 已完成：
2. 仿真已完成
3. 通信包
## 未完成
0. 适配时空体素层 !!!
1. 控制器插件重写
2. 航点请求 BT-service
## 待确定
1. ROS2 Navigation2 自带 BT-Tree，通信协议是否需要修改

# 编译说明
navi2_bringup
```shell
sudo apt install ros-humble-nav*  
sudo apt install ros-humble-spatio-temporal-voxel-layer*
sudo apt install ros-humble-openvdb-vendor*
```
communication
```shell
git clone https://github.com/libevent/libevent 
cd libevent
mkdir build && cd build
cmake .. && make -j16
sudo make install
```