# mmdetection_ros

## Installation

```shell
# Ubuntu 20.04 CUDA 10.1
$ pip3 install torch==1.8.1+cu101 torchvision==0.9.1+cu101 torchaudio==0.8.1 -f https://download.pytorch.org/whl/torch_stable.html
$ pip3 install mmcv-full -f https://download.openmmlab.com/mmcv/dist/cu101/torch1.8.0/index.html
$ pip3 install mmcv mmdet
$ cd ~/ros2_ws/src
$ git clone https://github.com/mgonzs13/mmdetection_ros.git
$ cd ~/ros2_ws
$ rosdep install --from-paths src -r -y --ignore-src
$ colcon build
```

## Demo

```shell
$ ros2 launch mmdetection_bringup mmdetection.launch.py
```

```shell
$ wget https://download.openmmlab.com/mmdetection/v2.0/yolact/yolact_r101_1x8_coco/yolact_r101_1x8_coco_20200908-4cbe9101.pth
$ ros2 launch mmdetection_bringup mmdetection.launch.py network_config:=yolact_r101_1x8_coco_20200908-4cbe9101.pth
```
