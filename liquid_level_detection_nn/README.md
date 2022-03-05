# Liquid Level Detection NN

This package detects the level of liquid in a container, using an implementation of [Holistically-Nested Edge Detection](https://arxiv.org/abs/1504.06375) based off of code from [here](https://github.com/buntyke/pytorch-hed).  Since HED runs using pytorch, which uses python 3, while ROS uses python 2, the neural network is run in a docker image.

## Instalation

Install `docker`
https://docs.docker.com/install/linux/docker-ce/ubuntu/#install-docker-ce

```
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"

sudo apt-get update

sudo apt-get install docker docker-ce
```

Follow post installation steps
https://docs.docker.com/install/linux/linux-postinstall/

Install `nvidia docker`
https://github.com/NVIDIA/nvidia-docker

```
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -

curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu16.04/amd64/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update

sudo apt-get install -y nvidia-docker2

sudo pkill -SIGHUP dockerd

```

Install CUDA

```
sudo apt-get install nvidia-cuda-toolkit
```

Navigate to `liquid_level_detection_nn/docker`.  Run `./build.sh` to build the docker image.

## Usage

Stop the non nn liquid level detection
`rosnode kill /Level_Detector_Node_cpp`

Start the node that converts the nn output into a height
`roslaunch liquid_level_detection_nn liquid_level_detect.launch`

Start the docker image and its wrapper
`rosrun liquid_level_detection_nn detector.py`

Wait for the detector node to print `hed_detector initialized` before attempting to process images