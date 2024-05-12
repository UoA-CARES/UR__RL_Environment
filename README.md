# 
<h1 align="center">UR-5 Ball in Cup Environment</h1>


<div align="center">
  This repository contains the code to use RL on a UR5 robot with the traditional ball-in-cup game. The necessary package and basic instructions are presented.
  Before running this environment, ensure that the following requirements are met:

</div>

## 1. Clone and Install Python URX Package

1. Clone the Python URX repository from GitHub:
  ```
  git clone https://github.com/UoA-CARES/python-urx.git
  ```
2. Navigate into the cloned repository directory:
  ```
  cd python-urx
  ```
3. Install the Python URX package in editable mode using pip:
  ```
  pip install -e .
  ```

## 2. Install Math3d Package

1. Install the `math3d` package version 3.4.1:

```
pip install math3d==3.4.1
```

## 3. Install Pypylon 

1. Install this library, need for Basler Camera
```
pip install pypylon
```
 Connect the camera to a USB 3.0 port. 

## 3. Test the simulation

A helpful Docker file is available for testing ideas in simulation. Simulation provides a safe environment to experiment 
with new ideas without risking the real robot. So highly recommended to uses. In a new terminal run: 

```
sudo docker run --rm -it   -p 6080:6080 -e ROBOT_MODEL=UR5 -v "${HOME}/ur_programs:/ursim/programs" universalrobots/ursim_e-series
```

Then open a web browser and past the following link and press connect  

```
http://172.17.0.2:6080/vnc.html?host=172.17.0.2&port=6080
```

You will be able to view the control screen panel exactly as it appears on the real robot. Simply follow the same 
initialization setup used for the real robot. Make sure to use the right IP address in the robot_config.json.
This simulation will not replicate the entire RL environment. Specifically, the ball (or cube) will not be present 
in the simulation. It focuses solely on the robot itself, without any external environmental influences.

Run the provided `train.py` example to ensure everything is set up correctly:

```
python train.py
```


## 3. Test the real robot

 Turn on the real robot and follow the initialization instructions displayed on the control panel screen. 
 Ensure that the robot is in the safe operational area before running any code. Make sure to use the right IP address in 
 the robot_config.json 

**Be careful here; the robot will start to move**

 Run the provided `train.py` example to ensure everything is set up correctly:

```
python train.py
```


