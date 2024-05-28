# #Move-Hackathon2024
Before run Framework, sure that you installed ROS (Noetic) and Carla (version >= 0.9.13).

# Initial
Step1: Build ros package 02_RosCommunication -> source to this package (run devel\setup.bat to link package with terminal)

Step2: In the same terminal, build ros package 01_RosBridge -> source to this package (this source step is important, need do this first at any terminal).

For Carla has different version (not 0.9.13), you must change this in 01_RosBridge/src/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION file. Then build again.

## Start:
1. Launch Carla server.

2. Make sure you source to ros package at step 2 in all Ros Terminal.

3. Ros Terminal 1 (Ros Bridge):
    
        roslaunch carla_ros_bridge carla_ros_bridge.launch

4. Ros Terminal 2 (Client):
    
        python /03_Client/main.py
    
5. Ros Terminal 3 (Dev code):
   
        python /04_Template/main.py

## Move API:

You can get template in /04_Template

## Link support debug:

    https://drive.google.com/drive/folders/1jWq_q5UNM6qG1bJA6EwZWt8-pFcCHXVC
