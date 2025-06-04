<img src="./Wiki_images/img/banderole.png" width="800" />

# GRiSP2 Robot

## Introduction

This project is an extension of the master thesis made by Francois Goens and Cédric Ponsard (link [here](https://github.com/FrancoisGgg/balancing_robot) (2023-2024)). It is done in the context of a new master thesis by Nicolas Daube and Thomas Vanbever (2024-2025). Both of these master thesis where done under the supervision of Prof. Peter Van Roy at UCLouvain. The goal of this project is to use [GRISP2 boards](https://github.com/grisp/grisp) (designed and comercialised by [Peer Stritzinger GMBH.](https://stritzinger.com/)) and the [Hera](https://github.com/Nicodaube/hera) framework to create a robot capable of standing upright (last master thesis) and to know its own position in an assembly of rooms (current project). The robot, and its associated sensors, need to know at all time the position of the robot in the system of rooms. Each sensor knows all the data sent by each sensors and by the robot. An other goal of this project is thus to allow each sensor to compute the position of the robot (using a Kalman Filter) at a reasonable rate. This implies that the filter must evolve over time, using the data given by the right sensors at the right time.

The project is done using Erlang OTP (current version 27.2.4). If you want more informations about the, project, visit our the [Wiki](https://github.com/Nicodaube/Grisp_robot/wiki) section of this repository.

## Organisation

This repository is organised in several subdirectories :

* **Hardware_robot** : contains all the necessary files to build the robot. In `Hardware_robot/printing_files` you will find all the files for the prints in 3D and the laser cuts.
* **Hardware_sensor** : Contains all the necessary files to build the sensors and their supports.
* **Laptop_controller** : contains all the code concerning the python server and controller for the robot. Note that its use is mandatory to have a fully functional system.
* **LilyGo_software** : contains all the code for the two LilyGO used in this project. They are used for communication between the robot and the controller.
* **Wiki_image** : just a directory containing all the wiki and README images.
* **balancing_robot** : contains all the code for the auto-stabilized robot.
* **electronics**: contains the files concerning the custom PCB designed for the robot.
* **sensor**: contains all the erlang code for the implementation of the sonar sensors.

