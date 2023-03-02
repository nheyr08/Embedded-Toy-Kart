# Embedded-Toy-Kart

###### tags: `2021Fall` `reinforcement learning`

Ai kart is a project that try to implement a self driving AI kart game.
This file is a result of the brainstorming behind the group semester project.

# Issues
## How to build a model for simulation
Gazebo or unity -> 10/27 Gazebo
- [x] done

1. Learning curve is lower
2. We only need simple geometric shape for the scence. Our primary concern is the joints and control etc.

    # Steps    
## Algorithm to train the model to play the game:
reinforcement.
## Find out how to interact with the computer and the board
Sending Keystrokes to a PC using a raspberry pi/ Jetson
## find target: KSP, track..., what is the game the model will play
car race, landing ...

# Presentation of Ideas
* Initiative
> Want to do embedded control but have no hardware, want to play multiplayer games but have no opponents - 10/27

* info
5 min presentation
3 min QA sesssion

11/10 or 11/17

# Notes
## Environment
Maybe use Ignition Fortress on both side
Ignition Gazebo: new, more suitable for RL, but lack of some feature and tutorial 
### Virtual
- Robostack 
    1. cross-plateform ROS maintained in conda
    2. [Document](https://robostack.github.io/noetic.html), [Github](https://github.com/RoboStack/ros-noetic)
- Ignition-Gazebo
    1. better Gazebo for RL and physic engine
    2. [Installation guild (conda)(gazebo, libignition-gazebo6)](https://www.youtube.com/watch?v=CY-_Yvu0ya0)
    3. [Installation guide(official)(ignition fortress)](https://github.com/ignitionrobotics/ign-gazebo/releases/tag/ignition-gazebo6_6.0.0)
- ScenarIO & gym-ignition
    - RL assist library paired with Ignition Gazebo
    - train with only python code importing ignition as a library, ScenarIO as backhand. While implementation, we can only change backhand to ROS, and still keep the code and train.
    -  RL package for Ignition-Gazebo may with some issue on sensor data extraction [ref](https://github.com/robotology/gym-ignition/issues/199)
    2. [Document](https://robotology.github.io/gym-ignition/devel/index.html)
    3. [Paper](https://arxiv.org/pdf/1911.01715.pdf) introduce gym_ignition, and compare with other simulator
    4. [Interview Video with developer](https://www.youtube.com/watch?v=whiExLlAbvU&t=1186s)
    5. [More with Developer](https://www.theconstructsim.com/86-reproducing-robotic-simulations-for-reinforcement-learning-with-gym-ignition/)
- ros-ign
    1. plugin associates ROS and Ignition Gazebo
    2. [Github](https://github.com/ignitionrobotics/ros_ign)

### Jetson Nano
- Linux aarch64 
- Ubuntu 18.04.6 LTS (Bionic Beaver)
- Works for Gazebo, but fail on Ignition Gazebo ?

## PyBullet
[Quick Start](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3)
[Track Example](https://github.com/axelbr/racecar_gym)
[Visual Example](https://ai.aioz.io/guides/robotics/2021-05-19-visual-obs-pybullet/)

Racecar example : [here](https://skywalker0803r.medium.com/pybullet-%E6%96%B0%E6%89%8B%E5%85%A5%E9%96%80-7919feb08aba)
https://blog.otoro.net/2017/11/12/evolving-stable-strategies


## URDF(universal robot description file) 
http://wiki.ros.org/urdf/Tutorials

link, joint, and material 

joint’s origin is defined in terms of the parent’s reference frame.
Since we didn’t specify a rpy (roll pitch yaw) attribute, the child frame will be default have the same orientation as the parent frame.

Online URDF viewer : [here](https://mymodelrobot.appspot.com/)

demo code [here](https://zhuanlan.zhihu.com/p/347177629)

Formula Pi [here](https://www.wilselby.com/formulapi-ros-simulation-environment/)

# Resource
* Machine learnng to control game class
https://hackmd.io/@Dingjunzhe/H1-JJtXSI

* restoring you usb key paritition after you set it as bootable
https://www.pendrivelinux.com/restoring-your-usb-key-partition/

* controle from external source KSP case
https://www.instructables.com/Kerbal-Space-Program-Controller/

* reddit on Gazebo vs Unity
https://www.reddit.com/r/robotics/comments/9thahr/gazebo_vs_unity_for_neural_network_controlled/

* Robostack
https://github.com/RoboStack
https://medium.com/robostack/cross-platform-conda-packages-for-ros-fa1974fd1de3
https://arxiv.org/pdf/2104.12910.pdf


* Open ai gem + ROS
https://youtu.be/o9FpZ1QQqcc

* Ignition Gazebo
https://github.com/ignitionrobotics/ign-gazebo
https://github.com/robotology/gym-ignition
https://ignitionrobotics.org/home
* Ros Tutorial
https://robocademy.com/2020/04/28/top-free-tutorials-to-learn-ros-robot-operating-system/


* socket example
https://ithelp.ithome.com.tw/articles/10205819
___________________




## Step towards self driving car/robot
 
This [method](https://github.com/harsha-20/Autopilot-Steering-Wheel-Simulation) uses CNN to map the raw pixels from a front-facing camera to the steering commands for a self-driving car. 
However, since we are using pybullet, it is said somewhere in the [documentation](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.u1jisfnt6984) that we can directly retrieve the camera output from pybullet.getCameraImage and use them for further purposes like training our model. but Im not sure if it can read through to opencv.

* Our main issue now, (if are to proceed with this approach) would be to find a way to connect the pybullet camera output for our robot, to the opencv input and train our model (I also couldnt find reliable way to do this yet)



Alternatively, I noticed in the documentation that pybullet provide some reinforcement learning trick to train our model [here](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.wz5to0x8kqmr) still working on figuring out those.

* And for the texture part I found out that it was not possible to do the texture wrapping on a non mesh object like the car and track we had from the URDF files, the way would be to use blender or some other process, but we are not there yet.

# Gym environment 
[DQN car](https://gerardmaggiolino.medium.com/creating-openai-gym-environments-with-pybullet-part-2-a1441b9a4d8e)


# NEAT algorithm
Practical introduction
https://m.youtube.com/playlist?list=PLzMcBGfZo4-lwGZWXz5Qgta_YNX3_vLS2


# PyQt GUI interface
- [x] Layout, second screen
- [ ] Multithread to start the game
- [ ] Make sure the TCP part is ok
- [ ] Good way to quit the game
- [ ] Speed indicator
- [ ] Add image, video, sound etc.
- [ ] User data storage

### Note
Qt Designer is platform and programming language independent. It doesn’t produce code in any particular programming language, but it creates .ui files. These files are XML files with detailed descriptions of how to generate Qt-based GUIs.

Widgit bar
Object Inspector 
Property editor
Action editor
Resource browser
signal/slot editor

Switch between:
Widget editing mode
signal/slot mode
buddies mode
tab order

pyuic5 change .ui to .py 
keyboard accelerators

> uddies refers to a special relationship between a label and a widget in which the label provides a keyboard accelerator or shortcut that allows you to access the buddy widget using your keyboard.

I suppose it's some kind of short key


widgit send signal to slot that have event triggered

pyuic5, which is a tool included in the PyQt installation

### QThread
The application’s main thread always exists. This is where the application and its GUI run. On the other hand, the existence of worker threads depends on the application’s processing needs.


If you create an object from any class that inherits from QObject in a particular thread, then that object is said to belong to, or have an affinity to, that thread. Its children must also belong to the same thread.

In step 1, you create Worker, a subclass of QObject. In Worker, you create two signals, finished and progress.

* communication
Mutual exclusion is a common pattern in multithreaded programming. Access to data and resources is protected using locks, which are a synchronization mechanism that typically allows only one thread to access a resource at a given time.

     A thread-safe object is an object that can be accessed concurrently by multiple threads and is guaranteed to be in a valid state. PyQt’s signals and slots are thread safe, so you can use them to establish interthread communication as well as to share data between threads.
     
    add mutex.lock() and mutex.unlock() on each modification

### Question
How exactly do we put our py implementation to call two window-> Answer: https://stackoverflow.com/a/13551018


Pybullet, disable mouse picking
configureDebugVisualizer
[here](https://usermanual.wiki/Document/pybullet20quickstart20guide.479068914.pdf) p.59

carenv as worker function
carenv update global variable car_speed, car_rank
run NEAT model independent from Openai gym



### Resource
* color/image
https://stackoverflow.com/questions/43282899/qt-designer-how-to-change-background/43283147

* Thread

* stylesheet
https://stackoverflow.com/questions/62497077/pyqt5-changing-qpushbutton-stylesheet-when-hovering-over-it

* set the location of the window on the screen
https://pythonprogramminglanguage.com/pyqt5-center-window/

* sample pyqt 
https://github.com/realpython/materials/tree/master/qt-designer-python/sample_editor


* speed meter
https://github.com/StefanHol/AnalogGaugeWidgetPyQt

*  audio
https://youtu.be/tF1U93I3-90

--- 

