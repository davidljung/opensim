# opensim - A 3D Simulator for autonomous robots

Most experimental AI/robotics research involves a simulation component. Typical simulations involve very simplified environments - often 2D environments with polygonal objects and circular robots. The aim of this project is to create a tool for higher-fidelity real-time simulations of autonomous mobile robots. Real-time simulation also allows hardware-in-the-loop simulations.

OpenSim is a 3D simulator that uses OpenGL for real-time rendering of the robot environment, including a physics engine to simulate dynamics in real-time (collision between arbitrary polyhedral objects with friction). 

<a href="url"><img src="https://raw.githubusercontent.com/davidljung/opensim/master/docs/viewenv_screen_small.png" width="440" ></a>
<a href="url"><img src="https://raw.githubusercontent.com/davidljung/opensim/master/docs/poscontrol_nonhol_platf.png" width="320" ></a>

__Note__: _OpenSim was actively developed between 2001-2006 and sporadically until around 2009.  It is no longer under active development, 
however, there is a moderate amount of useful C++ code that may be reused._

A handful of simple sensors are implemented. The current implementation also contains a component called IKOR (Inverse Kinematics Of Redundant systems) which can quickly solve the IK problem with constraints (holonomic & non-holonomic) for arbitrary kinematic systems (i.e. robot manipulators).

The current version contains terrain rendering and some primitive shapes using Open Scene Graph (OSG) and Demeter (OpenGL); an implementation of physics using Open Dynamics Engine (ODE) with collision detection/response; and some inverse kinematics solution code for redundant manipulators (IKOR). Everything is written in ISO C++07 (namespaces, exceptions, STL, etc.). The system compiles on Linux (using gcc 3.4.2).

There is also a <a href="https://github.com/davidljung/opensim/raw/master/docs/OpenSimManualv0.4.1.pdf">User &amp; Developer Guide (PDF)</a> available in the docs directory.

Feel free to scavange code. All the code is GPL'd. Contact me if you'd like to use the code in another project that uses a different OSS license. 
