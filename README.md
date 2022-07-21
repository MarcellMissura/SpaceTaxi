# SpaceTaxi
This is a simulation environment with a graphical user interface that contains the implementations of the STAA*, DWA, and a PD controllers for nonholonomic agents and also the Minimal Construct path planning algorithm. It is based on Qt the framework for the graphical user interface and uses the Box2D physics engine (https://box2d.org) for simulating the motion of the vehicles.

# Compilation

After cloning the project, change into the root directory of the project and type

qmake
make
./SpaceTaxi

This should compile and open the main window of the simulator.
Press enter to start the simulation.

For the compilation to work, you need to have following packages installed (tested on Ubuntu 18.04.6 LTS):
qt5-default
libpolyclipping22 and libpolyclipping-dev
libopencv-core3.2 and libopencv-imgproc3.2
libgl1-mesa-dev and libglew

