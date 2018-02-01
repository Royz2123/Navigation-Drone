# Sport Drone

The current goal of this project is to create a drone with an onboard camera, that can stabilize itself in front of some predefined pattern.
We used a Syma X5C drone with a TX5805 camera, and a computer with a video reciever and Arduino based remote controller attached.
The computer ran this code.

This project was developed by Roy Zohar and Roy Mazan from the RBD Lab, under supervision of Dr. Dan Feldman

A special thanks also to Tamir Gadot, Eli Tarnatutsky and Andrey Leneshko who are always helping us out.


## Building the code

You will need a Linux computer with OpenCV, CMake and some C++ compiler installed.

To send commands to the Arduino you will need to add yourself to the 'dialout' group. Execute:

```bash
sudo usermod -a -G dialout YOUR_USERNAME
```

Before you can compile you have to run the following commands from the commandline:

```bash
mkdir build
cd build
cmake ..
```

Then, each time you want to compile and run the code, go to the build/ directory, and execute:

```bash
make
./sport_drone
```

## Handling the drone via keyboard commands

'q' - exit the program (and kill engines)

'x' - kill engines completely

'c' - restore engines based on recognition

' ' - pause

'0-9' - change camera mode
