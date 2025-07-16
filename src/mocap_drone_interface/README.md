# Motive2Ros

Motive2Ros is a Ros 2 Humble Hawksbill package designed to assist communication with drones. 

## Installation

Note: This package depends on the Cross Platform NatNetSDK from [Whoening on Github.](https://github.com/whoenig/NatNetSDKCrossplatform) The files "NatNetClient.py", "DataDescriptions.py", and "MoCapData.py" from /samples/PythonClient must go in the /library folder, and their mutual imports may have to be altered to the filepath motive2ros.library.{file name}.

Navigate to the /src folder of your Ros workspace. Clone the repo in the space and colcon build it like any other ros package.
```bash 
git clone https://github.com/benfox0621/motive2ros.git
 colcon build --packages-select motive2ros
```

## Usage

Currently not much functionality is available. The main functions are limited to a talker and a subscriber for catching unicast optitrack data, and filtering by rigid body id. 

There are default arguments set for the AIO node publisher and subscriber classes, but they can be called in a script using normal snytax. Once the scripts are written, colcon build the package again and using the command
```bash
ros2 run motive2ros talker
```
in one terminal, and 
```bash
ros2 run motive2ros listener --ros-args -p id:={streaming_id}
```
in another. {streaming_id} is where the streaming id from motive goes, listed as an integer value. 