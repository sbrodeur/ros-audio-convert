# ros-audio-convert
ROS (Robotic Operating System) packages for audio conversion based on the audio_common stack

## Installing dependencies

Install dependencies (Debian/Ubuntu):
```
sudo apt-get install libgstreamer1.0-0 libgstreamer1.0-dev libgstreamer-plugins-base1.0-0 libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly ros-kinetic-audio-common-msgs
```

## Compiling the package in a new catkin workspace

Create the catkin workspace:
```
source /opt/ros/kinetic/setup.bash
mkdir -p catkin_workspace
cd catkin_workspace
mkdir -p src
cd src
catkin_init_workspace
```

Download the source code from the git repository:
```
git clone https://github.com/sbrodeur/ros-audio-convert.git audio_convert
```

Compile workspace with catkin:
```
cd ..
alias catkin_make="catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS=-O3 -DCMAKE_CXX_FLAGS=-O3"
catkin_make
source devel/setup.bash
```

## Usage

Convert a single bag file to wav:
```
rosrun audio_convert bag2wav --input=audio.bag --output=audio.wav --input-audio-topic=/audio
```

Batch convert bag files to wav (recursive) using the provided script:
```
cd src/audio_convert/scripts
./convert_bag2wav_batch.sh <DIRECTORY>
```