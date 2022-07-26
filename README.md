# UcoSLAM-IBOIS
![](./example/tracking_demo.gif)

The modified version of [UcoSLAM](http://www.uco.es/investiga/grupos/ava/node/62) for augmented carpentry project, which we intregrat [STag](https://github.com/bbenligiray/stag) and add [CLAHE](https://en.wikipedia.org/wiki/Adaptive_histogram_equalization) for preprocessing.
- The official document of UcoSLAM:  [link](https://docs.google.com/document/d/12EGJ3cI-m8XMXgI5bYW1dLi5lBO-vxxr6Cf769wQzJc)
- [STag ROS](https://github.com/usrl-uofsc/stag_ros): Code of STag is from this repo. This one upgrade the original STag from OpenCV 3 to OpenCV 4.
- [Dev Log of this modification](./dev_log)

## Build
```bash
# install library
sudo apt-get install cmake libopencv-dev qtbase5-dev libqt5opengl5-dev libopenni2-dev

# build
mkdir build
cd build
cmake ../ -DBUILD_GUI=ON
make -j4
```

## Usage
### Run with example (monocular video):
- Mapping
```-
./build/utils/reslam_monocular './example/STag23mm_smallCube/mapping.mp4' './example/calibration_pixel3.yml' -aurco-markerSize 0.023 -voc './orb.fbow' -out example_map -noX
```
> [!] Vocabulary is not required, but it will not be able to relocalization with keypoints if not specified.

- Tracking
```
unzip ./example/STag23mm_smallCube/example.zip -d ./example/STag23mm_smallCube
./build/utils/reslam_monocular './example/STag23mm_smallCube/tracking.mp4' './example/calibration_pixel3.yml' -aurco-markerSize 0.023 -map './example/STag23mm_smallCube/example.map'
```

- Run GUI:
```
./build/gui/UcoSLAM_GUI
```

- Exporting tag map to yml:
```
./build/utils/reslam_map_export xxx.map out.(pcd|ply|yml)
```

## Others
- `/example`: An example video for mapping and tracking.
- `/post_processing/cluster.py`: Taking the tag map (the one exported by `reslam_map_export`) as input and perform 3d plane fitting.
- `/stag_util/sticker_generator.py`: Script for generating the STag sticker.

