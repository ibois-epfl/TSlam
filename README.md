# TSlam
![](./example/tracking_demo.gif)

This is a modified version of [UcoSLAM](http://www.uco.es/investiga/grupos/ava/node/62) for augmented carpentry project. The main features are:
- Using [STag](https://github.com/bbenligiray/stag)
- Can do map fusion (merging one map into another).
- By indicating it's in instance mode, the system stop running global optimization and only keeps a fixed number of new added keyframes, which smooth the overall experience.
- Add [CLAHE](https://en.wikipedia.org/wiki/Adaptive_histogram_equalization) for preprocessing.
- Several [bugs are resolved](./dev_log/Bug_tracing.md).

Here are some reference related to this project:
- The official document of UcoSLAM:  [link](https://docs.google.com/document/d/12EGJ3cI-m8XMXgI5bYW1dLi5lBO-vxxr6Cf769wQzJc)
- [STag ROS](https://github.com/usrl-uofsc/stag_ros): Code of STag is from this repo. This one upgrade the original STag from OpenCV 3 to OpenCV 4.
- [Dev Log of this modification](./dev_log)

## Build
```bash
# install library (compile from source opencv 4.5.5)
sudo apt-get update
sudo apt-get install cmake qtbase5-dev libqt5opengl5-dev libopenni2-dev
git clone https://github.com/ibois-epfl/TSlam.git
cd TSlam

mkdir build && cd build
cmake ../ -DBUILD_4_API=OFF  # (ON: if building as API with no GUI)
make -j$(nproc)

sudo make install  # if building as API
```

## Run with examples(monocular video):
### Minimal example:
```bash
cd build/utils
unzip ../../example/example.map.zip -d ../../example/
./tslam_minimal_example
```
This runs `tslam_minimal_example.cpp`, which takes the `example.map` and `video.mp4` in `/example` as input to show a brief demo.

### Mapping
```bash
./tslam_monocular ../../example/video.mp4 ../../example/calibration_webcam.yml -voc ../../orb.fbow -out test
```
This runs `monocular_slam.cpp`.
- `2nd argument`: Input source, "live" indicates the camera[0].
- `3rd argument`: Path to the camera parameter file.
- `-voc`: Path to vocabulary file.
- `-out`: Name of the output map, it will be saved with extension `.map` (in this case, `test.map`)
Run without args to check all the avaliable ones.

When the window pop-out, you can press `f` to enable the 3D tracking and `s` to start/stop the video.

> [!] Vocabulary is not required, but it will not be able to relocalization with keypoints if not specified.

### Tracking
```bash
# first unzip the map
unzip ../../example/example.map.zip -d ../../example/
./tslam_monocular ../../example/video.mp4 ../../example/calibration_webcam.yml -map ../../example/example.map -isInstancing
```
- `map`: Path to the map.
- `isInstancing`: Indicating it's not mapping, so we can skip some operations.

### Run GUI:
```
./build/gui/UcoSLAM_GUI
```

## APIs
### Header
All interface is included in `tslam.h`.
```cpp
#include "tslam.h"
```

---

### Class `TSlam`
#### Initilization
```c++
tslam::TSlam *slam = new tslam::TSlam;

slam->setMap("long_new_param_comb.map");

// If already set map, the vocabulary can be skipped
slam->setVocabulary("../../orb.fbow");

// Indicate if it's 
slam->setInstancing(true);

// Set path to the camera calibration matrix.
// The yml file structure is described below.
slam->setCamParams("../../example/calibration_webcam.yml");
```
- Example `calibration_webcam.yml`:
    ```yml
    %YAML:1.0
    ---
    image_width:852
    image_height:480
    camera_matrix: !!opencv-matrix
        rows: 3
        cols: 3
        dt: f
        data: [832.069, 0, 424.286, 0, 831.192, 242.936, 0, 0, 1 ]
    distortion_coefficients: !!opencv-matrix
        rows: 1
        cols: 5
        dt: f
        data: [0.274109, -1.71439, 0.00250987, 5.0718e-05, 3.46554 ]
    ```

#### Functions
```c++
cv::Mat getLastTrackedCamPose();
```
- Get the last tracked camera pose (not guarentee to be the last frame)
- **Return:** A 4x4 cv::Mat; cv::Mat::eye if not tracked ever.

```c++
bool process(cv::Mat frame, cv::Mat &camPose);
```
- Process a frame.
- **Params:**
    - `cv::Mat frame`: Frame to process.
    - `cv::Mat &camPose`: The reference will be updated to the camera pose of the frame.

```c++
std::shared_ptr<Map> getMap();
```
- Get the pointer to the map being used
- **Return:** A `std::shared_ptr<Map>` point to the map in use.

---

### Class `Map`
```cpp
void saveToFile(std::string fpath);
```
- Save the (binary) map to a file.
- **Params:**
    - `std::string fpath`: Read/save path.

```cpp
void saveToMarkerMap(std::string filepath)const;
```
- Saves the set of markers to a marker map file (.yml) that can be used with aruco.
- **Params:**
    - `std::string filepath`: Save path, should include the extension (.yml)

```cpp
void merge(std::shared_ptr<Map> mapB);
```
- Merge mapB into this.
- **Params:**
    - `std::shared_ptr<Map> mapB`: Another map

```cpp
void optimization(int niters=50);
```
- Run fullba optimization.
- **Params:**
    - `niters`: Number of iterations(default = 50).

## Other
- `/example`: An example video for mapping and tracking.
- `/post_processing/cluster.py`: Taking the tag map (the one exported by `reslam_map_export`) as input and perform 3d plane fitting.
- `/stag_util/sticker_generator.py`: Script for generating the STag sticker.

## License
GPLv3