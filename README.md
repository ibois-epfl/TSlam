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

## Run with example (monocular video):
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

## APIs
```cpp
#include "ucoslam.h"
```
### Class `ucoslam::UcoSlam`
The main interface.
```c++
void setParams(std::shared_ptr<Map> map, const ucoslam::Params &params, const std::string &vocabulary="");
```
- Initialize the UcoSLAM.
- Params:
    - `std::shared_ptr<Map> map`: The map.
    - `const ucoslam::Params &params`: The system parameter (not the camera parameter).
    - `const std::string &vocabulary`: Path to the `.fbow` file.

```cpp
cv::Mat process(cv::Mat &in_image, const ImageParams &ip, uint32_t frameseq_idx);
```
- Process the frame and return the camera position.
- Params:
    - `cv::Mat &in_image`: The frame.
    - `const ImageParams &ip`: An initialized camera parameter.
    - `uint32_t frameseq_idx`: Input frame counter.
- Return:
    - `cv::Mat cameraPose`:
        - If tracked: A 4x4 transformation matrix looks like this:
            ```
            [ 0.936, -0.247,  0.250, 14.770;
             -0.236, -0.968, -0.073, -0.522;
              0.261,  0.009, -0.965, 43.664;
                  0,      0,      0,      1]
            ```
        - If not tracked: empty `cv::Mat`.


### Class `ucoslam::ImageParams`
Storing the [camera matrix and distortion coefficients](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
```c++
void readFromXMLFile(std::string filePath);
```
- Read from file (but .yml instead of .xml(?)
- Params:
    - `std::string filePath`: File path.
- Example file:
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

### Class `Params`
The system parameters.
```c++
void readFromYMLFile(const std::string &path);
void saveToYMLFile(const std::string &path);
```
- Read/save the params from/to a file.
- Params:
    - `std::string path`: Read/save path.
- Parameters we may use:
    - `aruco_markerSize`: `float`, size of the marker in the real-world.
    - `isInstancing`: `bool`, indicate if it's mapping/instancing.

### Map
```cpp
void readFromFile(std::string fpath);
void saveToFile(std::string fpath);
```
- Read/save the (binary) map from/to a file.
- Params:
    - `std::string fpath`: Read/save path.

```cpp
void saveToMarkerMap(std::string filepath)const;
```
- Saves the set of markers to a marker map file (.yml) that can be used with aruco.
- Params:
    - `std::string filepath`: Save path, should include the extension (.yml)

```cpp
void merge(std::shared_ptr<Map> mapB);
```
- Merge mapB into this.
- Params:
    - `std::shared_ptr<Map> mapB`: Another map

```cpp
void optimization(int niters=50);
```
- Run fullba optimization.
- Params:
    - `niters`: Number of iterations(default = 50).

## Other
- `/example`: An example video for mapping and tracking.
- `/post_processing/cluster.py`: Taking the tag map (the one exported by `reslam_map_export`) as input and perform 3d plane fitting.
- `/stag_util/sticker_generator.py`: Script for generating the STag sticker.

