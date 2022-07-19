`June 20`
- Replaced 3rd party library [ArUco](https://www.uco.es/investiga/grupos/ava/node/26) to version `3.1.15` to avoid the uglified code in the original repo.
- Added `Stag` in this repo, edited `CMakeLists.txt`, `cmake/dependencies.cmake`, and `cmake/options.cmake` to make it work.

## Note of code
- Markers are detected in `src/utils/frameextractor.cpp`, line 670
```cpp
auto markers=_mdetector->detect(Iinfo.im_org); // Iinfo.im_org is cv::Mat
```
- Marker's attribute (in `src/utils/markerdetector.h`):
```cpp
// unique marker id
uint32_t id;

// three dimentional points of the marker wrt its center
// (This is something like [-0.5 * marker_size, 0.5 * marker_size, 0 ... ])
std::vector<cv::Point3f> points3d;

// original corners in the image
std::vector<cv::Point2f> corners;

// optional info about the marker
// (This attribute is probably not used)
std::string info;
```

## Now it works with STag
![](./ucoslam_with_stag.png)
![](./demo.gif)
- However, the tracking of feature points disappears at some point, not sure why.
    > This problem has been figured out. It's probably because the  vocabulary is no specified. Also, the unconsistency of the marker size may be the reason that cause the distortion.

---

`June 4`
- I tried to used the `-nokeypoints` flag to map the timber, but the program would crash. The solution is to downgrade the UcoSLAM from 1.2.4 to 1.1.0, which is tagged "stable" in the releasing website. However, the result is unpromising (worse than the result with key points).
![](./mapping_nokeypoints.png)

- Tried with smaller object and slightly bigger tags (2cm vs 2.3cm), works pretty well!
![](./mapping_nokeypoints_good.png)

- Debug command
`gdb --args ./ucoslam_monocular '/home/tpp/UCOSlam-IBOIS/result/STag23mm_smallCube/use.mp4' '/home/tpp/UCOSlam-IBOIS/result/calibration_pixel3.yml' '-map' '/home/tpp/UCOSlam-IBOIS/result/STag23mm_smallCube/markers.map' "-aruco-markerSize" "0.023"`

## UcoSLAM 1.2.4 vs 1.1.0
| Function              | 1.2.4 | 1.1.0 |
| --------------------- | :---: | :---: |
| Mapping w/o keypoints |   X   |   O   |
| Mapping w/ keypoints  |   O   | slow  |
| Tracking              |   Yes, but Not w/ GUI   | X    |
> So now, we go back to 1.2.4.
> This table may be wrong :P

- Contrast augmentation is added
    - With augmentation:
      ![](./with_aug.png)
    - Without augmentation:
      ![](./with_out_aug.png)

## Failure
- It lost tracking (the camera position is not recovered) when keypoints are not matching (but marker is detected). This should be fixed.

  ![](./lost_tracking.gif)

## Remove Tag
- Testing on removing some tags (simulating cutting)

  ![](./tracking_remove_tag_out.gif)

## Others
- Done the sticker generation (`stag_util/sticker_generator.py`)
- Post processing now exports an .ply

## Testing with the sticker
![](./sticker.gif)

## TODO List
- [ ] Enhancement / preprocessing
- [ ] CMake on the main project
- [ ] Rewrite the map post-processing code to C++

## Bugs to deal with
### The lags
- Seems to come out when inconstency introduced in the scene, I guess it's doing optimization on the map.
    - During Mapping
        - ✔️ 2-phase mapping, which reduce this situation, but may still happen
            > Made a python script to do the experiment. It should be converted into the C++ version later.
        - Move the code cause the lag into another thread, and let the main thread keep going => but this probably causes problem
        - This mostly happens when the timber is moved, so
            - Mapping with only the markers: tried, the accuracy is not good
            - Sementic segmentaion which get rid of the background

    - During intance
        - Not updating Map -> lower accuracy
        - Move the code cause the lag into another thread, and let the main thread keep going
        - ✔️ Just cancel it
            > This has been tackled down by adding a system-scale parameter "-isInstancing", which disables the optimization process.

### Bug #1
- This is actually a bug caused by OpenCV. [Link to the issue](https://github.com/opencv/opencv/pull/19253)
- The problem should be solve by upgrading OpenCV to 4.5.5 (actually, we don't have to switch to 5.0.0!) and setting a flag when calling cv::solvePnPRansac(), in `src/utils/loopdetector.cpp`. This uses another algorithm (USAC), which also performs better than RANSAC.
- Available Flags: [Link to the file on Github](https://github.com/opencv/opencv/blob/2a4926f4178681306999cfb04f6de601ec12f47b/modules/calib3d/include/opencv2/calib3d.hpp)
```
//! type of the robust estimation algorithm
enum { LMEDS  = 4,  //!< least-median of squares algorithm
       RANSAC = 8,  //!< RANSAC algorithm
       RHO    = 16, //!< RHO algorithm
       USAC_DEFAULT  = 32, //!< USAC algorithm, default settings
       USAC_PARALLEL = 33, //!< USAC, parallel version
       USAC_FM_8PTS = 34,  //!< USAC, fundamental matrix 8 points
       USAC_FAST = 35,     //!< USAC, fast settings
       USAC_ACCURATE = 36, //!< USAC, accurate settings
       USAC_PROSAC = 37,   //!< USAC, sorted points, runs PROSAC
       USAC_MAGSAC = 38    //!< USAC, runs MAGSAC++
     };
```
#### Original Error Message
``` cpp
  what():  OpenCV(4.2.0) /home/tpp/Downloads/opencv-4.2.0/modules/calib3d/src/calibration.cpp:1171: error: (-2:Unspecified error) in function 'void cvFindExtrinsicCameraParams2(const CvMat*, const CvMat*, const CvMat*, const CvMat*, CvMat*, CvMat*, int)'
> DLT algorithm needs at least 6 points for pose estimation from 3D-2D point correspondences. (expected: 'count >= 6'), where
>     'count' is 5
> must be greater than or equal to
>     '6' is 6

/////////////////////////////////////

10 0x00007ffff7b3b122 in cvFindExtrinsicCameraParams2 () at /usr/local/lib/libopencv_calib3d.so.4.2
#11 0x00007ffff7c5a922 in cv::solvePnPGeneric(cv::_InputArray const&, cv::_InputArray const&, cv::_InputArray const&, cv::_InputArray const&, cv::_OutputArray const&, cv::_OutputArray const&, bool, cv::SolvePnPMethod, cv::_InputArray const&, cv::_InputArray const&, cv::_OutputArray const&)
    () at /usr/local/lib/libopencv_calib3d.so.4.2
#12 0x00007ffff7c5d7d4 in cv::solvePnP(cv::_InputArray const&, cv::_InputArray const&, cv::_InputArray const&, cv::_InputArray const&, cv::_OutputArray const&, cv::_OutputArray const&, bool, int) () at /usr/local/lib/libopencv_calib3d.so.4.2
#13 0x00007ffff7c5f615 in cv::solvePnPRansac(cv::_InputArray const&, cv::_InputArray const&, cv::_InputArray const&, cv::_InputArray const&, cv::_OutputArray const&, cv::_OutputArray const&, bool, int, float, double, cv::_OutputArray const&, int) ()
    at /usr/local/lib/libopencv_calib3d.so.4.2
#14 0x00007ffff7ee0da1 in ucoslam::LoopDetector::_8671179296205241382(ucoslam::Frame&, int) ()
#15 0x00007ffff7ee2280 in ucoslam::LoopDetector::detectLoopFromKeyPoints(ucoslam::Frame&, int) ()
    at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
#16 0x00007ffff7f056d6 in ucoslam::MapManager::_12295639104386009589() () at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
#17 0x00007ffff7f06900 in ucoslam::MapManager::_8669746328630631075() () at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
```

### Bug #2
- A workaround is added at `src/optimization/pnpsolver.cpp:205` to check if the index is not exceed the size of the array.
```cpp
if (map_matches[i].trainIdx >= TheMap->map_points.data_size()) return 0;
```

- It should be act as if it doesn't find any matched points in that frame and skip it.



#### Original Error Message
```cpp
Thread 1 "ucoslam_monocul" received signal SIGSEGV, Segmentation fault.
#0  0x00007ffff7f8c64e in ucoslam::PnPSolver::solvePnp(ucoslam::Frame const&, std::shared_ptr<ucoslam::Map>, std::vector<cv::DMatch, std::allocator<cv::DMatch> >&, ucoslam::se3&, long) () at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
#1  0x00007ffff7f16e06 in ucoslam::System::_11166622111371682966(ucoslam::Frame&, ucoslam::se3) ()
    at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
#2  0x00007ffff7f1b996 in ucoslam::System::process(ucoslam::Frame const&) () at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
#3  0x00007ffff7f1bd45 in ucoslam::System::process(cv::Mat&, ucoslam::ImageParams const&, unsigned int, cv::Mat const&, cv::Mat const&) ()
    at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
#4  0x00007ffff7e77960 in ucoslam::UcoSlam::process(cv::Mat&, ucoslam::ImageParams const&, unsigned int) ()
    at /home/tpp/UCOSlam-IBOIS-1.1.0/build/libs/libucoslam.so.1.1
#5  0x0000555555568837 in main ()
```

### Bug #3
- Like #2, a workaround is added at `src/basictypes/picoflann.h:562` to check before access the vector.
```cpp
if (currNode.idx[i] >= container.size()) continue; 
```
- Program stops unexpectedly with no error output, don't know why.
- vector::_M_range_check: __n (which is 1488514608) >= this->size() (which is 0)


### Bug #4
```cpp
OpenCV(4.5.5-dev) /home/tpp/opencv/modules/core/src/matrix_expressions.cpp:32: error: (-5:Bad argument) One or more matrix operands are empty. in function 'checkOperandsExist'

^C
Thread 1 "ucoslam_monocul" received signal SIGINT, Interrupt.
0x00007ffff48d6fcb in __GI___select (nfds=4, readfds=0x7fffffffbd60, writefds=0x0, exceptfds=0x0, timeout=0x7fffffffbd30)
    at ../sysdeps/unix/sysv/linux/select.c:41
41      ../sysdeps/unix/sysv/linux/select.c: No such file or directory.
(gdb) backtrace
#0  0x00007ffff48d6fcb in __GI___select (nfds=4, readfds=0x7fffffffbd60, writefds=0x0, exceptfds=0x0, timeout=0x7fffffffbd30)
    at ../sysdeps/unix/sysv/linux/select.c:41
#1  0x00007ffff7a5412b in cv::CvCaptureCAM_V4L::tryIoctl(unsigned long, void*, bool, int) const [clone .constprop.0] ()
    at /usr/local/lib/libopencv_videoio.so.405
#2  0x00007ffff7a59cc8 in cv::CvCaptureCAM_V4L::read_frame_v4l2() () at /usr/local/lib/libopencv_videoio.so.405
#3  0x00007ffff7a5a8b9 in cv::CvCaptureCAM_V4L::grabFrame() () at /usr/local/lib/libopencv_videoio.so.405
#4  0x00007ffff7a20f3e in cvGrabFrame () at /usr/local/lib/libopencv_videoio.so.405
#5  0x00007ffff7a2257b in cv::VideoCapture::grab() () at /usr/local/lib/libopencv_videoio.so.405
#6  0x00005555555c6cb8 in InputReader::grab() (this=0x7fffffffcbf0) at /home/tpp/UCOSlam-IBOIS/utils/inputreader.cpp:94
#7  0x00005555555c6f3e in InputReader::operator>>(cv::Mat&) (this=0x7fffffffcbf0, image=...)
    at /home/tpp/UCOSlam-IBOIS/utils/inputreader.cpp:68
#8  0x000055555556a3e7 in main(int, char**) (argc=<optimized out>, argv=<optimized out>)
    at /home/tpp/UCOSlam-IBOIS/utils/monocular_slam.cpp:560
```

## Merge 2 maps
- Takes two tag maps (the exported .yml file) as input
- Estimate the affine transformation matrix using OpenCV
- 
### Input mask
| Map 1 | Map 2 |
| :---: | :---: |
| ![](./merge_1.png) | ![](./merge_2.png) |

### Merged Result
![](./merge_result.png)


## Speed Up During Instancing
- During instancing, key frames will be keep adding into the map, causing the FPS drop after a long run.
- In `src/utils/mapmanager.cpp:1594`, the function `set<uint32_t> MapManager::keyFrameCulling()` is modified. During instancing, the system will keep a fixed amount of number of new inserted keyframes, which is mainly used when the camera is going close to the timber and can't see a marker anymore.
- In the test case, where the original map has 220 key frames and 20 new inserted key frames are preserved (which results in a total of 240 key frames), the FPS remains ~40 after 10 mins.
    ```
    Image 33000 fps=36.002  21.0113 draw=53.9523 tracked
    Image 33100 fps=47.5574 24.0895 draw=52.6297 tracked
    Image 33200 fps=37.6424 21.3407 draw=52.7255 tracked
    Image 33300 fps=41.806  22.4592 draw=52.3648 tracked
    Image 33400 fps=40.5782 21.8395 draw=50.4677 tracked
    Image 33500 fps=45.1767 22.8656 draw=49.971  tracked
    Image 33600 fps=43.7453 22.5114 draw=49.983  tracked
    Image 33700 fps=36.9097 20.708  draw=51.1201 tracked
    Image 33800 fps=42.6364 22.2163 draw=50.3684 tracked
    Image 33900 fps=40.427  22.5163 draw=55.1343 tracked
    Image 34000 fps=41.9248 22.2568 draw=51.4437 tracked
    ```