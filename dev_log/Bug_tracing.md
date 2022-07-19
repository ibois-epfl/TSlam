# Bug Tracing
- Here records all the bugs we meet so far.

## Bug #1 [Solved] 
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

## Bug #2 [Solved]
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

## Bug #3 [Testing]
- Like #2, a workaround is added at `src/basictypes/picoflann.h:562` to check before access the vector.

```cpp
if (currNode.idx[i] >= container.size()) continue; 
```
- Program stops unexpectedly with no error output, don't know why.
- vector::_M_range_check: __n (which is 1488514608) >= this->size() (which is 0)

## Bug #4 [Testing]
- In `src/utils/system.cpp:3385`, a check is performed beforehand.

#### Original Error Message
```cpp
KMeanIndex::knnsearch Index is not of the same size than features
ucoslam_monocular: /home/tpp/UCOSlam-IBOIS/src/basictypes/reusablecontainer.h:245: T& ucoslam::ReusableContainer<T>::at(uint32_t) [with T = ucoslam::Frame; uint32_t = unsigned int]: Assertion 'index<_data.size()' failed.

Thread 1 "ucoslam_monocul" received signal SIGABRT, Aborted.
__GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
50      ../sysdeps/unix/sysv/linux/raise.c: No such file or directory.
(gdb) backtrace
#0  __GI_raise (sig=sig@entry=6) at ../sysdeps/unix/sysv/linux/raise.c:50
#1  0x00007ffff47e1859 in __GI_abort () at abort.c:79
#2  0x00007ffff47e1729 in __assert_fail_base
    (fmt=0x7ffff4977588 "%s%s%s:%u: %s%sAssertion `%s' failed.\n%n", assertion=0x5555555cec4b "index<_data.size()", file=0x5555555cec10 "/home/tpp/UCOSlam-IBOIS/src/basictypes/reusablecontainer.h", line=245, function=<optimized out>) at assert.c:92
#3  0x00007ffff47f2fd6 in __GI___assert_fail
    (assertion=0x5555555cec4b "index<_data.size()", file=0x5555555cec10 "/home/tpp/UCOSlam-IBOIS/src/basictypes/reusablecontainer.h", line=245, function=0x5555555ceba8 "T& ucoslam::ReusableContainer<T>::at(uint32_t) [with T = ucoslam::Frame; uint32_t = unsigned int]")
    at assert.c:101
#4  0x000055555558fa37 in ucoslam::ReusableContainer<ucoslam::Frame>::at(unsigned int) (this=<optimized out>, index=<optimized out>)
    at /usr/include/c++/9/bits/stl_vector.h:1040
#5  0x00007ffff7f1eb62 in ucoslam::ReusableContainer<ucoslam::Frame>::operator[](unsigned int)
    (index=<error reading variable: Unhandled dwarf expression opcode 0x0>, this=<optimized out>)
    at /home/tpp/UCOSlam-IBOIS/src/basictypes/reusablecontainer.h:134
#6  ucoslam::System::_11166622111371682966(ucoslam::Frame&, ucoslam::se3)
   Python Exception <class 'gdb.MemoryError'> Cannot access memory at address 0x8: 
 (this=<optimized out>, _16940374161810747371=..., _14387478432460351890=#7  0x00007ffff7f20b36 in ucoslam::System::process(ucoslam::Frame const&) (this=<optimized out>, inputFrame=...) at /home/tpp/UCOSlam-IBOIS/src/utils/system.cpp:442
#8  0x00007ffff7f21768 in ucoslam::System::process(cv::Mat&, ucoslam::ImageParams const&, unsigned int, cv::Mat const&, cv::Mat const&) (this=<optimized out>, _6441194614667703750=<error reading variable: Unhandled dwarf expression opcode 0x0>, _18212413899834346676=<error reading variable: Unhandled dwarf expression opcode 0x0>, _9933887380370137445=<error reading variable: Unhandled dwarf expression opcode 0x0>, _46082575014988268=..., _1705635550657133790=...) at /home/tpp/UCOSlam-IBOIS/src/utils/system.cpp:764
#9  0x00007ffff7e8f8f2 in ucoslam::UcoSlam::process(cv::Mat&, ucoslam::ImageParams const&, unsigned int)
    (this=<optimized out>, in_image=..., ip=..., frameseq_idx=<optimized out>) at /home/tpp/UCOSlam-IBOIS/src/ucoslam.cpp:23
#10 0x0000555555569f26 in main(int, char**) (argc=<optimized out>, argv=<optimized out>)
    at /home/tpp/UCOSlam-IBOIS/utils/monocular_slam.cpp:505
```

## Bug #5 [Not Traced]
- Caused by camera?
```cpp
OpenCV(4.5.5-dev) /home/tpp/opencv/modules/core/src/matrix_expressions.cpp:32: error: (-5:Bad argument) One or more matrix operands are empty. in function 'checkOperandsExist'

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
