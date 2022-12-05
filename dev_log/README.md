# Dev Log
- [TODO](./TODO.md)
- [Bug Tracing](./Bug_tracing.md)
---

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
![](./tslam_with_stag.png)
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
`gdb --args ./tslam_monocular '/home/tpp/UCOSlam-IBOIS/result/STag23mm_smallCube/use.mp4' '/home/tpp/UCOSlam-IBOIS/result/calibration_pixel3.yml' '-map' '/home/tpp/UCOSlam-IBOIS/result/STag23mm_smallCube/markers.map' "-aruco-markerSize" "0.023"`

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

## Merge 2 maps
- Takes two tag maps (the exported .yml file) as input
- Estimate the affine transformation matrix using OpenCV

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

`Jul 25`
## Merge map in CPP
- Usage: `build/utils/tslam_conbine_map 1.map 2.map combined.map`
- In `map.h/map.cpp`, a new instance `Map::projectTo(Map anotherMap)` is added, which estimates the transformation matrix based on the duplicated tags and perform the transformation.
- Result
  ![](./merge_cpp.png)

### The long beam
#### Webcam
- The beam is curved, both the original map and the combined result.
- Scanned Map

| Map 1 | Map 2 |
| :---: | :---: |
| ![](./long_beam_1_webcam_1.png) | ![](./long_beam_1_webcam_2.png) |

- Combined Map

| Front | Up | Side |
| :-------------: | :------------: | :------------: |
| ![](./long_beam_1_webcam_f.png) | ![](./long_beam_1_webcam_u.png) | ![](./long_beam_1_webcam_s.png) |


#### Pixel 3
- The result is better
- Scanned Map

| Map 1 | Map 2 |
| :---: | :---: |
| ![](./long_beam_1_pixel_1.png) | ![](./long_beam_1_pixel_2.png) |

- Combined Map

| Front | Up | Side |
| :-------------: | :------------: | :------------: |
| ![](./long_beam_1_pixel_f.png) | ![](./long_beam_1_pixel_u.png) | ![](./long_beam_1_pixel_s.png) |


### Potential Solution
- Rolling-shutter Calibration: 
  https://github.com/ethz-asl/kalibr/wiki/Rolling-Shutter-Camera-calibration

## Most bugs are resolved
- We hit 100k! (But last time it crashed in 106XXX with a segmentation fault in the 3rd party library fbow :P)

![](./100k.gif)

`Aug 01`
- The map utilities (merge & optimization) is now a member function of Map.


### Reconstruction
We will use only the `.yaml` file since the point cloud obtained from the mapping phase has no definition. Mostly the points are concentrated where the fiducial markers are.

![](./snappoints.png)

Added a small visualizer for debugging and dev.

![](./snap_wireframe.png)

Here's the workflow for the reconstruction:
*[To be added]*

* parse the yml mapping file
![](./snap_wireframe.png)

* we get the plane for each tag
![](./planesinter.png)

* To get the intersections of the planes we switched to an AABB instead of OBB. It's simpler and faster for 
calculating the polygon intersections for all the tags. Here's the preliminary result:
![](./abbintersect.png)
