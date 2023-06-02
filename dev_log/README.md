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
- I tried to used the `-noKeyPoints` flag to map the timber, but the program would crash. The solution is to downgrade the UcoSLAM from 1.2.4 to 1.1.0, which is tagged "stable" in the releasing website. However, the result is unpromising (worse than the result with key points).
![](./mapping_nokeypoints.png)

- Tried with smaller object and slightly bigger tags (2cm vs 2.3cm), works pretty well!
![](./mapping_nokeypoints_good.png)

- Debug command
`gdb --args ./tslam_monocular '/home/tpp/UCOSlam-IBOIS/result/STag23mm_smallCube/use.mp4' '/home/tpp/UCOSlam-IBOIS/result/calibration_pixel3.yml' '-map' '/home/tpp/UCOSlam-IBOIS/result/STag23mm_smallCube/markers.map' "-markerSize" "0.023"`

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
            > This has been tackled down by adding a system-scale parameter "-localizeOnly", which disables the optimization process.

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

### Disabling optimization (loop detection)
- The optimization process is the main cause of the lagging problem.
- Actually there's a system-scale parameter "-enableLoopClosure" that controls this.

### Mapping with special conditions
#### Cut off beam
|     Mapping View     |    Mapped result     |
|:--------------------:|:--------------------:|
| ![](./short_cut.gif) | ![](./short_cut.png) |

#### Long beam with only marker in the end
|    Mapping View     |     Mapped result    |
|:-------------------:|:--------------------:|
| ![](./only_end.gif) | ![](./only_end.png)  |

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

### Reconstruction - Unit testing
We are building now the testing framework for the reconstruction part. The testing framework exploits doctest and the unit tests are composed of:

- unit testing of each geometric solver step with boolean check 
- value check for accuracy of comparison vertex-vertex with the ground truth synthetically generated model

Ideally, this has to be done with a series of very different geometries to test the robustness of the reconstruction method.

> ⚠️ **compatibility bug GTest-Open3d**: there is an error by linking Open3d and GTest that we tried at first to the same executable (it does not happen with other libraries i.e. Eigen):
> ```
> /usr/bin/ld: CMakeFiles/test_gsolver.dir/test_gsolver.cc.o: in function `GSolverTest_TestHelloWorld_Test::TestBody()':
> test_gsolver.cc:(.text+0x12d): undefined reference to `testing::internal::GetBoolAssertionFailureMessage(testing::AssertionResult const&, char const*, char const*, char const*)'
> collect2: error: ld returned 1 exit status
> gmake[2]: *** [tests/CMakeFiles/test_gsolver.dir/build.make:152: tests/test_gsolver] Error 1
> gmake[1]: *** [CMakeFiles/Makefile2:917: tests/CMakeFiles/test_gsolver.dir/all] Error 2
> gmake: *** [Makefile:91: all] Error 2
> ```
> To overcome this we decided to go for the header only library doctest.


### Reconstruction - refinement for the volume reconstruction
The main base code is over but some things are left to do (see `TODO.md` updated list). The major achievement is the precision at which faces of the timber are detected now. Of course this will need some adaptations once we will test with the generated synthetic data.

*The new overhaul reconstruction and modified gsolver:*
![](./newgsolver.png)

*The precise face matching of the reconstructed mesh:"
![](./precisetag.png)

### Reconstruction - tassellation (change of stratey to obtain the  polygon pools)
We change strategy to obtain the pool of all possible polygons. First, we intersect the polygons obtained from the intersection with the AABB among them. The splitting segments generated by the extremes given by the intersection on each plane are used to detect all the possible polygons describing the volume. The tassellation of the intersecting segments is obtained with `CGAL/Segmentation_2d.hh` toolkit.

> In order to use the tassellation we had to apply a plane-toXYplane transform to apply the CGAL toolkit only on 2d

![](./tassellation_process.png)

The results are quite solid but we are now facing the problem of the *selection of candidate faces* for notched timber in the next step. See the image below:

![](./problem_sel_face_candidate.png)

Here's the list of next TODOs:
- [x] check if planes are actually well assigned to each new tassellated polygons
- [x] in real world data tolerance needs to be adaptative and permissive for the process of considering tag's centers as part of the plane
- [x] internal polygons need to be effectivly culled out to leave only the external polygons.
- [x] faces might not have tags. To obviate this, we should find a mechanism to maximize the existing tags (e.g., extending them with a radius or circle)

### Reconstruction - eligible face selection + top interface
In order to select the face composing the actual volume we restructured the previous selectionag's algorithm into the 3 following filtering checks:
- **(i) an adaptative tolerance**: in order to find the thershold distance to consider a point belonging to a polygon's plane, we consider the median with tolerance (see code). It adapts from polygon to polygon;
- **(ii) additional selection criteria by normal comparison**: the filtering of closest tag's centers might not be enough in case of internal polygons or corner scenarios. In those cases we check the tag's center normal with the normal of the plane. If too different we pass.
- **(iii) is the tag's center inside the polygon**: the last check consists in projecting the filtered tag's center so far to the plane, if they are inside the polygon we select our face as a candidate.

![](./select_faces_a.png)
![](./select_faces_b.png)

> ⚠️ The currentmost important to the system is if *there is an eligible face, which has no sticker attached to the timber face*. In this case it will discared. The algorithm should be reinforced by covering also this case.

A basic top api interface is implemented, see the file `tslam_reconstructor.hh`.

We also tried to give it a shot for very complex geometries such as this one but without success.

|        real object       |     reconstruct geo      |
|:------------------------:|:------------------------:|
| ![](./complex_shape.png) | ![](./real_complexe.jpg) |

### Reconstruction - implementation of testing
We need testing for the next CI but also the refinement of the current reconstruction algorithm. We are using `DocTest` for testing and we test both synthetic and reals scans maps (`.yml`) with expected ground-truth data.

![](./doctest.png)


### Reconstruction - implement normal clustering 
We changed the way we cluster the tags and now we implement a normal-based clustering from [Cilantro](https://github.com/kzampog/cilantro/blob/master/examples/connected_component_extraction.cpp). By adding a refinement by planes of the group of tags the tag grouping result quite robust.

The current challenge is the **face selection** after the tassellation. It might happen that tags are not present to validate the face, hence the polygon will not be selected and we will have a hole.

![](./workingsplitting.gif)

*A different approach needs to be found for selecting eligible faces*

### Feb 15: stand up on the development
Today we got a general stand up meeting with Yves and Julien. The development is successfull but we need to focus now on the evaluation of the TSLAM. This means that we need to cut out some features from the road map, fix parameters, do a due diligence on the evaluation of SLAMS and esstablish a scientific protocol for the evaluation.

|                     |                      |
|:-------------------:|:--------------------:|
| ![](./standup1.jpg) | ![](./standup2.jpg)  |


### Reconstruction - new polygon faces selector
We implemented a new method to check wether a polygon is a face or not by checking if a tag is contained in it. It is robust and the algorithm has been parametrized to detect multiple faces. Here we can see that we are able to now to describe complex shapes.

|        notched piece       |     complex piece      |
|:------------------------:|:------------------------:|
| ![](./patchalg1.png) | ![](./patchalg2.png) |

What we are missing now is a resilient refiner to fill the holes with e.g. [one method in CGAL](https://doc.cgal.org/latest/Polygon_mesh_processing/Polygon_mesh_processing_2hole_filling_example_SM_8cpp-example.html). In general this will be a limitation of the system to have markers to cover the entire piece. The rule to avoid holes is not clear at the moment.

update `22-02-2023`: we tried the `triangulate_refine_and_fair_hole()` func in CGAL 5.5.1 but it is not giving the expected results. An issue is open on [CGAL github](https://github.com/CGAL/cgal/issues/7283).


## March 1: hole filling
The hole filling from CGAL is implemented and it patch some of the missing holes. Nonetheless, some non-manifold faces are still produced. A refinement of the hole patcher needs to be implemented.

|        notched piece       |     complex piece      |
|:------------------------:|:------------------------:|
| ![](./hole_patch.png) | ![](./hole_patch2.png) |

## March 13: Mesh View
The bug about clipping and perspective projection has been fixed. Mesh view is now working properly.
![](./mesh_view.png)

## March 23: Set up prep
We started to preparing the evaluation pipeline and the set up for the evaluation campaign.
![](./evalcampaign1.jpg)
![](./evalcampaign2.jpg)

## May 1
Test on "no key point detection" mode. From the screenshot we can see that in tag-only mode, the drift decreased.

|          with keypoints          |          without keypoints          |
|:--------------------------------:|:-----------------------------------:|
| ![](./keypoint_test_with_kp.png) | ![](./keypoint_test_without_kp.png) |

---

Current TODOs:
- [x] (urgent) a piece with double stripes on edges each is not reconstructing correctly, check the bug out;
- [x] (urgent) establish an evaluation protocol for the tslam
- [x] (urgent) find a better algorithm for selecting candidate polygon faces
- [x] (urgent) solve reconstruction for more complex objects
- [ ] (urgent) delete all dependecies of old UCOSlam and code labels replaced with TSLam
- [x] (urgent) integrate the reconstruction as a library in the main TSlam base code

- [x] (medium) TSLAM: adjust visualizer for overlapping mesh
- [x] (mdeium) TSLAM: establish scientific protocol for evaluation with optitrack
- [x] (medium) in reconstruction: the check mesh sanity is not wrokgin properly
- [x] (medium) in reconstruction: we need to implement a hole filler

- [x] (medium) set up CI/CD
- [x] (medium) for reconstruction sub-program: get rid of open3d and integrate a mesh I/O pipeline 
- [ ] (medium) package and integrate the new version of tslam in AC

- [ ] (optional) implement the a `.xac` (xml based) format in `ts_geometric_solver.hh` and `tslam_reconstructor.hh`;

- [x] for reconstruction: write test units
