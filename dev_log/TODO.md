# TODO

- [ ] (to be defined) integration to AC
- [x] add flag to console app tslam_monocular to disable GUI and run only slam
- [ ] solve the go away tracking bug
- [ ] UNIT TESTING:
- [ ] write a longer smoke test for inference of tslam
- [ ] write a test for mapping phase of tslam
- [ ] @Andrea: write exporter of single tags from the tslam_reconstruct for manually reconstruction
- [x] change slamuco to tslam when possible










==============================================================

- [x] Update dev_log (for optimization, +video)
- [x] Console program -> +trajectory (list of matrix)
- [x] Visualizing mesh with lines during instancing
- [x] Integrating reconstruction into TSlam
- [ ] Comparing with optic track
  - [ ] Exporting the trajectory as the same format as optic track
  - [ ] Transformation from the object to the camera
  - [ ] Look for a way to compare the two trajectories


- [x] reconstruction: Write unit tests btoh for reconstruction (and mapping): for reconstruction it's a set of generated mapping from gh to compare vertex-vertex distance / for the mapping: is a sequence of frames to see if there is a response
- [x] reconstruction: generate synthetic data for reconstruction evaluation and testing
- [x] reconstruction: write test for accuracy based on vertex by vertex comparison of synthetic data

- [x] reconstruction: rework CMakeLists files to generate and integrate light weigth library to be compiled and linked to the main TSlam
- [x] reconstruction: implement a profiler to obtain some metrics of the performance of the geometric solver (on the set of synthetic data)

- [ ] @Petingo: integrate the reconstruction in both the TSLAM and the AC (be sure in the AC to expose the parameteers of the geometric solver to the interface so that they can be tuned live)
- [x] @Petingo + @9and3 reconstruction: run the optitrack with the TSlam to obtain some trajectory testing. The protocol of the experiment needs to be designed (e.g., cutting pieces while doing it?)

## not urgent
- [x] @Petingo: add a simple mesh visualizer for the vanilla TSlam (not for AC).
- [ ] @Andrea: start writing reconstruction paper section
- [ ] in file `tslam_export.h` there are old naming to change

## solved
- [x] Rewrite the map reconstruction post-processing code to C++
- [x] Change visualizer during mapping and inference
- [x] Add precompiled macros like #PROFILER to enable/disable parts of the code on release
- [x] The plan approximation need a more precise method of merging the plans
