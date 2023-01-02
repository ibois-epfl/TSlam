# TODO

- [ ] reconstruction: Write unit tests btoh for reconstruction (and mapping): for reconstruction it's a set of generated mapping from gh to compare vertex-vertex distance / for the mapping: is a sequence of frames to see if there is a response
- [ ] reconstruction: generate synthetic data for reconstruction evaluation and testing
- [ ] reconstruction: write test for accuracy based on vertex by vertex comparison of synthetic data

- [ ] reconstruction: rework CMakeLists files to generate and integrate light weigth library to be compiled and linked to the main TSlam
- [ ] reconstruction: implement a profiler to obtain some metrics of the performance of the geometric solver (on the set of synthetic data)

- [ ] @Petingo: integrate the reconstruction in both the TSLAM and the AC (be sure in the AC to expose the parameteers of the geometric solver to the interface so that they can be tuned live)
- [ ] @Petingo + @9and3 reconstruction: run the optitrack with the TSlam to obtain some trajectory testing. The protocol of the experiment needs to be designed (e.g., cutting pieces while doing it?)

## not urgent
- [ ] @Petingo: add a simple mesh visualizer for the vanilla TSlam (not for AC).
- [ ] @Andrea: start writing reconstruction paper section

## solved
- [x] Rewrite the map reconstruction post-processing code to C++
- [x] Change visualizer during mapping and inference
- [x] Add precompiled macros like #PROFILER to enable/disable parts of the code on release
- [x] The plan approximation need a more precise method of merging the plans