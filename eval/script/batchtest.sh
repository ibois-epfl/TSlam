# for pose and video output
python /home/as/TSlam/eval/script/compute_poses.py --frames /home/as/TSlam/eval/script/test/01_sequences_c.txt --monoexe /home/as/TSlam/build/utils/tslam_monocular --calib /home/as/TSlam/assets/calibrationf/calibration_orange_A_1280_720_r.yml --map /home/as/TSlam/eval/script/test/01_opti_map.map --video /home/as/TSlam/eval/script/test/video_01.mp4 --output /home/as/TSlam/eval/script/test/outposes --name 01_test_v5_gui --exportVideo --onlyTags


# for metrics sequence
python eval/script/compute_slam_metrics.py --name 01overview --gt /home/as/TSlam/eval/script/test/refined_stream_01.csv --ts /home/as/TSlam/eval/script/test/outposes/01_test_v5_gui_2023-06-02_17-24-45/ --out /home/as/TSlam/eval/script/test/outmetrics --debug --saveImg --makeAnimation ""
