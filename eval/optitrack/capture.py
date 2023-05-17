#! python3
# -*- coding: utf-8 -*-

import sys
import argparse
import time
from Nat_Net_Python import NatNetClient
import os 
from datetime import datetime
import cv2
from waiting import wait

from viewer import Viewer
import wrighter
from wrighter import Wrighter

import threading

import refinement

__root__ = os.path.abspath( os.path.dirname( __file__ ) )
now = datetime.now()
folder = now.strftime("%d_%m_%Y_%H_%M_%S")
IMG_OUTPUT_PATH = __root__ + f"/data/Recordings/{folder}/img"
CSV_OUTPUT_PATH = __root__ + f"/data/Recordings/{folder}/raw_stream.csv"
OUTPUT_DIR = __root__ + f"/data/Recordings/{folder}"
FFMPEG_LOG_PATH = __root__ + f"/data/Recordings/{folder}/ffmpeg_capture.log"

os.makedirs(IMG_OUTPUT_PATH,exist_ok=True)

def main(_verbose : int,
         _camera_idx : int,
         _camera_name : str,
         _fps : int,
         _res : str,
         )-> None:
    # set up for streaming client
    streaming_client = NatNetClient.NatNetClient()
    streaming_client.set_print_level(_verbose)
    
    # run streaming client
    is_running = streaming_client.run(csv_file=CSV_OUTPUT_PATH,
                                      img_folder=IMG_OUTPUT_PATH)
    
    # check for net connection
    if not is_running:
        print("\033[91m [ERROR]: Could not start streaming client.\033[0m")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    # wait(lambda: os.path.exists(CSV_OUTPUT_PATH), timeout_seconds=10, sleep_seconds=1, waiting_for="streaming to start")
    time.sleep(1)
    if not os.path.exists(CSV_OUTPUT_PATH):
        print("\033[91m [ERROR]: Could not connect properly. Check that Motive streaming is set to ON.\033[0m")
        streaming_client.shutdown()
        sys.exit(2)
    is_looping = True
    time.sleep(1)

    # set up the viewer in a separate thread process to avoid blocking the streaming client
    print("[INFO]: Starting viewer on a seperate thread...")
    viewer = Viewer(IMG_OUTPUT_PATH)
    viewer.start()

    # start video streamer
    print("[INFO]: Starting video streamer...")
    video_writer = Wrighter(video_path=_camera_name,
                              frames_out_path=IMG_OUTPUT_PATH,
                              fps=_fps,
                              res=_res)
    video_writer.start()

    # Type "e" + Enter to stop streaming
    while is_looping:
        if (sys.stdin.read(1) == 'e'):
            is_looping = False

            print("[INFO] Closing the viewer...")
            viewer.close()
            viewer.join()

            print("[INFO] Closing the video writer...")
            video_writer.close()
            viewer.join()
            wait(lambda: video_writer.is_alive() is False)
            time.sleep(1)

            print("[INFO] Exiting optitrack streaming and image recording...")
            streaming_client.shutdown()
            time.sleep(1)

            print("INFO] Creating video from frames...")
            Wrighter.create_video(IMG_OUTPUT_PATH, OUTPUT_DIR)

            print("[INFO] Post processing video frames and optitrack poses...")
            refinement.refine_poses(CSV_OUTPUT_PATH, IMG_OUTPUT_PATH, FFMPEG_LOG_PATH)

            try:
                sys.exit(0)
            except SystemExit:
                print("...")
            finally:
                print("[INFO] Closing the recording script...")


if __name__ == "__main__":
    # implement argparse
    parser = argparse.ArgumentParser(description="The script connects to the NatNetPython streaming client and streams the data to a csv file and images to a folder. You need to have both a Motive and camera connected to your computer.",
                                     epilog="Short example: python capture.py -c=0")
    parser.add_argument("-v", "--verbose",
                        help="set output verbosity, from 0 to 2",
                        # action="store_true",
                        default=1)
    parser.add_argument("-c", "--camera",
                        help="set camera index",
                        action="store",
                        type=int,
                        default=2)
    parser.add_argument("-cn", "--camera-name",
                        help="set camera name for the video writer",
                        action="store",
                        type=str,
                        default="iCatch V37")
    parser.add_argument("-f", "--fps",
                        help="set camera fps",
                        action="store",
                        type=int,
                        default=30)
    parser.add_argument("-r", "--res",
                        help="set camera resolution",
                        action="store",
                        type=str,
                        default="1280x720")
    args = parser.parse_args()
    
    print("\033[92m [INFO]: Before use the script set _ Broadcast Frame _ ON on Motive. \033[0m")
    print("\033[93m [WARNING]: Be sure that the _ Up Axis _ value of the data streaming pane (View > DataStreamingPane) in Motive 2.3.1 is set to _ ZUp _. \033[0m")
    print("\033[93m [WARNING]: It is not possible to pause/restart the streaming. Once stoped, the recordings shuts down. \033[0m")

    print("\033[92m [INFO]: Checking for camera connection ... \033[0m")
    cv2.VideoCapture(args.camera).release()
    if cv2.VideoCapture(args.camera).isOpened() is False:
        print("\033[91m [ERROR]: Camera is not connected. \033[0m")
        cv2.VideoCapture(args.camera).release()
        sys.exit(1)
    print("\033[92m [INFO]: Camera is connected. \033[0m")

    main(_verbose = int(args.verbose),
         _camera_idx = args.camera,
         _camera_name = args.camera_name,
         _fps = args.fps,
         _res = args.res)