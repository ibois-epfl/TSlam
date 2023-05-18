#! python3
# -*- coding: utf-8 -*-

import cv2
import os
import subprocess
import sys
import argparse
import time
import glob
import shutil

__path__ = os.path.abspath(os.path.dirname(__file__))

def _str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def main(_is_saved : bool,
         _res : tuple,
         _fps : int,
         _camera_idx : int,
         _session_dir : str,
         _frames_dir : str,
        ) -> None:
    """
        Record a video from a camera with given fps app.
            1. set the camera settings to cap the fps
            2. start a timer
            3. open the csv file stream
            in the loop:
              4. get the frame and save int
              5. get the time and save it in the csv
              6-7. check for stop/start key input or exit input
            8. print the total time in minutes and save it in a txt file
            9. assemble all the frames into a video of format .avi
        
        :param _is_saved: if True, output data, else just show the camera buffer
        :param _res: the resolution to record the video
        :param _fps: the fps to record the video at
        :param _camera_idx: the camera index to record from
        :param _session_dir: the directory to save the video and the csv file
        :param _frames_dir: the directory to save the frames
    """

    # 1. set the camera settings to cap the fps
    print("[INFO] Setting camera parameters up...")
    is_res_set : bool = False
    cap = cv2.VideoCapture(_camera_idx)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, _res[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, _res[1])
    
    fps = cap.get(cv2.CAP_PROP_FPS)
    camera_settings : str = "camera settings:: fps: {}, cap_width: {}, cap_height: {}".format(
                                                                                      _fps,
                                                                                      cap.get(3),
                                                                                      cap.get(4))
    with open(os.path.join(_session_dir, "camera_settings.txt"), "w") as f:
        f.write(camera_settings)
        print(camera_settings)

    # 2. start a timer
    start_time = time.time()
    time_stamp = 0  # in minutes

    # 3. open the csv file stream
    print("[INFO] Opening csv file stream...")
    csv_file = open(os.path.join(_session_dir, "time_stamps.csv"), "w")

    # in the loop:
    print("[INFO] Recording...")
    frame_nbr = 0
    while cap.isOpened():
        # 4. get the frame and save int
        ret, frame = cap.read()
        if not is_res_set:
            frame = cv2.resize(frame, _res)
        key = cv2.waitKey(1)
        if ret is False:
            print("\033[91m [ERROR]: Could not read frame. \033[0m")
            sys.exit(1)
        frame_nbr += 1
        time_stamp = (time.time() - start_time) / 60

        cv2.putText(frame, "Time (minutes): {}".format(time_stamp), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)  # time
        cv2.putText(frame, "Nbr frames: {}".format(frame_nbr), (10, 53), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)  # frame number
        if _is_saved:
            cv2.imwrite((os.path.join(_frames_dir, str(frame_nbr)+".jpg")), frame)

        # 5. get the time and save it in the csv
        time_stamp = (time.time() - start_time) / 60
        if _is_saved:
            csv_file.write("{},{}\n".format(frame_nbr, time_stamp))

        # * show the camera image
        cv2.imshow("frame", frame)

        # * cap fps
        time.sleep(1/_fps)

        # 6-7. check for stop/start key input as spacebar
        if (key & 0xFF == ord(' ')):
            print("\033[93m [WARN]: Recording paused. Press 'spacebar' to resume. \033[0m")
            while key & 0xFF != ord(' '):
                pass
            print("\033[93m [WARN]: Recording resumed. \033[0m")
        if (key & 0xFF == ord('q')):
            break
        if (key & 0xFF == ord('Q')):
            break

    if _is_saved:
        # 8. print the total time in minutes and save it in a txt file
        print ("[INFO] Writing total time in minutes to text file...")
        total_time = (time.time() - start_time) / 60
        print("\033[92m [INFO]: Total time: {} minutes. \033[0m".format(total_time))
        with open(os.path.join(_session_dir, "total_time.txt"), "w") as f:
            f.write("Total time: {} minutes.".format(total_time))
        print("\033[92m [INFO]: Successfull, data saved in: {} \033[0m".format(_session_dir))

        # 9. assemble all the frames into a video of format .avi
        print("[INFO] Assembling frames into video...")
        if not is_res_set:
            os.system("ffmpeg -r {} -i {} -vcodec libx264 -crf 25 -s:v {}x{} -pix_fmt yuv420p {}".format(_fps,
                                                                                        os.path.join(_frames_dir, "%d.jpg"),
                                                                                        _res[0], _res[1],
                                                                                        os.path.join(_session_dir, "video.avi")))
        else:
            os.system("ffmpeg -r {} -i {} -vcodec libx264 -crf 25 -s:v {}x{} -pix_fmt yuv420p {}".format(30,
                                                                                        os.path.join(_frames_dir, "%d.jpg"),
                                                                                        int(cap.get(3)), int(cap.get(4)),
                                                                                        os.path.join(_session_dir, "video.avi")))
        print("\033[92m [INFO]: Successfull, video saved in: {} \033[0m".format(_session_dir))

    print("[INFO] Cleaning up...")
    cap.release()
    cv2.destroyAllWindows()
    csv_file.close()
    # erase the frames directory
    if _is_saved:
        shutil.rmtree(_frames_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record and time a video from camera at a given fps value.",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     epilog="Example: python record.py -n beam_12 -f 15 -c 0 -o custom_dir",
                                     add_help=True)
    parser.add_argument("-n", "--name", type=str, default="no_name", help="Set the name of the session.")
    parser.add_argument("-res", "--resolution", type=str, default="1280x720", help="Set the resolution of the camera in format e.g. 1280x720 set via accessing directX.")
    parser.add_argument("-ns", "--save", type=_str2bool, nargs='?', const=True, default=True, help="If you just want to show the video set to false.")
    parser.add_argument("-f", "--fps", type=int, default=4, help="Set the fps value.")
    parser.add_argument("-c", "--camera", type=int, default=0, help="Set the camera index.")
    parser.add_argument("-o", "--outputdir", type=str, default="", help="""Set the output dir name. In the output dir you will find:\n +
                                                                           frames named by index\n +
                                                                           .csv with time stamps in seconds per frame\n +
                                                                           .txt file with the total time of the recording\n +
                                                                           .avi recording of the sequence.""")
    args = parser.parse_args()
    print("[INFO]: Record and time a video from camera at a given fps value. Press 'spacebar' to stop/resume or 'q' to exit.")

    session_dir = ""
    frames_dir = ""
    if args.save:
        print("\033[93m [WARN]: To output the video the script needs the machine to have 'ffmpeg' installed. You can install it with 'winget install ffmpeg'. \033[0m")
        print("\033[93m [INFO]: Creating directories ... \033[0m")
        if args.outputdir == "":
            args.outputdir = os.path.join(__path__, "data")
        else:
            if not os.path.exists(args.outputdir):
                os.makedirs(args.outputdir)
        session_dir = os.path.join(args.outputdir, args.name + '-' + time.strftime("%Y%m%d-%H%M%S"))
        if not os.path.exists(session_dir):
            os.makedirs(session_dir)
        else:
            print("\033[91m [ERROR]: Session folder already exists. \033[0m")
            sys.exit(1)
        frames_dir = os.path.join(session_dir, "frames")
        os.makedirs(frames_dir)
        print("\033[92m [INFO]: Successfull, data will be saved in: {} \033[0m".format(session_dir))
    else:
        print("\033[93m [WARN]: Data will not be saved. \033[0m")

    # parse resolution
    res = args.resolution.split("x")
    if len(res) != 2:
        print("\033[91m [ERROR]: Resolution format is not correct. \033[0m")
        sys.exit(1)
    res = (int(res[0]), int(res[1]))

    print("\033[93m [INFO]: Checking if camera is connected ... \033[0m")
    if not cv2.VideoCapture(args.camera).isOpened():
        print("\033[91m [ERROR]: Camera is not available. \033[0m")
        sys.exit(1)
    print("\033[92m [INFO]: Successfull, camera is connected. \033[0m")

    main(_is_saved=args.save,
         _res=res,
         _fps=args.fps,
         _camera_idx=args.camera,
         _session_dir=session_dir,
         _frames_dir=frames_dir
         )