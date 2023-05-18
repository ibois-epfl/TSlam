#! python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import subprocess
from subprocess import Popen, PIPE, STDOUT, CREATE_NEW_CONSOLE
import threading
from pathlib import Path
import time


class Wrighter(threading.Thread):
    """ Write frames by accessing the camera with ffmpeg at a given fix framerate. """
    def __init__(self,
                 video_path : str,
                 frames_out_path : str,
                 fps : int,
                 res : tuple):
        super(Wrighter, self).__init__()
        self.video_path = '"' + video_path + '"'
        self.frames_out_path = os.path.join(Path(frames_out_path), "%d.png")
        self.fps = fps
        self.res = res

        self.is_running = False

        self._first_frame_tstamp : str = None
        self._last_frame_tstamp : str = None
        self._last_frame_nbr : int = None

        self.proc = None

    @staticmethod
    def create_video(img_folder,
                    session_dir,
                    fps=30,
                    size_x=1280,
                    size_y=720):
        """ Save frames to video file with local ffmpeg at 30 fps just for visualization"""
        os.system("ffmpeg -r {} -i {} -vcodec libx264 -crf 25 -s:v {}x{} -pix_fmt yuv420p {}".format(fps,
                                                                                        os.path.join(img_folder, "%d.png"),
                                                                                        size_x, size_y,
                                                                                        os.path.join(session_dir, "video.mp4")))
        print("\033[92m [INFO]: Successfull, video saved in: {} \033[0m".format(session_dir))

    def _clean_untracked_frames(self) -> None:
        """ Clean the frames that have been saved but not tracked in the log file for timestamps. """
        if self._last_frame_nbr is None:
            return

        for file in os.listdir(os.path.dirname(self.frames_out_path)):
            if int(file.split(".")[0]) > self._last_frame_nbr:
                os.remove(os.path.join(os.path.dirname(self.frames_out_path), file))

    def run(self):
        # check for paths validity
        if not os.path.exists(os.path.dirname(self.frames_out_path)):
            print(f"\033[91m [ERROR]: Out folder does not exist \033[0m")
        
        #set ffmpeg cmd
        ffmpeg_log_path = os.path.join(os.path.dirname(
                                        os.path.dirname(self.frames_out_path)), 
                                         "ffmpeg_capture.log")
        cmd : str = [f"ffmpeg " ,                           # -> ffmpeg command
                    f"-f dshow " ,                          # -> DirectX on windows
                    f"-s {self.res} " ,                     # -> resolution
                    "-r 30 " ,                              # -> framerate
                    "-vcodec mjpeg " ,                      # -> video codec
                    "-rtbufsize 2147.48M " ,                # -> buffer size (fix top avoid runtime error for memory))
                                                            # but it is consuming more memory
                    f"-i video={self.video_path} " ,        # -> input device
                    "-vsync 1  " ,                          # -> is a passthrough where each frame is passed with its
                                                            # original timestamp. 1 means that frames are duplicated
                                                            # to achieve exactly the requested constant frame rate
                    f"{self.frames_out_path} -y"]           # -> output path for saving the frames
        cmd = " ".join(cmd)
        self.proc = subprocess.Popen(cmd,
                                     shell=True,
                                     stdin=PIPE,
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT)
        self.is_running = True
        
        # read process output and save
        with open(ffmpeg_log_path, "w") as f:
            try:
                for p in self.proc.stdout:
                    if "frame=" in p.decode():
                        if self._first_frame_tstamp is None:
                            self._first_frame_tstamp = p.decode().split("frame=")[1].split("time")[1][1:12]
                            print(f"\033[92m [INFO]:  First frame captured at {self._first_frame_tstamp} \033[0m")
                            start_time = time.time()
                            f.write('\n' + f">>>>>>>>>>>>>>>>>>>>> World START time: {start_time} \n" + '\n')
                            print(f"\033[92m [INFO]:  World START time: {start_time} \033[0m")
                        if (not self.is_running and self._last_frame_tstamp is None):
                            self.proc.communicate(input="q".encode())
                            self._last_frame_tstamp = p.decode().split("frame=")[1].split("time")[1][1:12]
                            self._last_frame_nbr : int = int(p.decode().split("frame=")[1].split("fps=")[0])

                            print(f"\033[92m [INFO]:  Last frame captured at {self._last_frame_tstamp} \033[0m")
                            print(f"\033[92m [INFO]:  Last frame number: {self._last_frame_nbr} \033[0m")


                            # get the end time by summing the world start time to ffmpeg last timestamp
                            h = int(self._last_frame_tstamp[:2])
                            m = int(self._last_frame_tstamp[3:5])
                            s = int(self._last_frame_tstamp[6:8])
                            ds = int(self._last_frame_tstamp[9:11])
                            tstamp_sd = (h * 3600 + m * 60 + s) + ds / 100.0
                            end_time = start_time + tstamp_sd
                            f.write('\n' + f">>>>>>>>>>>>>>>>>>>>> World END time: {end_time} \n" + '\n')
                            print(f"\033[92m [INFO]:  World END time: {end_time} \033[0m")
                            f.write(p.decode())

                            self._clean_untracked_frames()

                            break
                    f.write(p.decode())

            except KeyboardInterrupt:
                self.close()
                self.join()

    def close(self):
        self.is_running = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract frames from video at a given framerate")
    parser.add_argument("--video", type=str, help="Path to video")
    parser.add_argument("--frames", type=str, help="Output path to frames")
    parser.add_argument("--fps", type=int, default=30, help="Framerate of the video")
    parser.add_argument("--res", type=str, default="1280x720",
                        help="Resolution of the video in format WxH")
    args = parser.parse_args()

    wrigther = Wrighter(args.video, args.frames, args.fps, args.res)
    wrigther.start()

    input("[INFO] Press Enter to close the wrigther... \n")
    if not wrigther.is_running:
        wrigther.join()
        sys.exit(1)
    wrigther.close()
    wrigther.join()
    print("[INFO] Wrighter closed")