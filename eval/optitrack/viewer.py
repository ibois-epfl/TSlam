#! python3
# -*- coding: utf-8 -*-

import os
import sys
import argparse
import winsound

import threading

import cv2

class Viewer(threading.Thread):
    """ Show penultimate image in directory """
    def __init__(self, dir: str):
        super(Viewer, self).__init__()
        self.dir = dir
        self._penultimate_image = None
        self.is_running = True
        self._healt_check = 0

    def run(self):
        img_path_prev = ""
        health_check = 0
        while self.is_running:
            img_path = self.penultimate_image

            if img_path != None:
                img = cv2.imread(img_path)
                cv2.putText(img, "Press q to quit", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                cv2.imshow("Penultimate image", img)
                cv2.resizeWindow("Penultimate image", 1280, 720)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.is_running = False
            
            img_path_prev = img_path
        cv2.destroyAllWindows()

    def close(self):
        self.is_running = False
    
    @property
    def penultimate_image(self):
        """ Return the path to the second-to-last image in the directory without loading into memory the entire directory"""
        with os.scandir(self.dir) as entries:
            recent_image = None
            penultimate_image = None
            for entry in entries:
                if entry.is_file() and entry.name.endswith((".jpg", ".png")):
                    file_stat = entry.stat()
                    file_mod_time = file_stat.st_mtime
                    if recent_image is None or file_mod_time > recent_image[0]:
                        penultimate_image = recent_image
                        recent_image = (file_mod_time, entry.name)
                    elif penultimate_image is None or file_mod_time > penultimate_image[0]:
                        penultimate_image = (file_mod_time, entry.name)
            if penultimate_image is not None:
                if self._penultimate_image == os.path.join(self.dir, penultimate_image[1]):
                    self._healt_check += 1
                    if self._healt_check > 50:
                        winsound.Beep(440, 5000)
                else:
                    self._healt_check = 0
                    self._penultimate_image = os.path.join(self.dir, penultimate_image[1])
                    return self._penultimate_image
            else:
                return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Show penultimate image")
    parser.add_argument("--dir", type=str, help="Directory of images")
    args = parser.parse_args()

    viewer = Viewer(args.dir)
    viewer.start()
    viewer.join()