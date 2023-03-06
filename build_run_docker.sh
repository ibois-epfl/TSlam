#!/bin/bash
# delete all cmake cache files 
find . -name "CMakeCache.txt" -type f -delete

sudo docker build -t tslam .

sudo docker run -it tslam /bin/bash