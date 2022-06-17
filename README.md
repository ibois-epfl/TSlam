# UcoSLAM-IBOIS
UcoSLAM modified for augmented carpentry project

- The official document of UcoSLAM:  [link](https://docs.google.com/document/d/12EGJ3cI-m8XMXgI5bYW1dLi5lBO-vxxr6Cf769wQzJc)

## Build
```bash
# install library
sudo apt-get install cmake libopencv-dev qtbase5-dev libqt5opengl5-dev libopenni2-dev

# build
mkdir build
cd build
cmake ../ -DBUILD_GUI=ON
make -j4
[sudo make install] //no needed, will install it on your system
```

## Usage
- Run GUI:
```
./build/gui/UcoSLAM_GUI
```

- Exporting tag map to yml:
```
./build/utils/reslam_map_export <map file exported from GUI> out.(pcd|ply|yml)
```

## Post processing
- `cluster.py`: Taking the tag map as input and perform 3d plane fitting.