# Evaluation of TSlam
This folder contains all the documents describing the evaluation designed for TSlam.
The TSlam *is an hybrid monocular camera's pose localization algorithm based on both direct feature detection and fiducial markers*.

It is composed by 3 distinct but dependent phases:
- **(1) Mapping**: the piece is first mapped with the camera to obtain a map file of the location of each tags
- **(2) Reconstruction**: from the obtained map file, a 3d model of the piece is reconstructed
- **(3) Tracking**: finally the tracking is fed the map file and 3d model mesh. Hence the algorithm can provide the camera's pose when a tag is tracked and visualize the mesh as an overlay on the physical object.


## Experimental Protocol
These are the criteria for the evaluation:

- **(A) Reconstruction accuracy**: how close is the reconstructed mesh to its GT.
  - *Evaluation set-up*: the evaluation and GT are synthetically generated via Blender. The repository for the generation of the data is [here](https://github.com/ibois-epfl/TSlam-gt-data).
  - *Test population*: 20 timber pieces
  - *Fix params*:
    - `no round shape`: the shapes of the reconstructed objects cannot be round.
    - `no noise`: the tags present no noise (e.g., displaced/inclined planes in the same row).
  - *Varying params*:
    - `timber shape`: we limit the testing to the following shapes and formal characteristics of the objects: (a) box-like objects, (b) skewed object-like objects, (c) length limits of 6 meters MAX and 10 cm min, (d) the number of notch per piece are limited to 2, (e) the type of joinery is limted to the 3 most common joineries (lap-joint, chair-joint, dove-tail)

- **(B) Tracking accuracy**: how close the output TSlam's pose is to its GT. 
  - *Evaluation set-up*: the GT is monitored via the Optitrack system. The code for running the Optitrack capturing is [here](https://github.com/ibois-epfl/aiac-optitrack).
  - *Test population*: 10 timber pieces (two takes per piece as we need to flip sides)
  - *Fix params*:
    - `fix timber position`: the timber piece will not move during the evaluation. It will be placed in a horizontal position and turned twice to track both sides. Each side will be considered as a separate evaluation session.
  - *Varying params*:
    - `timber shape`: 

- **(C) Influence of tags on tracking**: how the number and location of tags on the timber piece influence TSlam tracking's stability and accuracy.
  - *Evaluation set-up*:
  - *Test population*:
  - *Fix params*:
  - *Varying params*:


## List of abbreviations/vocabulary
- **GT**: ground truth
- **synthetic**: data which is computationally generated