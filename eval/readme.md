
# Evaluation protocol of TSlam

- [Evaluation protocol of TSlam](#evaluation-protocol-of-tslam)
  - [Objectives](#objectives)
  - [Methodology](#methodology)
    - [Evaluation variables and repetitions](#evaluation-variables-and-repetitions)
    - [Evaluation pipeline](#evaluation-pipeline)
      - [(i) Reconstruction](#i-reconstruction)
      - [(ii) Camera trajectory](#ii-camera-trajectory)
    - [Data processing](#data-processing)
      - [(i) Reconstruction](#i-reconstruction-1)
      - [(ii) Camera trajectory](#ii-camera-trajectory-1)
    - [Data analysis (error metrics)](#data-analysis-error-metrics)
      - [(i) TSlam reconstruction](#i-tslam-reconstruction)
      - [(ii) Camera trajectory](#ii-camera-trajectory-2)
  - [General Notes](#general-notes)
  - [Annexe](#annexe)


---
## Objectives
This folder contains all the documents describing the evaluation designed for TSlam.
The TSlam *is an hybrid monocular camera's pose localization algorithm based on both direct feature detection and fiducial markers*.

The scope of the current evaluation protocol is limited to assist the two most important criteria for fabrication in woodworking:
- **i)** the accuracy of the reconstruction model, useful to produce fabrication drawings
- **ii)** the accuracy of the camera to locate itself at runtime during fabrication.

## Methodology
To gauge these metrics, we will test TSlam in multiple real-life scenarios where the user is tasked with the fabrication of popular joineries most commonly used in timber carpentry. During the fabrication sequence, the TSlam will be tested with real-life noises and interferences typical of timber manual fabrication such as chips, vibrations, view obstructions (e.g. from the tool head), rapid movement, extremely close captures, etc.

To evaluate the two enounced evaluation targets we designed an evaluation pipeline identical for each repetition, independently from the experimental parameters identified as worthy of variation. The evaluation pipeline will always outcome the two error target values informing about (**i**) the accuracy of the reconstruction model, and (**ii**) the accuracy of the calculated camera's trajectory.

The first error value (**i**), it will be obtained by comparing the reconstructed model from TSlam and a ground truth model obtained by high-precision laser scanning. For gauging the second goal **(ii)**, we will record the fabrication with an Optitrack system able of recording the ground truth camera's pose per frame. The recorded video during the fabrication will be fed to the TSlam and the computed pose will be calculated. Finally, the TSlam's recorded trajectory and its corresponding ground truth will be evaluated following state-of-the-art SLAM metrics. For term of comparison, the same dataset will be run with a state-of-the-art markless SLAM alternative (ORB-SLAM3) and the results will be compared to TSLAM.

To resume the evaluation design:
```mermaid
graph LR
    subgraph Evaluation objectives
        A(i. accuracy of 3D model)
        B(ii. accuracy of tracking)
    end

    subgraph TSlam software
      a(1. Mapping)
      b(2. Reconstruction)
      c(3. Tracking)

      z((map))
      k((3D \n model))
      j((camera \n trajectory))

      a --> z
      b --> k
      c --> j

      z --> b
      z --> c
      k --> c
    end

    subgraph Evaluation methodology
      AA(Scanning and model comparison)
      BB(Optitrack dataset and trajectory comparison)

    k --> A
    j --> B

    A --> AA
    B --> BB
  end
```
<p align = "center">
  <i>Fig.1 - Overview of the objectives and methodology for the evaluation.</i>
</p>

---
### Evaluation variables and repetitions
In this chapter we present the variables and fix parameters identified in TSlam. The following scheme defines all the variables. Here's some clarifications on some choices made for selecting determined intervals or defined values for the parameters:

* `fabrication tools`: we will limit the fabrication to the use of a saber saw and drill(with different mesh bits).
* `timber dimensions`: 14x14x200cm. This size is defined by the maximal tracking area of the Optitrack set up. The square session 
* `type of joineries`: we selected 4 of the most famous type of joineries for carpentry (scarf, full, half-, and sliced lap). One for each face so that we need to turn the piece. For holes we want a variation in angles (30-60 deg) and two types of drilling (for washers and pegs/dowels) with 3 different types of the most commonly used mesh bits (spiral mesh bits in two lengths, and a washer mesh bit).
* `distribution and density of tags`: we limit the scope of the use of tags by stripes because we consider this as a plausible use rather than attaching tags one by one. We explore two orientations (long and short axis) and 3 densities.
* `timber shape`: it is worthy of note also that we considered as a variable the initial shape of the timber piece at the beginning of each fabrication session. The timber piece will be processed so that to present either 1,2,3, or 4 joints before the recording session will start. This is not so influential in the evaluation of tracking (**ii**) but rather for evaluating the reconstruction capabilities of TSlam (**i**) of obtaining 3D models of pre-processed and complex geometries (i.e. timber elements presenting irregularities or joineries). Each specimen will present the complete number and types of drilling and cutting joinery (12 holes + 4 joints).

Next, we combined the selected parameters to obtain a combination matrix for the evaluation.


```mermaid
  flowchart LR

    Variables
    Variables --> Cc
    Variables --> J
    Variables --> Ti
    Variables --> T

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Cc(camera)
    Cc --> Ccr(resolution)
    Ccr --> Ccr1(1280 x 720)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    J[joinery]
    J --> Cuts
    J --> Hole

    Cuts[Cuts]
    Cuts --> CutsT
    Cuts --> CutsNum
    Cuts --> CutsTH
    CutsT[types]
    CutsT_S(joint scar)
    CutsT_H(joint half lap)
    CutsT_L(joint full lap)
    CutsT_N(joint sliced lap)
    CutsTH(toolhead)
    CutsTH --> CutsTH_1(sabersaw blade)
    CutsT --> CutsT_S
    CutsT --> CutsT_H
    CutsT --> CutsT_L
    CutsT --> CutsT_N
    CutsNum[number]
    CutsNum --> CutsNum_1(1 per piece)
    CutsNum --> CutsNum_2(2 per piece)
    CutsNum --> CutsNum_3(3 per piece)
    CutsNum --> CutsNum_4(4 per piece)

    Hole[Hole]
    Hole --> HoleT
    Hole --> HoleN
    Hole --> HoleDeg
    HoleT[toolhead]
    HoleT --> HoleT1(washer head d:50mm, L:15cm)
    HoleT --> HoleT2(spiral piercing head d:20mm, L:20 cm)
    HoleT --> HoleT3(spiral piercing head d:20mm, L:35 cm)
    HoleT --> HoleT4(timber screws d:20mm, L:10 cm)
    HoleN[number]
    HoleN --> HoleN_10(25 per piece)
    HoleDeg(angle)
    HoleDeg --> HoleDeg_90(90deg)
    HoleDeg --> HoleDeg_45(45deg)
    HoleDeg --> HoleDeg_30(30deg)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Ti(timber)
    Ti --> Tdim
    Ti --> TNcuts

    Tdim[timber dimensions]
    Tdim --> Td1(140x140x2000)
    TNcuts[timber initial number of cuts]
    
    TNcuts[number of pre-existing cuts]
    TNcuts --> TNcuts_1(1 joint per piece)
    TNcuts --> TNcuts_2(2 joints per piece)
    TNcuts --> TNcuts_3(3 joints per piece)
    TNcuts --> TNcuts_4(4 joints per piece)

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    T(tags)
    T --> TDt
    T --> TDd

    TDt[tag distribution]
    TDt --> TDt1(longitudinal to beam's length)
    TDt --> TDt2(horizontal to beam's length)

    TDd[tag density]
    TDd --> TDd1(low)
    TDd --> TDd2(medium)
    TDd --> TDd3(high)
```
<p align = "center">
  <i>Fig.2 - Scheme of all the variations of parameters selected for the study.</i>
</p>

![image_joineries](./img/sketch1.png)
<p align = "center">
  <i>Fig.3 - Image of all the joineries that will be done on each timber at the end of the repetition. Some timber pieces will present either 1,2,3, or 4 joints before the recorded fabrication session will start to test mainly the 3D reconstruction algorithm of TSlam.</i>
</p>

|        stripe distribution           |          ring distribution           |
|:----------------------:|:-----------------------:|
| ![](./img/stripe_1.png) | ![](./img/ring_1.png)  |
| ![](./img/stripe_2.png) | ![](./img/ring_2.png)  |
| ![](./img/stripe_3.png) | ![](./img/ring_3.png)  |
<p align = "center">
  <i>Fig.4 - Illustrations of the two layouts defined for the evaluation. For each layout we define 3 different level of densities. Tags' distribution and density will impact both the reconstruction (**i**) and tracking (**ii**) accuracy.</i>
</p>

<table border="1" class="dataframe">
  <tbody>
    <tr>
      <td>speciment index</td>
      <td>timber dimensions</td>
      <td>cuts itype/number in initial state</td>
      <td>drills type/number to execute</td>
      <td>tags distribution</td>
      <td>tags density</td>
    </tr>
    <tr>
      <td>1</td>
      <td>14x14x2000</td>
      <td>[]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>2</td>
      <td>14x14x2000</td>
      <td>[]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>3</td>
      <td>14x14x2000</td>
      <td>[]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>4</td>
      <td>14x14x2000</td>
      <td>[]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>5</td>
      <td>14x14x2000</td>
      <td>[]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>6</td>
      <td>14x14x2000</td>
      <td>[1xscar]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>7</td>
      <td>14x14x2000</td>
      <td>[1xscar]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>8</td>
      <td>14x14x2000</td>
      <td>[1xscar]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>9</td>
      <td>14x14x2000</td>
      <td>[1xscar]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>10</td>
      <td>14x14x2000</td>
      <td>[1xscar]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>11</td>
      <td>14x14x2000</td>
      <td>[1xscar]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>12</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>13</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>14</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>15</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>16</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>17</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>18</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>19</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>20</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>21</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>22</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>23</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>24</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap, 1xspliced]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>25</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap, 1xspliced]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>26</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap, 1xspliced]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>stripe layout</td>
      <td>high density</td>
    </tr>
    <tr>
      <td>27</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap, 1xspliced]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>low density</td>
    </tr>
    <tr>
      <td>28</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap, 1xspliced]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>medium density</td>
    </tr>
    <tr>
      <td>29</td>
      <td>14x14x2000</td>
      <td>[1xscar, 1xhalf-lap, 1xfull-lap, 1xspliced]</td>
      <td>[8xspiral20, 2xspiral20, 2xwasher50, 12xscrews]</td>
      <td>ring layout</td>
      <td>high density</td>
    </tr>
  </tbody>
</table>

<p align = "center">
  <i>Fig.5 - Matrix of all possible combinations to test.</i>
</p>


---
### Evaluation pipeline
Each. The two outcomes will always be (**i**) the accuracy of the reconstructed model, and the accuracy of the TSlam tracking, i.e. the camera's trajectory (**ii**).

#### (i) Reconstruction
Each timber will be scanned to obtain a point cloud of the object. This will constitute our ground truth model for the reconstruction comparison with the reconstructed model from TSlam. 

#### (ii) Camera trajectory
After the application of the Optitrack and TSlam tracking beacons and the preparation of the necessary set-up for the MotiveOptitrack software to be operative, the beam is mapped with TSlam. The piece will be marked manually as in traditional carpentry practice. The fabrication will be carried out with manual electric tools (saber saw and drill). <u>The operator will not follow any augmented instructions and they will not be provided with any graphical support. The fabrication will be carried out by following the precedently applied marks</u>. This will get rid of any biases in the evaluation of TSlam. If the operator had visual feedback informing them on the TSlam tracking health, they might tend to modify the current wood-working action to ameliorate the visual feedback at their disposal, hence the tracking signal. Our goal for the construction of the dataset is to have a video sequence of common and unbiased woodworking movements. This very sequence will be fed to TSlam and ORBSlam for estimating camera trajectories that will be processed and evaluated in the last section.


```mermaid
graph LR

    %% styling
    subG_prep:::subGStylI
    %% classDef subGStylI padding-left:30em fill:#f9f;
    %% classDef subGStylII padding-left:30em;

    subG_prefab ==> 
    subG_prep ==>
    subG_exec ==>
    subG_post ==>
    subG_data ==>
    subG_analysis

    O[timber beam]
    OO[camera]
    O -.-> optPrep1
    O -.-> mark
    OO -.-> optPrep1

    subgraph subG_prefab[0. pre-fabrication]
      mark(apply marks \n for cutting/drilling)
      mark --> prefab(make cuts \n before fabrication)
    end


    subgraph subG_prep[1. preparation]
      optPrep1(apply Optitrack beacons on objects)
      optPrep2(register objects in Optitrack)
      optPrep1 --> optPrep2

      rigidB1[camera rigid body]
      rigidB2[timber rigid body]
      optPrep2 --> rigidB1
      optPrep2 --> rigidB2

      prefab -.-> tslamPr1(apply STags on the timber piece)
      
      tslamPr2(run tslam mapping)
      tslamPr1 --> tslamPr2
      tslamPrR{{Tslam map}}
      tslamPr2 --> tslamPrR

      scan(scan the timber)
      tslamPr1 --> scan
      scanR{{Scan model}}
      scan --> scanR
    end


    subgraph subG_exec[2. recorded fabrication]
      opti1(Optitrack tracking)
      rigidB1 -.-> opti1
      rigidB2 -.-> opti1
      optR1{{RGB \n frames}}
      optR2{{Tr: GT raw camera \n transform}}
      optR3{{Tt: GT timber \n transform}}
      opti1 --> optR1
      opti1 --> optR2
      opti1 --> optR3
    end


    subgraph subG_post[3. post fabrication]
      tslam1(TSlam tracking)
      optR1 -.-> tslam1
      tslamPrR -.-> tslam1
      tslamR1{{Ts: camera \n pose}}
      tslam1 --> tslamR1

      ORBSLAM3(ORBSLAM3 tracking)
      ORBSLAM3 --> ORBSLAM3_R1{{To: camera pose}}

      tslam2(TSlam model \n reconstruction)
      tslamPrR -.-> tslam2
      tslamR2{{Reconstructed model}}
      tslam2 --> tslamR2
    end


    subgraph subG_data[4. data processing]
      proc1(combine transform: Tr x Tt)
      optR2 -.-> proc1
      optR3 -.-> proc1
      procR1{{Tc: GT camera \n trajectory}}
      proc1 --> procR1
      procR1 --> proc2
      optR2 -.-> proc3

      proc2(allign TSLAM trajectories)
      proc2 --> proc2ATE(realligned \n entire trajectory)
      proc2 --> proc2RE(realligned only \n fabrication sequences)
      proc2ATE --> Ts_ar_gt{{Ts_ar_gt: TSlam \n trajectory alligned to GT}}
      proc2RE --> Ts_rr_gt{{Ts_rr_gt: TSlam trajectory \n sequences alligned to GT}}

      proc3(allign ORBSLAM trajectories)
      proc3 --> proc3ATE(realligned \n entire trajectory)
      proc3 --> proc3RE(realligned only \n fabrication sequences)
      proc3ATE --> To_ar_gt{{To_ar_gt: ORBSLAM3 \n trajectory alligned to GT}}
      proc3RE --> To_rr_gt{{To_rr_gt: ORBSLAM3 trajectory \n sequences alligned to GT}}

    end

    subgraph subG_analysis[5, data anlysis]
      %% style result
      classDef resultClass fill:#f66, color:#234F 

      %%%%%%%%%%%%%% trajectory errors %%%%%%%%%%%%%%
      compareT_ATE_TSLAM_GT(compare ATE TSlam/GT)
      Ts_ar_gt -.-> compareT_ATE_TSLAM_GT
      procR1 -.-> compareT_ATE_TSLAM_GT

      compareT_RE_TSLAM_GT(compare RE TSlam/GT)
      Ts_rr_gt -.-> compareT_RE_TSLAM_GT
      procR1 -.-> compareT_RE_TSLAM_GT

      compareT_ATE_ORBSLAM_GT(compare ATE ORBSLAM/GT)
      To_ar_gt -.-> compareT_ATE_ORBSLAM_GT
      optR2 -.-> compareT_ATE_ORBSLAM_GT

      compareT_RE_ORBSLAM_GT(compare RE ORBSLAM/GT)
      To_rr_gt -.-> compareT_RE_ORBSLAM_GT
      optR2 -.-> compareT_RE_ORBSLAM_GT

      compareT_ATE_TSLAM_GT --> ATE_TSLAM_translation{{ATE error for translation \n x,y,z}}:::resultClass %% <------ R
      compareT_ATE_TSLAM_GT --> ATE_TSLAM_rotation{{ATE error for rotation \n axis x,y,z}}:::resultClass %% <------- R
      compareT_RE_TSLAM_GT --> RE_TSLAM_translation{{RE error for translation \n x,y,z}}:::resultClass %% <--------- R
      compareT_RE_TSLAM_GT --> RE_TSLAM_rotation{{RE error for rotation \n axis x,y,z}}:::resultClass %% <---------- R

      compareT_ATE_ORBSLAM_GT --> ATE_ORBSLAM_translation{{ATE error for translation \n x,y,z}}:::resultClass %% <-- R
      compareT_ATE_ORBSLAM_GT --> ATE_ORBSLAM_rotation{{ATE error for rotation \n axis x,y,z}}:::resultClass %% <--- R
      compareT_RE_ORBSLAM_GT --> RE_ORBSLAM_translation{{RE error for translation \n x,y,z}}:::resultClass %% <----- R
      compareT_RE_ORBSLAM_GT --> RE_ORBSLAM_rotation{{RE error for rotation \n axis x,y,z}}:::resultClass %% <------ R

      %%%%%%%%%%%%%% comparison TSLMA/ORBSLAM errors %%%%%%%%%%%%%%
      compareTOSLAM_ATE(ATE comparison TSLAM/ORBSLAM)
      ATE_TSLAM_translation --> compareTOSLAM_ATE
      ATE_TSLAM_rotation --> compareTOSLAM_ATE
      ATE_ORBSLAM_translation --> compareTOSLAM_ATE
      ATE_ORBSLAM_rotation --> compareTOSLAM_ATE
      compareTOSLAM_ATE --> compareTOSLAM_ATE_R{{ATE diff TSLAM/ORBSLAM}}:::resultClass  %% <----------------------- R

      compareTOSLAM_RE(RE comparison TSLAM/ORBSLAM)
      RE_TSLAM_translation --> compareTOSLAM_RE
      RE_ORBSLAM_translation --> compareTOSLAM_RE
      RE_TSLAM_rotation --> compareTOSLAM_RE
      RE_ORBSLAM_rotation --> compareTOSLAM_RE
      compareTOSLAM_RE --> compareTOSLAM_RE_R{{RE diff TSLAM/ORBSLAM}}:::resultClass  %% <-------------------------- R

      %%%%%%%%%%%%%% reconstruction errors %%%%%%%%%%%%%%
      compareM(compare models)
      tslamR2-.-> compareM
      scanR-.-> compareM
      compareMR1{{models \n error}}:::resultClass  %% <------------------------------------------------------------- R
      compareM --> compareMR1
    end
```
<p align = "center">
  <i>Fig.6 - Description of the evaluation pipeline step by step.</i>
</p>

---
### Data processing
In this section we go into details of the data processing phase where we collect all the raw data out of the experimental activity, and process it so that it can be used in the next step for the analysis.

#### (i) Reconstruction
After the initial scanning for each timber piece, we have a ground truth point cloud ($Pcd_{gt}$) and the mesh from TSlam(`Mesh_ts`). To compare the two in the next phase we need to process the data in two steps:
- we first obtain a point cloud of the mesh by sampling the mesh into a point cloud($Pcd_{ts}$)
- next we align the two point clouds by finding the rigid RANSAC registration that defines the transformation $T_{reg}$ to apply to $Pcd_{ts}$ which becomes $Pcd_{ts}^i$, the error of the transformation is measured in *fitness* and *RMSE* of inlier points between the two clouds.

The total output processed data is the following:
- $Pcd_{gt}$: the scan point cloud of the timber piece
- $Pcd_{ts}^i$: the registered sampled point cloud from the tslam reconstructed 3D model

#### (ii) Camera trajectory
After the Optitrack tracking of the fabrication we obtain the following data:
- $T_t$: the transformation records from the Optitrack of the timber rigid body (translation + quaternions) expressed in a global coordinate system
- $T_r$: the transformation records from the Optitrack of the camera rigid body (translation + quaternions) expressed in a global coordinate system
- $f_{RGB}$: frames of the fabrication video sequence

Next, the TSlam runs on the recorded video sequence and output the following data:
- $T_s$: the transformation records from the TSLAM of the camera (translation + quaternions) expressed in the coordinate system of the timber piece (the object is fix and the camera moves around)
- $f_{RGB}^{overlay}$: the RGB frames with visual hints of markers or visual features detections

Now, in order to compare the two trajectories we need to do 3 operations:
- **(1)** *object-related frame system transposal for ground truth trajectory*: express $T_r$ in the coordinate of $T_t$ to obtain a new list of transformations named $T_c$. This will be our ground truth and it is necessary to have both $T_c$ and $T_s$ on the same timber beam's coordinate system.
- **(2)** *trajectory alignment for absolute trajectory error (**ATE**)*: now that the two sets of trajectories ($T_c$ and $T_s$) are expressed in the same local system we need to align them with the Horn transform[^1]. This will align $T_s$ to $T_c$ to obtain $T_{sa}$. We will do the same for the ORBSlam $T_o$ to $T_r$ to obtain a rigid transformation to apply to $T_o$ to obtain $T_{oa}$. For the ATE the transformation is made on the entirety of the trajectories.
- **(3)** *selection of intervals in trajectories and local alignement (**RE**)*: for the second type of metric, the relative one, we need to select the fabrication intervals (when cutting or drilling). The same intervals are locally aligned following the method by Zhang et al.[^2]. We apply this step to both the TSlam ($T_s$) and ORBSlam ($T_o$) trajectories with target the GT trajectory ($T_c$ for $T_s$ and $T_r$ for $T_o$) to obtain an ensemble of aligned sequences for TSlam, $A_{Tsr}$, and for ORBSlam, $A_{Tor}$

<p align = "center">
  <img src=img/evaltrajallign.png> </img>
</p>
<p align = "center">
  <i>Fig.7 - Two different alignement (and error distance calculation) for the ATE and RE. Ref: https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf</i>
</p>

The total output processed data is the following:
- $T_r$: the trajectory of the ground truth camera from the Optitrack recording expressed in global coordinates
- $T_c$: the ground truth trajectory expressed in the timber beam's coordinate system
- $T_{sa}$: the TSlam trajectory aligned to $T_c$
- $T_{oa}$: the ORBSlam trajectory aaligned to $T_r$
- $A_{Tsr}$: set of sub-sequences of TSlam trajectory alligned to $T_c$ corresponding sequences
- $A_{Tor}$: set of sub-sequences of ORBSLAM trajectory alligned to $T_r$ corresponding sequences
- $f_{RGB}$: video capture of the fabrication sequence
- $f_{RGB}^{overlay}$: video capture of the fabrication sequence with tslam graphical info overlay

With these processed data we are able to proceed to the next phase: the data analysis.

[^1]: the Horn transformation for trajectories' alignement: B. K. P. Horn, “Closed-form solution of absolute orientation using unit quaternions,” Journal of the Optical Society of America A, vol. 4, no. 4, p. 629, Apr. 1987. [Online]. Available: https://doi.org/10.1364/josaa.4.000629.

[^2]: tutorial on evaluation of trajectories: Z. Zhang and D. Scaramuzza, “A tutorial on quantitative trajectory evaluation for visual(-inertial) odometry,” in 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2018, pp. 7244–7251. [Online]. Available: https://github.com/uzh-rpg/rpg_trajectory_evaluation.

---
### Data analysis (error metrics)
In this section we run analysis on the processed data to obtain the final evaluation results for (**i**) quantifying the error of the reconstructed algorithm, (**ii**) and the distance between the TSlam computed trajectory and the ground-truth trajectory, the ORBSlam and the ground-truth trajectory, and a comparison of the two errors for the two SLAM systems.

#### (i) TSlam reconstruction
The evaluation of the accuracy of the reconstructed method is expressed by the hausdorff distance.

#### (ii) Camera trajectory
For both TSlam and ORBSLAM3 we compare the trajectories with the ground truth and espress the error as the distance between two trajectories. We consider two error distances:
- **AET**: absolute error distance on the entire trajectory for each component of translation and rotation. This is done both for the TSlam and ORBSlam
- **RE**: the relative error distance on the selected sub-trajectories corresponding to the cutting/drilling sequences for each component of translation and rotation. This is done both for the TSlam and ORBSlam.
- *error comparison between TSlam and ORBSlam*: we will compare the two eror sets from the TSlam and ORBSlam to qunatify the efficenc of TSlam over open-sourced markless state-of-the-art slam methods existing today.

At the end of the error estimation, these are the error values we obtain,
for the ATE:
- $ATE^{tslam}_{transX}$: the absolute trajectory error for the translation X component of TSlam
- $ATE^{tslam}_{transY}$: the absolute trajectory error for the translation Y component of TSlam
- $ATE^{tslam}_{transZ}$: the absolute trajectory error for the translation Z component of TSlam
- $ATE^{tslam}_{rotX}$: the absolute rotation error for the translation X component of TSlam
- $ATE^{tslam}_{rotY}$: the absolute rotation error for the translation Y component of TSlam
- $ATE^{tslam}_{rotZ}$: the absolute rotation error for the translation Z component of TSlam
- $ATE^{orbslam}_{transX}$: the absolute trajectory error for the translation X component of ORBSLAM3
- $ATE^{orbslam}_{transY}$: the absolute trajectory error for the translation Y component of ORBSLAM3
- $ATE^{orbslam}_{transZ}$: the absolute trajectory error for the translation Z component of ORBSLAM3
- $ATE^{orbslam}_{rotX}$: the absolute rotation error for the translation X component of ORBSLAM3
- $ATE^{orbslam}_{rotY}$: the absolute rotation error for the translation Y component of ORBSLAM3
- $ATE^{orbslam}_{rotZ}$: the absolute rotation error for the translation Z component of ORBSLAM3

and for the RE:
- $RE^{tslam}_{transX}$: the ensemble of the absolute trajectory errors for the translation X component of TSlam
- $RE^{tslam}_{transY}$: the ensemble of the absolute trajectory errors for the translation Y component of TSlam
- $RE^{tslam}_{transZ}$: the ensemble of the absolute trajectory errors for the translation Z component of TSlam
- $RE^{tslam}_{rotX}$: the ensemble of the absolute rotation errors for the translation X component of TSlam
- $RE^{tslam}_{rotY}$: the ensemble of the absolute rotation errors for the translation Y component of TSlam
- $RE^{tslam}_{rotZ}$: the ensemble of the absolute rotation errors for the translation Z component of TSlam
- $RE^{orbslam}_{transX}$: the ensemble of the absolute trajectory errors for the translation X component of ORBSLAM3
- $RE^{orbslam}_{transY}$: the ensemble of the absolute trajectory errors for the translation Y component of ORBSLAM3
- $RE^{orbslam}_{transZ}$: the ensemble of the absolute trajectory errors for the translation Z component of ORBSLAM3
- $RE^{orbslam}_{rotX}$: the ensemble of the absolute rotation errors for the translation X component of ORBSLAM3
- $RE^{orbslam}_{rotY}$: the ensemble of the absolute rotation errors for the translation Y component of ORBSLAM3
- $RE^{orbslam}_{rotZ}$: the ensemble of the absolute rotation errors for the translation Z component of ORBSLAM3


Finally we will express the errors difference betwen TSlam and ORBSLAM across in ATE and RE as two mean values.

The data will be graphically represented as follow:

- *the drifting of tslam in translation and orientation for the ATE*: for both the error values of TSlam and ORBSLAM3.
<p align = "center">
  <img src=img/drift.png> </img>
</p>
<p align = "center">
  <i>Fig.8 - Draft illustration for drifting values for ATE. Both ORBSLAM and TSLAM will be in the same. Ref: https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf</i>
</p>

- *boxplots for the translation and rotation error for each sequence RE*: for both the error values of TSlam and ORBSLAM3.
<p align = "center">
  <img src=img/boxplot.png> </img>
</p>
<p align = "center">
  <i>Fig.9 - Draft illustration for box plot RE for each trajectory sequence: error values of TSlam and ORBSLAM3 will be integrated in the same graph. REf: https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf</i>
</p>


## General Notes
The evaluation protocol is designed to assess parameters that are important for the fabrication process. As every other SLAM, also TSlam could be evaluated under many other aspects and criteria proper to the computer vision domain. Nevertheless, we limit the evaluation to the obtention of quantitative data only for those parameters impacting operational aspects of the developed SLAM pipeline. We will mention all relevant state-of-the-art evaluation methods and will make public the collected data and source code for further computer-vision fundamental analysis on TSlam.

## Annexe
- planning for TSLAM working package
- most relevant state of the art on evaluation of SLAM algorithms
- the "ordre de commande" for the  needed timber for the experience
§