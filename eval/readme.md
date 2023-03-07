# Evaluation protocol of TSlam

- [Evaluation protocol of TSlam](#evaluation-protocol-of-tslam)
  - [Objectives](#objectives)
  - [Methodology](#methodology)
    - [i - Evaluation of the 3d model](#i---evaluation-of-the-3d-model)
      - [Variables](#variables)
      - [Step-by-step overview](#step-by-step-overview)
    - [ii - Evaluation of the camera trajectory](#ii---evaluation-of-the-camera-trajectory)
      - [Variables](#variables-1)
      - [Step-by-step overview](#step-by-step-overview-1)
  - [General Notes](#general-notes)


---
## Objectives
This folder contains all the documents describing the evaluation designed for TSlam.
The TSlam *is an hybrid monocular camera's pose localization algorithm based on both direct feature detection and fiducial markers*.

The scope of the current evaluation protocol is limited to assest the two most important criteria for fabrication in wood working:
- **i)** the accuracy of the reconstruction model to produce the fabrication drawings
- **ii)** the accuracy of the camera to locate itself at runtime during fabrication.

## Methodology
To gauge these metrics, we will test TSlam in multiple real-life scenarios where the user is tasked with the fabrication of popular joineries most commonly used in timber carpentry. During the fabrication sequence, the TSlam will be tested with real-life noises and interferences typical of timber manual fabrication such as chips, vibrations, view obstructions (e.g. from the tool head), rapid movement, extremely close captures, etc.

To evaluate the two enounced evaluation targets, we will: for **(i)** obtain a scan of the reconstructed piece and compare it to the reconstructed model from SLAM. For gauging the second goal **(ii)**, we will record the fabrication with an Optitrack system able of recording the ground truth camera's pose per each frame. The recorded video will be fed to the TSlam and the computed pose will be calculated. Finally, the TSlam's recorded trajectory and its corresponding ground truth will be evaluated following state-of-the-art SLAM metrics (e.g. average relative translation (ART), and average relative rotation (ARR), or absolute trajectory error (ATE)).

To resume the evaluation design:
```mermaid
graph LR
    subgraph Evaluation objectives
      %% subgraph goals
        A(i. accuracy of 3D model)
        B(ii. accuracy of tracking)
      %% end
      %%subgraph variables
      %%  var1(distribution of tags)
      %%  var2(density of tags)
      %%  var3(type of joineries)
      %%  var4(fabrication piece shape)
      %%end
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
    end

    k --> A
    j --> B

    A --> AA
    B --> BB
```

In the following two chapters we are going to describe in details the modalities of evaluation for the first and second objectives of the protocol.s

---
### i - Evaluation of the 3d model
intro
#### Variables

#### Step-by-step overview
```mermaid
graph LR
    subgraph  
      O[timber beam]
      O --> A
      O --> E

      A(apply STags on the timber piece)
      --> B(run the TSlam mapping)
      --> C(run the TSlam reconstruction)
      --> D{{TSlam \n mesh}}

      E(scan the timber piece with FAROS)
      --> G{{GT \n mesh}}

      compare[mesh comparison \n Hudson distance]
      G --> compare
      D --> compare

      compareR{error between \n models}
      compare --> compareR
    end
```

---
### ii - Evaluation of the camera trajectory
intro

(*) to be able to compare the Optitrack ground truth for the camera pose with the TSlam we need to apply the Optitrack transformation recorded for the timber to the Optitrack camera. The objective is to bring the camera pose expressed in a global coordinate system, to the timber's coordinate system as in the TSlam.

(**) the trajectories will be first put back to the same coordinate system with the Horn transform. Next, we consider only the values in which the tracking is active and compare the absolute
trajectory error (ATE, rotation + translation).

#### Variables

#### Step-by-step overview
```mermaid
graph LR

    %% styling
    subG_prep:::subGStyl
    %% classDef subGStyl padding-left:30em;


    subgraph  
      O[timber beam]
      OO[camera]
      O -.-> optPrep1
      O -.-> mark
      O -.-> tslamPr1
      OO -.-> optPrep1


      subgraph subG_prep[1. preparation]
        optPrep1(apply Optitrack beacons on objects)
        optPrep2(register objects in Optitrack)
        optPrep1 --> optPrep2

        rigidB1[camera rigid body]
        rigidB2[timber rigid body]
        optPrep2 --> rigidB1
        optPrep2 --> rigidB2

        mark(apply marks \n for cutting/drilling)
        tslamPr1(apply STags on the timber piece)
        tslamPr2(run tslam mapping)
        tslamPr1 --> tslamPr2
        tslamPrR{{Tslam map}}
        tslamPr2 --> tslamPrR
      end


      subgraph subG_exec[2. execution of cutting/drilling]
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
      end


      subgraph subG_data[4. data processing]
        proc1(* combine transform: Tr x Tt)
        optR2 -.-> proc1
        optR3 -.-> proc1
        procR1{{Tc: GT camera \n transform}}
        proc1 --> procR1

        compare(** compare trajectories)
        procR1 --> compare
        tslamR1 -.-> compare
        compareR1{trajectories \n error}
        compare --> compareR1
      end

    end
```

## General Notes
The evaluation protocol is designed to assess parameters that are important for the fabrication process. As every other SLAM, also TSlam could be evaluated under many other aspects and criteria proper to the computer vision domain. Nevertheless, we limit the evaluation to the obtention of quantitative data only for those parameters impacting operational aspects of the developed SLAM pipeline. We will mention all relevant state-of-the-art evaluation methods and will make public the collected data and source code for further computer-vision fundamental analysis on TSlam.