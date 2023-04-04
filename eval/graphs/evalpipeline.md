
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

      proc2(allign trajectories)
      proc2 --> proc2ATE(realligned \n entire trajectory)
      proc2 --> proc2RE(realligned only \n fabrication sequences)
      proc2ATE --> Ts_ar_gt{{Ts_ar_gt: TSlam \n trajectory alligned to GT}}
      proc2RE --> Ts_rr_gt{{Ts_rr_gt: TSlam trajectory \n sequences alligned to GT}}
      proc2ATE --> To_ar_gt{{To_ar_gt: TSlam \n trajectory alligned to GT}}
      proc2RE --> To_rr_gt{{To_rr_gt: TSlam trajectory \n sequences alligned to GT}}

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
      procR1 -.-> compareT_ATE_ORBSLAM_GT

      compareT_RE_ORBSLAM_GT(compare RE ORBSLAM/GT)
      To_rr_gt -.-> compareT_RE_ORBSLAM_GT
      procR1 -.-> compareT_RE_ORBSLAM_GT

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