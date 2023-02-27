graph TD;
    RGBFrame-->Mapping;
    RGBFrame-->Tracking;

    TagVocabulary-->Mapping;
    TagVocabulary-->Tracking;

    Mapping-->MapFile;

    MapFile-->Tracking;

    MapFile-->Reconstruction
    Reconstruction-->3DModel

    3DModel-->Tracking

    Tracking-->CameraPose
    Tracking-->UI