#include "reconstruction/tslam_reconstructor.hh"

#include <iostream>

// TODO: refactor param parsing and flags
int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << "\n"
            << "----------------------------------------" << "\n"
            << "This program reconstructs a mesh from a timber element map." << "\n"
            << "----------------------------------------" << "\n"
            << "NEEDED Arguments:" << "\n"
            << "<path_to_map_yml>" << "\n"
            << "<output_dir>" << "\n"
            << "<mesh_name>" << "\n"
            << "----------------------------------------" << "\n"
            << "OPTIONAL Arguments for the geometric solver:" << "\n"
            << "radiusSearch" << "\n"
            << "creaseAngleThreshold" << "\n"
            << "minClusterSize" << "\n"
            << "maxPlnDist" << "\n"
            << "maxPlnAngle" << "\n"
            << "aabbScaleFactor" << "\n"
            << "maxPolyDist" << "\n"
            << "eps" << "\n"
            << std::endl;
        return 1;
    }

    // create a TSLAMReconstructor object
    tslam::Reconstruction::TSLAMReconstructor reconstructor;

    // set the parameters if provided
    if (argc == 12)
    {
        reconstructor.setParams(std::stof(argv[4]), std::stod(argv[5]), std::stoi(argv[6]), std::stod(argv[7]), std::stod(argv[8]), std::stod(argv[9]), std::stod(argv[10]), std::stod(argv[11]));
    }

    // load the map
    reconstructor.loadMap(argv[1]);

    // run the reconstruction
    if (reconstructor.run())
    {
        // save the mesh
        reconstructor.saveMeshAsPLY(argv[2], argv[3]);

        // save the tags
        std::string tags_filename = argv[3];
        tags_filename += "_tags";
        reconstructor.saveTagsAsPly(argv[2], tags_filename);
    }
    std::cout << "[INFO]: mesh saved in " << argv[2] << "/" << argv[3] << ".ply" << std::endl;
    return 0;
}