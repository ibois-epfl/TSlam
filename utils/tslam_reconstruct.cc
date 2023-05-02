#include "reconstruction/tslam_reconstructor.hh"

#include <iostream>
#include <filesystem>

// TODO: update the README with new flags
// TODO: refactor param parsing and flags
int main(int argc, char** argv)
{
    // parse the input args
    std::vector<std::string> args;
    for (int i = 1; i < argc; i++) {args.push_back(argv[i]);}

    std::string pathToMapYml;
    std::string output = std::filesystem::current_path().string();
    std::string meshName = "model" + std::to_string(std::time(nullptr));
    bool saveTagsAsPcd = false;
    float radiusSearch = 2.0;
    double creaseAngleThreshold = 5.0;
    int minClusterSize = 1;
    double aabbScaleFactor = 1.1;
    double maxPlnDist2Merge = 1.0;
    double maxPlnAngle2Merge = 5.0;
    double maxPolyDist = 1.0;
    double eps = 1e-05;

    if (args.size() == 0 || args[0][0] != '-' || args[0] == "-h" || args[0] == "--help")
    {
        std::cout << "Usage: " << "\n"
            << "------------------------------------------------" << "\n"
            << "This program reconstructs a mesh from a timber element map." << "\n"
            << "------------------------------------------------" << "\n"
            << "NEEDED args:" << "\n"
            << "[--pathToMapYml: the path to the yml file output by the tslam mapping]" << "\n"
            << "\n"
            << "OPTIONAL args for output:" << "\n"
            << "[--output: the output directory]" << "\n"
            << "[--meshName: the name of the output mesh model]" << "\n"
            << "[--saveTagsAsPcd: if enabled (just put the flag) it will create a pcd in .ply format with the tags' corners]" << "\n"
            << "\n"
            << "OPTIONAL args for the geometric solver:" << "\n"
            << "[--radiusSearch: radius search for the normal clustering of the tags]" << "\n"
            << "[--creaseAngleThreshold]: the threshold for detection of crease's angle (the smaller the more creases will be detected)" << "\n"
            << "[--minClusterSize: the minimal cluster possible of tags]" << "\n"
            << "[--aabbScaleFactor: the scale factor for scaleing up the AABB of the timber element map]" << "\n"
            << "[--maxPlnDist2Merge: the maximal distance between planes of stripes to be eligible for merging]" << "\n"
            << "[--maxPlnAngle2Merge: the maximal angle difference in degs between two planes'normals' angle to be eligible for merging]" << "\n"
            << "[--maxPolyDist: the maximal distance between a polygon and a tag to be considered as a candidate face]" << "\n"
            << "[--eps: the tolerance for all the computation (e.g. for the intersections)]" << "\n"
            << std::endl;
        return 1;
    }
    for (int i = 0; i < args.size(); i++)
    {
        if (args[i] == "--pathToMapYml") {pathToMapYml = args[i+1];}
        else if (args[i] == "--output") {output = args[i+1];}
        else if (args[i] == "--meshName") {meshName = args[i+1];}
        else if (args[i] == "--saveTagsAsPcd") {saveTagsAsPcd = true;}
        else if (args[i] == "--radiusSearch") {radiusSearch = std::stof(args[i+1]);}
        else if (args[i] == "--creaseAngleThreshold") {creaseAngleThreshold = std::stod(args[i+1]);}
        else if (args[i] == "--minClusterSize") {minClusterSize = std::stoi(args[i+1]);}
        else if (args[i] == "--aabbScaleFactor") {aabbScaleFactor = std::stod(args[i+1]);}
        else if (args[i] == "--maxPlnDist2Merge") {maxPlnDist2Merge = std::stod(args[i+1]);}
        else if (args[i] == "--maxPlnAngle2Merge") {maxPlnAngle2Merge = std::stod(args[i+1]);}
        else if (args[i] == "--maxPolyDist") {maxPolyDist = std::stod(args[i+1]);}
        else if (args[i] == "--eps") {eps = std::stod(args[i+1]);}
    }

    std::cout << "pathToMapYml: " << pathToMapYml << std::endl;
    std::cout << "output: " << output << std::endl;
    std::cout << "meshName: " << meshName << std::endl;

    // create a TSLAMReconstructor object
    tslam::Reconstruction::TSLAMReconstructor reconstructor;

    // set the parameters if provided
    reconstructor.setParams(
        radiusSearch,
        creaseAngleThreshold,
        minClusterSize,
        maxPlnDist2Merge,
        maxPlnAngle2Merge,
        aabbScaleFactor,
        maxPolyDist,
        eps
    );

    // load the map
    reconstructor.loadMap(pathToMapYml);

    // run the reconstruction
    if (reconstructor.run())
    {
        reconstructor.saveMeshAsPLY(output, meshName);
        std::cout << "[INFO]: mesh saved in " << output << "/" << meshName << ".ply" << std::endl;

        // save the tags
        if (saveTagsAsPcd)
        {
            std::string tags_filename = meshName;
            tags_filename += "_tags";
            reconstructor.saveTagsAsPly(output, tags_filename);
            std::cout << "[INFO]: tags saved in " << output << "/" << tags_filename << ".ply" << std::endl;
        }
    }

    return 0;
}