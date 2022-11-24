#include <iostream>
#include "ts_plane.hh"

// files from: https://github.com/jimmiebergmann/mini-yaml
// #include "YAML.hpp"

#include <Eigen/Core>


#include <iostream>
#include <fstream>


// test files in https://drive.google.com/drive/folders/1wYFZq54syWwTFVQ5soJTMVUmcufcvQoT?usp=share_link
// if you have problem installing open3d:
// missing libc++ --> sudo apt install clang lldb lld


/*
[1] import and parse the yaml files into a vector of planes
[2] display those planes in a 3d window
*/


int main()
{
    std::vector<std::shared_ptr<tslam::TSPlane>> planes;

    // File location
    const std::string FILENAME = "/home/as/TSlam/src/reconstruction/long_comb.yml";

    // Set the fstream
    std::ifstream ifss(FILENAME);
    if (!ifss.is_open())
    {
        std::cout << "Could not open file " << FILENAME << std::endl;
        return 1;
    }

    // Find the number of aruco markers
    const std::string keyNmarkers = "aruco_bc_nmarkers";
    std::string line;
    uint nmarkers = 0;
    while (std::getline(ifss, line))
    {
        if (line.find(keyNmarkers) != std::string::npos)
        {
            nmarkers = std::stoi(line.substr(line.find(":") + 1));
            std::cout << "Found " << nmarkers << " markers" << std::endl;
            break;
        }
    }

    // Find the aruco marker data
    const std::string keyMarkers = "aruco_bc_marker";
    std::vector<uint> markerIds;

    std::vector<std::string> markersToParse;

    while (std::getline(ifss, line))
    {
        if (line.find(keyMarkers) != std::string::npos)
        {
            // run a loop every 7 lines
            uint j = 0;
            for (uint i = 0; i < nmarkers; i++)
            {
                std::string marker2P;
                for (uint j = 0; j < 7; j++)
                {
                    std::getline(ifss, line);
                    marker2P += line + "\n";
                }
                markersToParse.push_back(marker2P);
            }

            // std::string marker2Parse;
            // for (uint i = 0; i < 7; i++)
            // {
            //     std::getline(ifss, line);
            //     marker2Parse += line;
            // }
            // std::cout << "Found marker: " << marker2Parse << std::endl;
            // markersToParse.push_back(marker2Parse);






            break;
        }
    }



    // =======================================================================

    // std::string yamlContent((std::istreambuf_iterator<char>(ifss)), std::istreambuf_iterator<char>());


    // // find the "aruco_bc_nmarkers" key and save the value in an integer
    // std::string key = "aruco_bc_nmarkers";
    // std::size_t found = yamlContent.find(key);
    // std::cout << "found: " << found << std::endl;
    // std::string value = yamlContent.substr(found + key.size() + 2, 1);
    // std::cout << "value: " << value << std::endl;
    // int nmarkers = std::stoi(value);
    // std::cout << "nmarkers: " << nmarkers << std::endl;

    // =======================================================================






    // std::ifstream file(FILENAME);
    // if (!file.is_open())
    // {
    //     std::cout << "Could not open file " << FILENAME << std::endl;
    //     return 1;
    // }

    // std::string line;
    // std::vector<std::string> tokens;
    // std::string token;






    // while (std::getline(file, line))
    // {
    //     //find line with "aruco_bc_markers"
    //     std::size_t found = line.find("aruco_bc_markers");
    //     if (found!=std::string::npos)
    //     {
    //         std::cout << "found aruco_bc_markers" << std::endl;

    //         // int cap = 255;

    //         // read the next 7 lines
    //         for (int i = 0; i < 7; i++)
    //         {
    //             std::getline(file, line);
    //             token += line;
    //         }

    //         std::cout << token << std::endl;




    //         // // parse the token
    //         // std::size_t pos = 0;
    //         // std::string delimiter = ":";
    //         // while ((pos = token.find(delimiter)) != std::string::npos)
    //         // {
    //         //     token.erase(0, pos + delimiter.length());
    //         //     tokens.push_back(token);
    //         // }

    //         // std::cout << tokens[1] << std::endl;


    //         // read file at line ix

    //         // break;

    //         // print number of line
    //         // from now on, read everything betweeen "{}"
    //         std::string line;

    //     }

    // }


    // YAML::Node root;
    // YAML::Parse(root, FILENAME.c_str());
    // YAML::Parse(root, "/home/as/TSlam/src/reconstruction/long_comb.yml");




    // std::cout << root.As<std::string>() << std::endl;

    // for (auto it = root.Begin(); it != root.End(); ++it)
    // {
    //     std::shared_ptr<tslam::TSPlane> plane = std::make_shared<tslam::TSPlane>();
    //     std::vector<Eigen::Vector3f> corners;
    //     for (auto it2 = it->Begin(); it2 != it->End(); ++it2)
    //     {
    //         Eigen::Vector3f corner;
    //         corner << it2->second[0].as<float>(), it2->second[1].as<float>(), it2->second[2].as<float>();
    //         corners.push_back(corner);
    //     }
    //     plane->setCorners(corners);
    //     planes.push_back(plane);
    // }







    return 0;
}