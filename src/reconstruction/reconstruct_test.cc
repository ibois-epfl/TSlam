#include <iostream>
#include "ts_plane.hh"

#include <Eigen/Core>

#include <iostream>
#include <fstream>
#include <sstream>

#include <stdexcept>

// test files in https://drive.google.com/drive/folders/1wYFZq54syWwTFVQ5soJTMVUmcufcvQoT?usp=share_link
// if you have problem installing open3d:
// missing libc++ --> sudo apt install clang lldb lld


/*
[1] import and parse the yaml files into a vector of planes
[2] display those planes in a 3d window
*/


int main()
{
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
            break;
        }
    }

    // Find the aruco marker data
    const std::string keyMarkers = "aruco_bc_marker";
    std::vector<std::string> markersToParse;

    while (std::getline(ifss, line))
    {
        if (line.find(keyMarkers) != std::string::npos)
        {
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
            break;
        }
    }

    // Check if the number of markers found is correct
    if (markersToParse.size() != nmarkers)
    {
        throw std::runtime_error("[Error]: number of markers to parse is not correct.");
    }


    // =======================================================================


    // Parse the markers string
    std::vector<std::shared_ptr<tslam::TSPlane>> planes;
    for (uint i = 0; i < markersToParse.size(); i++)
    {
        // current marker to parse
        std::string marker = markersToParse[i];

        // Parse the marker string
        std::shared_ptr<tslam::TSPlane> plane = std::make_shared<tslam::TSPlane>();

        // Set id
        std::string id =  marker.substr(marker.find("id:") + 3, 3);
        plane->setID(std::stoi(id));

        // Find the corners points' cordinates
        const std::string keyCorners = "corners";
        while (marker.find(keyCorners) != std::string::npos)
        {
            marker = marker.substr(marker.find("corners") + 8);

            std::stringstream ss(marker);
            std::string token;
            std::vector<std::string> tokens;
            while (std::getline(ss, token, ','))
            {
                token.erase(std::remove(token.begin(), token.end(), '['), token.end());
                token.erase(std::remove(token.begin(), token.end(), ']'), token.end());
                token.erase(std::remove(token.begin(), token.end(), '{'), token.end());
                token.erase(std::remove(token.begin(), token.end(), '}'), token.end());
                token.erase(std::remove(token.begin(), token.end(), ' '), token.end());
                tokens.push_back(token);
            }

            float xAF, yAF, zAF, xBF, yBF, zBF, xCF, yCF, zCF, xDF, yDF, zDF;
            xAF = std::stof(tokens[0]);
            yAF = std::stof(tokens[1]);
            zAF = std::stof(tokens[2]);
            xBF = std::stof(tokens[3]);
            yBF = std::stof(tokens[4]);
            zBF = std::stof(tokens[5]);
            xCF = std::stof(tokens[6]);
            yCF = std::stof(tokens[7]);
            zCF = std::stof(tokens[8]);
            xDF = std::stof(tokens[9]);
            yDF = std::stof(tokens[10]);
            zDF = std::stof(tokens[11]);

            Eigen::Vector3f A(xAF, yAF, zAF);
            Eigen::Vector3f B(xBF, yBF, zBF);
            Eigen::Vector3f C(xCF, yCF, zCF);
            Eigen::Vector3f D(xDF, yDF, zDF);

            plane->setCorners(A, B, C, D);

            break;
        }
        planes.push_back(plane);
    }

    // print all the planes
    for (uint i = 0; i < planes.size(); i++)
    {
        std::cout << *planes[i] << std::endl;
        std::cout << "corner A: " << planes[i]->getCornerA().transpose() << std::endl;
        std::cout << "corner B: " << planes[i]->getCornerB().transpose() << std::endl;
        std::cout << "corner C: " << planes[i]->getCornerC().transpose() << std::endl;
        std::cout << "corner D: " << planes[i]->getCornerD().transpose() << std::endl;
    }

    return 0;
}