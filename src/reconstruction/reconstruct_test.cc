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

    std::vector<std::shared_ptr<tslam::TSPlane>> planes;

    // Parse the markers string
    for (uint i = 0; i < markersToParse.size(); i++)
    {
        // current marker to parse
        std::string marker = markersToParse[i];

        // Parse the marker string
        std::shared_ptr<tslam::TSPlane> plane = std::make_shared<tslam::TSPlane>();

        // Set id
        std::string id =  marker.substr(marker.find("id:") + 3, 3);
        plane->setID(std::stoi(id));

        // Find the corners
        const std::string keyCorners = "corners";
        std::vector<Eigen::Vector3f> corners;
        while (marker.find(keyCorners) != std::string::npos)
        {
            marker = marker.substr(marker.find("corners") + 8);

            // std::cout << marker << std::endl;

            // std::string cornerA = marker.substr(0, marker.find("]") + 1);
            // marker = marker.substr(marker.find("]") + 1);

            // std::cout << cornerA << std::endl;


            // split the marker string with char ","
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

            


            // // erase all chars [ ] { } from the tokens
            // for (uint i = 0; i < tokens.size(); i++)
            // {
            //     tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(), '['), tokens[i].end());
            //     tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(), ']'), tokens[i].end());
            //     tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(), '{'), tokens[i].end());
            //     tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(), '}'), tokens[i].end());
            // }

            // for (auto token : tokens)
            // {
            //     std::cout << token << std::endl;
            // }

            // return 0;




            // std::string cornerA2Parse = cornerA.substr(cornerA.find("[") + 3, cornerA.find("]") - cornerA.find("[") - 3);

            // // get the next 3 numbers from corner

            // std::string cornerB2Parse = cornerA2Parse.substr(cornerA2Parse.find(",") + 2);


            // std::cout << cornerA2Parse << std::endl;
            // std::cout << cornerB2Parse << std::endl;


            // return 0;

            // std::stringstream ssA(cornerA2Parse);
            // std::stringstream ssB(cornerB2Parse);
            // std::stringstream ssC(cornerC2Parse);
            // std::stringstream ssD(cornerD2Parse);

            // std::string xA, yA, zA, xB, yB, zB, xC, yC, zC, xD, yD, zD;

            // std::vector<std::string> seglistA, seglistB, seglistC, seglistD;

            // while (std::getline(ssA, xA, ','))
            // {
            //     seglistA.push_back(xA);
            // }
            // while (std::getline(ssB, xB, ','))
            // {
            //     seglistB.push_back(xB);
            // }
            // while (std::getline(ssC, xC, ','))
            // {
            //     seglistC.push_back(xC);
            // }
            // while (std::getline(ssD, xD, ','))
            // {
            //     seglistD.push_back(xD);
            // }

            // //print the vector
            // for (uint i = 0; i < seglistA.size(); i++)
            // {
            //     std::cout << seglistA[i] << std::endl;
            //     std::cout << seglistB[i] << std::endl;
            // }


            // return 0;


            // float xAF, yAF, zAF, xBF, yBF, zBF, xCF, yCF, zCF, xDF, yDF, zDF;
            // xAF = std::stof(seglistA[0]);
            // yAF = std::stof(seglistA[1]);
            // zAF = std::stof(seglistA[2]);
            // xBF = std::stof(seglistB[0]);
            // yBF = std::stof(seglistB[1]);
            // zBF = std::stof(seglistB[2]);
            // xCF = std::stof(seglistC[0]);
            // yCF = std::stof(seglistC[1]);
            // zCF = std::stof(seglistC[2]);
            // xDF = std::stof(seglistD[0]);
            // yDF = std::stof(seglistD[1]);
            // zDF = std::stof(seglistD[2]);


            std::cout << "id: " << id << std::endl;
            std::cout << "corner A: " << xAF << ", " << yAF << ", " << zAF << std::endl;
            std::cout << "corner B: " << xBF << ", " << yBF << ", " << zBF << std::endl;
            std::cout << "corner C: " << xCF << ", " << yCF << ", " << zCF << std::endl;
            std::cout << "corner D: " << xDF << ", " << yDF << ", " << zDF << std::endl;

            break;
        }



        // // Set the corners
        // plane->setCorners(corners);

        // // Add the plane to the list
        // planes.push_back(plane);
    }




    return 0;
}