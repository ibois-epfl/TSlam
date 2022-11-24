#include "ts_plane.hh"
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>

namespace tslam
{
    TSPlane::TSPlane() {};

    void TSPlane::setCorners(std::vector<Eigen::Vector3f> corners)
    {
        if (corners.size() != 4)
        {
            throw std::invalid_argument("[ERROR]: corners must be 4");
        }
        m_corners.clear();
        m_corners = corners;
    }

    void TSPlane::setCorners(Eigen::Vector3f A, Eigen::Vector3f B, Eigen::Vector3f C, Eigen::Vector3f D)
    {
        m_corners.clear();
        m_corners.push_back(A);
        m_corners.push_back(B);
        m_corners.push_back(C);
        m_corners.push_back(D);
    }

    void TSPlane::parseFromMAPYAML(const std::string& filename, std::vector<std::shared_ptr<tslam::TSPlane>>& planes)
    {
        std::ifstream ifss(filename);
        if (!ifss.is_open())
        {
            throw std::runtime_error("[ERROR]: could not open file " + filename);
        }

        uint nmarkers = 0;
        const std::string keyNmarkers = "aruco_bc_nmarkers";
        std::string line;
        while (std::getline(ifss, line))
        {
            if (line.find(keyNmarkers) != std::string::npos)
            {
                nmarkers = std::stoi(line.substr(line.find(":") + 1));
                break;
            }
        }

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

        if (markersToParse.size() != nmarkers)
        {
            throw std::runtime_error("[Error]: number of markers to parse is not correct.");
        }


        for (uint i = 0; i < markersToParse.size(); i++)
        {
            std::string marker = markersToParse[i];

            std::shared_ptr<tslam::TSPlane> plane = std::make_shared<tslam::TSPlane>();

            std::string id =  marker.substr(marker.find("id:") + 3, 3);
            plane->setID(std::stoi(id));

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
    }

    o3dMeshPtr TSPlane::toOpen3dMesh()
    {
        o3dMeshPtr mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> triangles;

        vertices.push_back(Eigen::Vector3d(m_corners[0](0), m_corners[0](1), m_corners[0](2)));
        vertices.push_back(Eigen::Vector3d(m_corners[1](0), m_corners[1](1), m_corners[1](2)));
        vertices.push_back(Eigen::Vector3d(m_corners[2](0), m_corners[2](1), m_corners[2](2)));
        vertices.push_back(Eigen::Vector3d(m_corners[3](0), m_corners[3](1), m_corners[3](2)));

        triangles.push_back(Eigen::Vector3i(0, 1, 2));
        triangles.push_back(Eigen::Vector3i(0, 2, 3));

        mesh->vertices_ = vertices;
        mesh->triangles_ = triangles;

        m_PlaneMesh = *mesh;  //TODO: whatch out if this is creating problems (memory leak eg)

        return mesh;
    }
}