#include "ts_rtag.hh"
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>

namespace tslam::Reconstruction
{
    void TSRTag::setCorners(std::vector<Eigen::Vector3d> corners)
    {
        if (corners.size() != 4)
        {
            throw std::invalid_argument("[ERROR]: corners must be 4");
        }
        this->m_Corners.clear();
        this->m_Corners = corners;

        this->compute();
    }
    void TSRTag::setCorners(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D)
    {
        this->m_Corners.clear();
        this->m_Corners.push_back(A);
        this->m_Corners.push_back(B);
        this->m_Corners.push_back(C);
        this->m_Corners.push_back(D);

        this->compute();
    }

    void TSRTag::parseFromMAPYAML(const std::string& filename, std::vector<TSRTag>& tags)
    {
        // Open file
        std::ifstream ifss(filename);
        if (!ifss.is_open())
        {
            throw std::runtime_error("[ERROR]: could not open file " + filename);
        }

        // Get number of markers
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

        // Split string stream into seperate markers
        const std::string keyMarkers = "aruco_bc_marker";
        std::string marker2P;
        std::vector<std::string> markersToParse;
        while (std::getline(ifss, line))
        {
            if (line.find(keyMarkers) != std::string::npos)
            {
                uint k = 0;
                while(std::getline(ifss, line))
                {
                    if (line.find("corners") != std::string::npos && k != 0)
                    {
                        markersToParse.push_back(marker2P);
                        marker2P = "";
                    }

                    marker2P += line + "\n";

                    if ((markersToParse.size()) == nmarkers)
                    {
                        markersToParse.push_back(marker2P);
                        break;
                    }
                    k++;
                }
                markersToParse.push_back(marker2P);
                break;
            }
        }
        // Check number of markers entry with the number of markers found
        if (markersToParse.size() != nmarkers)
            throw std::runtime_error("[Error]: number of markers to parse is not correct.");
        
        // Parse markers
        for (uint i = 0; i < markersToParse.size(); i++)
        {
            std::string marker = markersToParse[i];

            TSRTag tag = TSRTag();

            // parse  id
            const std::string keyID = "id:";

            std::string id = marker.substr(marker.find(keyID) + keyID.size(), marker.find(",") - marker.find(keyID) - keyID.size());
            tag.setID(std::stoi(id));

            // parse the corners of the marker
            const std::string keyCorners = "corners:";
            while (marker.find(keyCorners) != std::string::npos)
            {
                marker = marker.substr(marker.find(keyCorners) + keyCorners.size());  // ORI

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

                Eigen::Vector3d A(xAF, yAF, zAF);
                Eigen::Vector3d B(xBF, yBF, zBF);
                Eigen::Vector3d C(xCF, yCF, zCF);
                Eigen::Vector3d D(xDF, yDF, zDF);

                // std::cout << "A: " << A << std::endl;  // <<<<<<< DEBUG
                // std::cout << "B: " << B << std::endl;  // <<<<<<< DEBUG
                // std::cout << "C: " << C << std::endl;  // <<<<<<< DEBUG
                // std::cout << "D: " << D << std::endl;  // <<<<<<< DEBUG

                // fill TSRTag's info
                tag.setCorners(A, B, C, D);
                tag.compute();

                break;
            }
            // store TSRTag
            tags.push_back(tag);
        }
    }

    void TSRTag::compute()
    {
        if (this->m_Corners.size() != 4) throw std::runtime_error("[ERROR]: corners are not set.");
        this->computeCenter();
        this->computePlaneEquation();
        this->computeOpen3dMesh();
    }
    void TSRTag::computeCenter()
    {
        Eigen::Vector3d center((this->m_Corners[0] + 
                                this->m_Corners[1] + 
                                this->m_Corners[2] + 
                                this->m_Corners[3]) / 4.0);
        this->m_Center = center;
    }
    void TSRTag::computePlaneEquation()
    {
        Eigen::Vector3d p1 = this->m_Corners[0];
        Eigen::Vector3d p2 = this->m_Corners[1];
        Eigen::Vector3d p3 = this->m_Corners[3];
        Eigen::Vector3d p4 = this->m_Corners[2];

        Eigen::Vector3d v1 = p2 - p1;
        Eigen::Vector3d v2 = p3 - p1;

        Eigen::Vector3d n = v1.cross(v2);
        n.normalize();

        double d = n.dot(p1);

        TSPlane tsplane = TSPlane(n[0], n[1], n[2], d);

        this->m_Plane = tsplane;
    }
    void TSRTag::computeOpen3dMesh()
    {
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> triangles;

        vertices.push_back(Eigen::Vector3d(this->m_Corners[0](0), this->m_Corners[0](1), this->m_Corners[0](2)));
        vertices.push_back(Eigen::Vector3d(this->m_Corners[1](0), this->m_Corners[1](1), this->m_Corners[1](2)));
        vertices.push_back(Eigen::Vector3d(this->m_Corners[2](0), this->m_Corners[2](1), this->m_Corners[2](2)));
        vertices.push_back(Eigen::Vector3d(this->m_Corners[3](0), this->m_Corners[3](1), this->m_Corners[3](2)));

        triangles.push_back(Eigen::Vector3i(0, 1, 2));
        triangles.push_back(Eigen::Vector3i(0, 2, 3));

        mesh->vertices_ = vertices;
        mesh->triangles_ = triangles;

        this->m_PlaneMesh = *mesh;
    }
}