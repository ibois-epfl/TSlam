#include "ts_rtag.hh"
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>

namespace tslam
{
    void TSRTag::setCorners(std::vector<Eigen::Vector3d> corners)
    {
        if (corners.size() != 4)
        {
            throw std::invalid_argument("[ERROR]: corners must be 4");
        }
        m_Corners.clear();
        m_Corners = corners;
    }

    void TSRTag::setCorners(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D)
    {
        m_Corners.clear();
        m_Corners.push_back(A);
        m_Corners.push_back(B);
        m_Corners.push_back(C);
        m_Corners.push_back(D);
    }

    void TSRTag::parseFromMAPYAML(const std::string& filename, std::vector<TSRTag>& planes)
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

            tslam::TSRTag plane = tslam::TSRTag();

            std::string id =  marker.substr(marker.find("id:") + 3, 3);
            plane.setID(std::stoi(id));

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

                Eigen::Vector3d A(xAF, yAF, zAF);
                Eigen::Vector3d B(xBF, yBF, zBF);
                Eigen::Vector3d C(xCF, yCF, zCF);
                Eigen::Vector3d D(xDF, yDF, zDF);

                plane.setCorners(A, B, C, D);

                break;
            }
            planes.push_back(plane);
        }
    }

    std::shared_ptr<open3d::geometry::TriangleMesh> TSRTag::toOpen3dMesh()
    {
        std::shared_ptr<open3d::geometry::TriangleMesh> mesh = std::make_shared<open3d::geometry::TriangleMesh>();

        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> triangles;

        vertices.push_back(Eigen::Vector3d(m_Corners[0](0), m_Corners[0](1), m_Corners[0](2)));
        vertices.push_back(Eigen::Vector3d(m_Corners[1](0), m_Corners[1](1), m_Corners[1](2)));
        vertices.push_back(Eigen::Vector3d(m_Corners[2](0), m_Corners[2](1), m_Corners[2](2)));
        vertices.push_back(Eigen::Vector3d(m_Corners[3](0), m_Corners[3](1), m_Corners[3](2)));

        triangles.push_back(Eigen::Vector3i(0, 1, 2));
        triangles.push_back(Eigen::Vector3i(0, 2, 3));

        mesh->vertices_ = vertices;
        mesh->triangles_ = triangles;

        m_PlaneMesh = *mesh;

        return mesh;
    }

    Eigen::Vector3d TSRTag::computeUnorientedPlaneNormal(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C)
    {
        Eigen::Vector3d AB = B - A;
        Eigen::Vector3d AC = C - A;
        Eigen::Vector3d normal = AB.cross(AC);
        return normal;
    }

    open3d::geometry::TriangleMesh& TSRTag::getOpen3dMesh()
    {
        if (m_Corners.size() != 4) throw std::runtime_error("[ERROR]: corners are not set.");

        toOpen3dMesh();
        return m_PlaneMesh;
    }

    Eigen::Vector3d& TSRTag::getUnorientedPlaneNormal()
    {
        if (m_Corners.size() != 4) throw std::runtime_error("[ERROR]: corners are not set.");

        m_UnorientedPlaneNormal = computeUnorientedPlaneNormal(m_Corners[0], m_Corners[1], m_Corners[2]);
        return m_UnorientedPlaneNormal;
    }

    Eigen::Vector3d TSRTag::computeCenter(std::vector<Eigen::Vector3d> corners)
    {
        Eigen::Vector3d center(((corners[0] + corners[1] + corners[2] + corners[3]) / 4.0).cast<double>());
        return center;
    }

    Eigen::Vector3d& TSRTag::getCenter()
    {
        if (m_Corners.size() != 4) throw std::runtime_error("[ERROR]: corners are not set.");

        m_Center = computeCenter(m_Corners);
        return m_Center;
    }
}