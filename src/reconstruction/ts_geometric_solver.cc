#include "ts_geometric_solver.hh"
#include<open3d/Open3D.h>
#include <stdexcept>
#include <algorithm>
#include <math.h>
#include <map>
#include <numeric>
#include <algorithm>
#include <iterator>


namespace tslam
{
    void TSGeometricSolver::reconstruct()
    {
        this->rDetectCreasesTags();
        this->rIntersectTagPlnAABB();
        this->rIntersectMeanPolygonPlnAABB();

        // // =============================================================================
        // testing
        // // =============================================================================

        // // intersection between two segments
        // TSSegment segT1 = TSSegment(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1));
        // TSSegment segT2 = TSSegment(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(1, 0, 1));

        // Eigen::Vector3d* intersectPtT = new Eigen::Vector3d();
        // bool isIntersectTRes = segT1.intersect(segT2, *intersectPtT);
        // std::cout << "isIntersect: " << isIntersectTRes << std::endl;

        // ask for user input
        // std::cout << "Press ENTER to continue..." << std::endl;
        // std::cin.ignore(std::numeric_limits<std::streamsize>::max(), ' ');

        // return;

        // test for polygon equals /////////////////////////////////////////////////

        // Eigen::Vector3d p1 = Eigen::Vector3d(0, 0, 0);
        // Eigen::Vector3d p2 = Eigen::Vector3d(10, 0, 0);
        // Eigen::Vector3d p3 = Eigen::Vector3d(0, 10, 0);
        // Eigen::Vector3d p4 = Eigen::Vector3d(10, 10, 0);
        // std::vector<Eigen::Vector3d> poly1 = {p1, p2, p3, p4};
        // std::vector<Eigen::Vector3d> poly2 = {p4, p3, p1, p2};
        // Eigen::Vector3d p5 = Eigen::Vector3d(0, 0, 0);
        // Eigen::Vector3d p6 = Eigen::Vector3d(0, 0, 10);
        // Eigen::Vector3d p7 = Eigen::Vector3d(0, 10, 0);
        // Eigen::Vector3d p8 = Eigen::Vector3d(10, 10, 0);
        // std::vector<Eigen::Vector3d> poly3 = {p5, p6, p7, p8};


        // TSPlane pln1 = TSPlane(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0.5, 0.5, 0));

        // TSPolygon poly1T = TSPolygon(poly1, pln1);
        // TSPolygon poly2T = TSPolygon(poly1, pln1);

        // std::cout << "poly vertices nubmer 1: " << poly1T.getPoints().size() << std::endl;
        // std::cout << "poly vertices nubmer 2: " << poly2T.getPoints().size() << std::endl;
        // std::cout << "poly center 1: " << poly1T.getCenter().transpose() << std::endl;
        // std::cout << "poly center 2: " << poly2T.getCenter().transpose() << std::endl;

        // // bool isEquals = false;
        // // if (poly1T == poly2T)
        // //     isEquals = true;


        // // std::cout << "isEquals: " << isEquals << std::endl;

        // // return;

        // // intersect polygon
        // bool isIntersect = false;
        // TSSegment segTST;
        // isIntersect = poly1T.intersectPolygon(poly2T, segTST);

        // std::cout << "Is intersect: " << isIntersect << std::endl;


        // return;



        // // =============================================================================
        // intersect polygons and get segments connecting the intersection points
        // // =============================================================================

        std::vector<TSSegment> splitSegs;
        this->rIntersectPolygons(this->m_MergedPolygons, splitSegs);


        // TEST


        /////////////////////////////////////////
        // test multiple intersections
        /////////////////////////////////////////

        std::vector<TSPolygon> subSplitPolygonsT = {this->m_MergedPolygons[1]};
        std::vector<TSPolygon> tempSubContainerPolyT;  // <--- TODO: this probably must be set of unique elements
        std::tuple<TSPolygon, TSPolygon> subTuplePolys;
        bool isSubSplitVT;

        bool isFound0 = false;  // TEST
        bool isFoundA = false;  // TEST
        bool isFoundB = false;  // TEST

        bool isUnique = true;  // TEST


        uint NSubSplit = 0;

        // int counter = 0;  // DEBUG
        int whileCounter = 0;

        do
        {
            std::cout << "[DEBUG] while loop counter: " << ++whileCounter << std::endl;

            for (int i = 0; i < subSplitPolygonsT.size(); i++)
            {
                NSubSplit = 0;
                // counter = 0;  // DEBUG

                for (auto& seg : splitSegs)
                {
                    // std::cout << "[DEBUG] loop counter: " << ++counter << std::endl;

                    isSubSplitVT = subSplitPolygonsT[i].splitPolygon(seg, subTuplePolys);

                    // std::cout << "[DEBUG] isSubSplitVT: " << isSubSplitVT << std::endl;

                    if (isSubSplitVT)
                    {

                        NSubSplit++;

                        TSPolygon polySplitA = std::get<0>(subTuplePolys);
                        TSPolygon polySplitB = std::get<1>(subTuplePolys);

                        // if (polySplitA.isValid())
                        // {
                            isUnique = true;
                            for (auto& poly : tempSubContainerPolyT)
                            {
                                if (poly == polySplitA)
                                {
                                    isUnique = false;
                                    break;
                                }
                                    
                            }
                            if (isUnique)
                                tempSubContainerPolyT.push_back(polySplitA);
                        // }

                        // if (polySplitB.isValid())
                        // {
                            isUnique = true;
                            for (auto& poly : tempSubContainerPolyT)
                            {
                                if (poly == polySplitB)
                                {
                                    isUnique = false;
                                    break;
                                }
                            }
                            if (isUnique)
                                tempSubContainerPolyT.push_back(polySplitB);
                        // }

                    }
                }


                if (NSubSplit == 0)
                {
                    isUnique = true;
                    for (auto& poly : tempSubContainerPolyT)
                    {
                        if (poly == subSplitPolygonsT[i])
                        {
                            isUnique = false;
                            break;
                        }
                    }
                    if (isUnique)
                        tempSubContainerPolyT.push_back(subSplitPolygonsT[i]);
                }

                // std::cout << "----------------------------- loop: " << i << " / " << subSplitPolygonsT.size() << " ----------------------------" << std::endl;
                // std::cout << "[DEBUG] NSubSplit: " << NSubSplit << std::endl;
                // std::cout << "[DEBUG] subSplitPolygonsT size: " << subSplitPolygonsT.size() << std::endl;
                // std::cout << "[DEBUG] tempSubContainerPolyT size: " << tempSubContainerPolyT.size() << std::endl;
                // std::cout << "------------------------" << std::endl;
                // // for (auto& poly : tempSubContainerPolyT)
                // //     std::cout << "[DEBUG] centers: " << poly.getCenter().transpose() << std::endl;
                // std::cout << "---------------------------------------------------------------------" << std::endl;

                // if (whileCounter == 1 && i == 1) return;  // DEBUG
            }
            // if (whileCounter == 0) return;  // DEBUG
            
            // std::cout << "------------------------------- before ------------------------------" << std::endl;
            // std::cout << "[DEBUG] NSubSplit: " << NSubSplit << std::endl;
            // std::cout << "[DEBUG] subSplitPolygonsT size: " << subSplitPolygonsT.size() << std::endl;
            // std::cout << "[DEBUG] tempSubContainerPolyT size: " << tempSubContainerPolyT.size() << std::endl;
            // std::cout << "---------------------------------------------------------------------" << std::endl;

            subSplitPolygonsT.clear();

            if (tempSubContainerPolyT.size() > 0)
            {
                for (auto& tpoly : tempSubContainerPolyT)
                {
                    subSplitPolygonsT.emplace_back(tpoly);
                }
            }
            tempSubContainerPolyT.clear();

            // std::cout << "------------------------------- after -------------------------------" << std::endl;
            // std::cout << "[DEBUG] NSubSplit: " << NSubSplit << std::endl;
            // std::cout << "[DEBUG] subSplitPolygonsT size: " << subSplitPolygonsT.size() << std::endl;
            // std::cout << "[DEBUG] tempSubContainerPolyT size: " << tempSubContainerPolyT.size() << std::endl;
            // std::cout << "---------------------------------------------------------------------" << std::endl;

            // return;
            // whileCounter++;  // DEBUG

        } while (NSubSplit > 0);



        ////////////////////////////////
        // clean out the intersections
        // for each polygon, if it contains any other polyygon's center, remove it
        ////////////////////////////////

        // // TODO: to be implemented

        // std::cout << "----------------------------- loop: " << "x" << " / " << subSplitPolygonsT.size() << " ----------------------------" << std::endl;
        // std::cout << "[DEBUG] NSubSplit: " << NSubSplit << std::endl;
        // std::cout << "[DEBUG] subSplitPolygonsT size: " << subSplitPolygonsT.size() << std::endl;
        // std::cout << "[DEBUG] tempSubContainerPolyT size: " << tempSubContainerPolyT.size() << std::endl;
        // std::cout << "------------------------" << std::endl;
        // // for (auto& poly : tempSubContainerPolyT)
        // //     std::cout << "[DEBUG] centers: " << poly.getCenter().transpose() << std::endl;
        // std::cout << "---------------------------------------------------------------------" << std::endl;

        // std::vector<TSPolygon> cleanSubSplitPolygonsT;
        // bool isContained;

        // for (int i = 0; i < subSplitPolygonsT.size(); i++)
        // {
        //     isContained = false;
        //     for (int j = 0; j < subSplitPolygonsT.size(); j++)
        //     {

        //         // if (subSplitPolygonsT[i].getCenter().isApprox(subSplitPolygonsT[j].getCenter()))
        //         // {
                    
        //         //     isContained = true;
        //         //     break;
        //         // }



        //         // approach with point inside polygon
        //         if (subSplitPolygonsT[i].isPointInsidePolygon(subSplitPolygonsT[j].getCenter()))
        //         {
        //             // // erase the polygon if true
        //             subSplitPolygonsT.erase(subSplitPolygonsT.begin() + i);
        //             // i--;


        //             // std::cout << "POOOOP" << std::endl;
        //             isContained = true;
        //             break;
        //         }
        //     }
        //     if (!isContained)
        //         cleanSubSplitPolygonsT.push_back(subSplitPolygonsT[i]);
        // }

        // subSplitPolygonsT.clear();
        // subSplitPolygonsT = cleanSubSplitPolygonsT;




        std::cout << "[DEBUG] Number of sub split polygons: " << subSplitPolygonsT.size() << std::endl;  // DEBUG
        std::cout << "[DEBUG] N split segments: " << splitSegs.size() << std::endl;




        // return;





#ifdef TSLAM_REC_DEBUG
    // Debug visualizer
    open3d::visualization::Visualizer* vis(new open3d::visualization::Visualizer());
    vis->CreateVisualizerWindow("TSPlaneTags", 1920, 1080);

    // draw base plane tags as wireframe
    for (auto& tag : this->m_Timber->getPlaneTags())
    {
        open3d::geometry::TriangleMesh tagBase = tag.getOpen3dMesh();
        auto planeTagsLineset1 = open3d::geometry::LineSet::CreateFromTriangleMesh(tagBase);
        if (tag.isEdge())
            tag.setColor(Eigen::Vector3d(0, 1, 0));
        else
            tag.setColor(Eigen::Vector3d(1, 0, 0));
        planeTagsLineset1->PaintUniformColor(tag.getColor());
        vis->AddGeometry(planeTagsLineset1);
    }

    // draw tag centers as point cloud with normals
    this->m_Timber->getTagsCtrs().PaintUniformColor(Eigen::Vector3d(0, 0, 0));
    vis->AddGeometry(std::make_shared<open3d::geometry::PointCloud>(this->m_Timber->getTagsCtrs()));

    // draw AABB
    std::shared_ptr<open3d::geometry::LineSet> aabbLineset = std::make_shared<open3d::geometry::LineSet>();

    // draw polygon segments3D 
    // for (auto& pg : this->m_PlnAABBPolygons)
    // {
    //     std::vector<Eigen::Vector3d> pts = pg.getPoints();
    //     for (int i = 0; i < pts.size(); i++)
    //     {
    //         std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //         std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //         segLineset->points_.push_back(segm->Origin());
    //         segLineset->points_.push_back(segm->EndPoint());
    //         segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->PaintUniformColor(Eigen::Vector3d(0.5, 1, 0.5));
    //         vis->AddGeometry(segLineset);
    //     }
    // }

    // // draw first intersection polygon centers
    // for (auto& fpoly : this->m_PlnAABBPolygons)
    // {
    //     std::shared_ptr<open3d::geometry::PointCloud> pntCldPCtr = std::make_shared<open3d::geometry::PointCloud>();
    //     pntCldPCtr->points_.push_back(fpoly.getCenter());
    //     pntCldPCtr->PaintUniformColor(Eigen::Vector3d(0, 1, 1));
    //     vis->AddGeometry(pntCldPCtr);
    // }

    // // draw new merged planes centers
    // std::shared_ptr<open3d::geometry::PointCloud> pntCldNewPlanes = std::make_shared<open3d::geometry::PointCloud>();
    // for (auto& pg : this->m_MergedPolygons)
    // {
    //     pntCldNewPlanes->points_.push_back(pg.getCenter());
    //     pntCldNewPlanes->PaintUniformColor(Eigen::Vector3d(0.7, 0.3, 0.9));
    //     vis->AddGeometry(pntCldNewPlanes);
    // }

    // // draw new interesected polygons
    // for (auto& pg : this->m_MergedPolygons)
    // {
    //     std::vector<Eigen::Vector3d> pts = pg.getPoints();
    //     for (int i = 0; i < pts.size(); i++)
    //     {
    //         std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //         std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //         segLineset->points_.push_back(segm->Origin());
    //         segLineset->points_.push_back(segm->EndPoint());
    //         segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->PaintUniformColor(Eigen::Vector3d(0., 0., 1.));
    //         vis->AddGeometry(segLineset);
    //     }
    // }

    // // draw splitting segments
    // for (auto& seg : splitSegs)
    // {
    //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //     segLineset->points_.push_back(seg.Origin());
    //     segLineset->points_.push_back(seg.EndPoint());
    //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->PaintUniformColor(Eigen::Vector3d(1., 0., 1.));
    //     vis->AddGeometry(segLineset);
    // }
        
    // // // show split polygons
    // for (auto& pg : splitPolygons)
    // {
    //     std::vector<Eigen::Vector3d> pts = pg.getPoints();
    //     for (int i = 0; i < pts.size(); i++)
    //     {
    //         std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //         std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //         segLineset->points_.push_back(segm->Origin());
    //         segLineset->points_.push_back(segm->EndPoint());
    //         segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //         segLineset->PaintUniformColor(Eigen::Vector3d(0., 0., 1.));
    //         vis->AddGeometry(segLineset);
    //     }
    // }

    // // show the unsplit polygon
    // // std::vector<Eigen::Vector3d> pts = splitPolygons[0].getPoints();
    // for (int i = 0; i < pts.size(); i++)
    // {
    //     std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //     segLineset->points_.push_back(segm->Origin());
    //     segLineset->points_.push_back(segm->EndPoint());
    //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->PaintUniformColor(Eigen::Vector3d(1., 0., 1.));
    //     vis->AddGeometry(segLineset);
    // }

    // // show just one polygon test result
    // std::vector<Eigen::Vector3d> ptsrr = subSplitPolygonsT[0].getPoints();
    // for (int i = 0; i < ptsrr.size(); i++)
    // {
    //     std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
    //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //     segLineset->points_.push_back(segm->Origin());
    //     segLineset->points_.push_back(segm->EndPoint());
    //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     // generate a random vector for color
    //     std::random_device rd;
    //     std::mt19937 gen(rd());
    //     std::uniform_real_distribution<> dis(0, 1);
    //     Eigen::Vector3d color = Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
    //     segLineset->PaintUniformColor(color);
    //     vis->AddGeometry(segLineset);
    //     std::cout << "POP" << std::endl;
    // }

    // // // draw split polygo by accessing segments
    // TSPolygon polyTESTSS = subSplitPolygonsT[2];
    // for (auto& segVT : polyTESTSS.getSegments())
    // {
    //     std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    //     segLineset->points_.push_back(segVT.Origin());
    //     segLineset->points_.push_back(segVT.EndPoint());
    //     segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    //     segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));

    //     std::random_device rd;
    //     std::mt19937 gen(rd());
    //     std::uniform_real_distribution<> dis(0, 1);
    //     Eigen::Vector3d color = Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
    //     segLineset->PaintUniformColor(color);
    //     vis->AddGeometry(segLineset);

    // }

    // // draw one segment 1
    // std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
    // segLineset->points_.push_back(segTV.Origin());
    // segLineset->points_.push_back(segTV.EndPoint());
    // segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
    // segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    // segLineset->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    // segLineset->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
    // vis->AddGeometry(segLineset);

    // // draw one segment 2
    // std::shared_ptr<open3d::geometry::LineSet> segLinesetC = std::make_shared<open3d::geometry::LineSet>();
    // segLinesetC->points_.push_back(segTVCOPY.Origin());
    // segLinesetC->points_.push_back(segTV.EndPoint());
    // segLinesetC->lines_.push_back(Eigen::Vector2i(0, 1));
    // segLinesetC->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    // segLinesetC->colors_.push_back(Eigen::Vector3d(0, 1, 0));
    // segLinesetC->PaintUniformColor(Eigen::Vector3d(0., 1., 0.));
    // vis->AddGeometry(segLinesetC);

    // // visualize segment extremes
    // std::shared_ptr<open3d::geometry::PointCloud> pcdVTP1 = std::make_shared<open3d::geometry::PointCloud>();
    // pcdVTP1->points_.push_back(segTVCOPY.P1);
    // pcdVTP1->colors_.push_back(Eigen::Vector3d(1., 0., 0.));
    // vis->AddGeometry(pcdVTP1);

    // std::shared_ptr<open3d::geometry::PointCloud> pcdVTP2 = std::make_shared<open3d::geometry::PointCloud>();
    // pcdVTP2->points_.push_back(segTVCOPY.P2);
    // pcdVTP2->colors_.push_back(Eigen::Vector3d(0., 0., 1.));
    // vis->AddGeometry(pcdVTP2);

    // draw all polygons in a vector with different colors
    std::shared_ptr<open3d::geometry::PointCloud> pcdPolyCentersT = std::make_shared<open3d::geometry::PointCloud>();
    for (auto& poly : subSplitPolygonsT)
    {
        pcdPolyCentersT->points_.push_back(poly.getCenter());

        std::vector<Eigen::Vector3d> pts = poly.getPoints();
        // random color
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 1);
        Eigen::Vector3d color = Eigen::Vector3d(dis(gen), dis(gen), dis(gen));
        for (int i = 0; i < pts.size(); i++)
        {
            std::shared_ptr<open3d::geometry::Segment3D> segm = std::make_shared<open3d::geometry::Segment3D>(pts[i], pts[(i+1)%pts.size()]);
            std::shared_ptr<open3d::geometry::LineSet> segLineset = std::make_shared<open3d::geometry::LineSet>();
            segLineset->points_.push_back(segm->Origin());
            segLineset->points_.push_back(segm->EndPoint());
            segLineset->lines_.push_back(Eigen::Vector2i(0, 1));
            segLineset->PaintUniformColor(color);
            vis->AddGeometry(segLineset);
        }
    }
    pcdPolyCentersT->PaintUniformColor(Eigen::Vector3d(1., 0., 0.));
    vis->AddGeometry(pcdPolyCentersT);




    vis->Run();
    vis->Close();
    vis->DestroyVisualizerWindow();
#endif
    }

    void TSGeometricSolver::rDetectCreasesTags()
    {
        open3d::geometry::KDTreeFlann kdtree;
        kdtree.SetGeometry(open3d::geometry::PointCloud(this->m_Timber->getTagsCtrs()));

        std::vector<int> indices;
        std::vector<double> distances;
        const int knn = 2;

        for (int i = 0; i < this->m_Timber->getPlaneTags().size(); i++)
        {
            kdtree.SearchKNN(this->m_Timber->getPlaneTags()[i].getCenter(), knn, indices, distances);

            double angle = this->rAngleBetweenVectors(this->m_Timber->getPlaneTags()[i].getNormal(), 
                                                      this->m_Timber->getPlaneTags()[indices[1]].getNormal());

            if (angle < this->m_CreaseAngleThreshold) 
            {
                this->m_Timber->getPlaneTags()[i].setType(TSRTagType::Face);
            }
            else
            {
                this->m_Timber->getPlaneTags()[i].setType(TSRTagType::Edge);
            }
        }
    }
    double TSGeometricSolver::rAngleBetweenVectors(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2)
    {
        double angle = std::acos(v1.dot(v2) / (v1.norm() * v2.norm()));
        return (angle * 180 / M_PI);
    }

    void TSGeometricSolver::rIntersectTagPlnAABB()
    {
        this->m_Timber->scaleAABB(this->m_AABBScaleFactor);

        this->timeStart("[PROFILER]: intersect planes with AABB");

        Eigen::Vector3d* outPtsPtr = new Eigen::Vector3d[3*6];
        std::vector<Eigen::Vector3d>* planeIntersections = new std::vector<Eigen::Vector3d>();
        for (auto& t : this->m_Timber->getPlaneTags())
        {
            // a. skip face tags
            if (t.isFace()) continue;
            
            // b. caculate the intersection points
            unsigned int outPtsCount;
            this->rPlane2AABBSegmentIntersect(t.getPlane(),
                                              this->m_Timber->getAABB().min_bound_, 
                                              this->m_Timber->getAABB().max_bound_,
                                              outPtsPtr,
                                              outPtsCount);
            // b. save the result into a polygon
            planeIntersections->reserve(outPtsCount);
            planeIntersections->clear();
            for (unsigned int i = 0; i < outPtsCount; i++)
            {
                planeIntersections->push_back(outPtsPtr[i]);
            }

            TSPolygon tempPoly = TSPolygon(*planeIntersections, t.getPlane());
            tempPoly.reorderClockwisePoints();
            this->m_PlnAABBPolygons.push_back(tempPoly);
        }
        delete outPtsPtr;
        delete planeIntersections;


        this->timeEnd();
    }
    bool TSGeometricSolver::rRay2PlaneIntersection(const Eigen::Vector3d &RayOrig,
                                                   const Eigen::Vector3d &RayDir,
                                                   const TSPlane &Plane,
                                                   float *OutT,
                                                   float *OutVD)
    {
        const Eigen::Vector3d PlaneNormal(Plane.A, Plane.B, Plane.C);

        const double Denominator = PlaneNormal.dot(RayDir);
        if (Denominator == 0.0f)  return false;  // ray is parallel to plane

        const double Numerator = Plane.D - PlaneNormal.dot(RayOrig);
        const double t = Numerator / Denominator;

        if (OutT) *OutT = t;
        if (OutVD) *OutVD = Denominator;

        return true;
    }
    void TSGeometricSolver::rPlane2AABBSegmentIntersect(const TSPlane &plane,
                                                        const Eigen::Vector3d &aabb_min, 
                                                        const Eigen::Vector3d &aabb_max,
                                                        Eigen::Vector3d* out_points,
                                                        unsigned &out_point_count)
    {
        out_point_count = 0;
        float vd, t;

        // Test edges along X axis, pointing right
        Eigen::Vector3d dir = Eigen::Vector3d(aabb_max[0] - aabb_min[0], 0.f, 0.f);
        Eigen::Vector3d orig = aabb_min;
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;

        // Test edges along Y axis, pointing up
        dir = Eigen::Vector3d(0.f, aabb_max[1] - aabb_min[1], 0.f);
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_max[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;

        // Test edges along Z axis, pointing forward
        dir = Eigen::Vector3d(0.f, 0.f, aabb_max[2] - aabb_min[2]);
        orig = Eigen::Vector3d(aabb_min[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_min[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_min[0], aabb_max[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;
        orig = Eigen::Vector3d(aabb_max[0], aabb_max[1], aabb_min[2]);
        if (this->rRay2PlaneIntersection(orig, dir, plane, &t, &vd) && t >= 0.f && t <= 1.f)
            out_points[out_point_count++] = orig + dir * t;

        // Test the 8 vertices
        orig = aabb_min;
        if (plane.A * orig.x() + plane.B * orig.y() + plane.C * orig.z() + plane.D == 0.f)
            out_points[out_point_count++] = orig;
    }

    void TSGeometricSolver::rIntersectMeanPolygonPlnAABB()
    {
        this->rMeanPolygonPlanes();

        Eigen::Vector3d* outPtsPtr2 = new Eigen::Vector3d[3*6];
        for (auto& mpoly : this->m_MergedPolygons)
        {
            // a. caculate the intersection points
            unsigned int outPtsCount2;
            this->rPlane2AABBSegmentIntersect(mpoly.getLinkedPlane(),
                                              this->m_Timber->getAABB().min_bound_, 
                                              this->m_Timber->getAABB().max_bound_,
                                              outPtsPtr2,
                                              outPtsCount2);

            // b. store the new itnersection points into the polygon
            for (unsigned int i = 0; i < outPtsCount2; i++)
            {
                mpoly.addPoint(outPtsPtr2[i]);
            }
            mpoly.reorderClockwisePoints();
        }
        delete outPtsPtr2;
    }
    void TSGeometricSolver::rMeanPolygonPlanes()
    {
        open3d::geometry::KDTreeFlann kdtree;

        std::shared_ptr<open3d::geometry::PointCloud> pntCldPCtr = std::make_shared<open3d::geometry::PointCloud>();
        for (auto& p : this->m_PlnAABBPolygons) pntCldPCtr->points_.push_back(p.getCenter());
        kdtree.SetGeometry(*pntCldPCtr);

        std::vector<int> indices;
        std::vector<int> mergedIndices;
        std::vector<double> distances;

        Eigen::Vector3d meanPt;
        Eigen::Vector3d meanNorm;

        for (int i=0; i < pntCldPCtr->points_.size(); i++)
        {
            kdtree.SearchRadius(pntCldPCtr->points_[i], this->m_MinPolyDist, indices,distances);

            int k = 0;
            meanPt = {0,0,0};
            meanNorm = {0,0,0};
            
            for (int j=0; j < indices.size(); j++)
            {
                // check if the index has already been merged
                if (std::find(mergedIndices.begin(), mergedIndices.end(), indices[j]) != mergedIndices.end())
                {
                    // if it has, check the next one
                    continue;
                }
                else
                {
                    // if it hasn't,
                    // mean the center of the polygons
                    meanPt += pntCldPCtr->points_[indices[j]];

                    // get the TSPlanes of the polygons and mean their normals
                    meanNorm += this->m_PlnAABBPolygons[indices[j]].getNormal();

                    // add the index to the merged indices
                    mergedIndices.push_back(indices[j]);
                    k++;
                }
            }

            if (k > 1)
            {
                meanPt /= k;
                meanNorm /= k;
                TSPolygon newPoly;
                newPoly.setLinkedPlane(TSPlane(meanNorm, meanPt));
                this->m_MergedPolygons.push_back(newPoly);
            }
        }
    }

    void TSGeometricSolver::rIntersectPolygons(std::vector<TSPolygon> &polygons,
                                               std::vector<TSSegment> &segments)
    {
        for (auto& polyA : this->m_MergedPolygons)
        {
            for (auto& polyB : this->m_MergedPolygons)
            {
                TSSegment seg;
                bool isIntersect = false;
                isIntersect = polyA.intersectPolygon(polyB, seg);

                // check the segments are unique
                for (auto& s : segments)
                {
                    if (s == seg)
                    {
                        isIntersect = false;
                        break;
                    }
                }
                if (isIntersect) segments.push_back(seg);
            }
        }
    }

    bool TSGeometricSolver::check4PlaneTags()
    {
        std::vector<tslam::TSRTag> tstag = this->m_Timber->getPlaneTags();
        if (tstag.size() != 0) return true;
        else
        {
            throw std::runtime_error("[ERROR]: corners are not set.");
        }
        return false;
    }
}