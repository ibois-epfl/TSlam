/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied Andrea Settimi and Hong-Bin Yang.
*/
#include "reconstruction/tslam_reconstructor.hh"

#include <iostream>

// build a console app that takes as first argument the path to the yml file of the map and as a second the output directory of the mesh and as a third its name
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
    }
    std::cout << "[INFO]: mesh saved in " << argv[2] << "/" << argv[3] << ".ply" << std::endl;
    return 0;
}