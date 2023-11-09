/**
This file is part of TSLAM and distributed under the terms of the GPLv3.

The modified version: Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)
The original project: Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas, MODIFIED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
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
or implied, of Rafael Muñoz Salinas.
*/
#include "io_utils.h"
namespace tslam{
void   toStream__ ( const  cv::Mat &m,std::ostream &str ) {

    int r=0,c=0,t=0;
    if (!m.empty()){
        r=m.rows;
        c=m.cols;
        t=m.type();
    }

    str.write ( ( char* ) &r,sizeof ( int ) );
    str.write ( ( char* ) &c,sizeof ( int ) );
    str.write ( ( char* ) &t,sizeof ( int ) );

    //write data row by row
    for ( int y=0; y<m.rows; y++ )
        str.write ( m.ptr<char> ( y ),m.cols *m.elemSize() );
}
/**
 */

void  fromStream__ ( cv::Mat &m,std::istream &str ) {
    int r,c,t;
    str.read ( ( char* ) &r,sizeof ( int ) );
    str.read ( ( char* ) &c,sizeof ( int ) );
    str.read ( ( char* ) &t,sizeof ( int ) );
    if (r*c>0){
        m.create ( r,c,t );
        for ( int y=0; y<m.rows; y++ )
            str.read ( m.ptr<char> ( y ),m.cols *m.elemSize() );
    }
    else m=cv::Mat();
}

void   toStream__ ( const  std::string &m,std::ostream &str ) {
    uint32_t s=m.size();
    str.write((char*)&s,sizeof(s));
    str.write(&m[0],m.size());
}

void  fromStream__ ( std::string &m,std::istream &str ) {
    uint32_t s;
    str.read((char*)&s,sizeof(s));
    m.resize(s);
    str.read(&m[0],m.size());
}

}
