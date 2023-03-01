#ifndef cv__projectPoint_H
#define cv__projectPoint_H

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
//dervs rx,ry,rz,tx,ty,tx,fx,fy,cx,cy,k1,k2,p1,p2,k3,X,Y,Z
cv::Point2d __projectPoint( const  cv::Point3d &objectPoint,
                            const double *rvec /*3*/,const double *tvec/*3*/,const double fx,const double fy,const double cx,const double cy,const double *dist/*5*/, double *_DERVS_=0 /*2x18*/,const double *RMAT=0/*3x3 Rotation Matrix (cv::Rodrigues)*/,const double *RVECDER=0/* 3x9 Derivative of Rotation matrix (cv::Rodrigues)*/)
{


    int calc_derivatives=_DERVS_!=0;

    double   R[9]={1,0,0, 0,1,0, 0,0,1},
            dRdr[27],     k[14] = {dist[0],dist[1],dist[2],dist[3],dist[4],0,0,0,0,0,0,0,0,0} ;


    double zero_3[3]={0,0,0};
    cv::Matx33d matTilt = cv::Matx33d::eye();


    if(rvec==nullptr || tvec==nullptr){//no translation
        RMAT=R;
        rvec=zero_3;
        tvec=zero_3;
        for(int i=0;i<27;i++) dRdr[i]=0;
    }
    else{


        if(calc_derivatives ){
            if(  RVECDER==0 || RMAT==0  ){
                cv::Mat matR = cv::Mat( 3, 3, CV_64F, R ), _dRdr = cv::Mat( 3, 9, CV_64F, dRdr );
                cv::Mat __rvec(1,3,CV_64F,(double*)rvec);
                cv::Rodrigues(__rvec,matR,_dRdr);
            }
            else{
                memcpy(R,RMAT,9*sizeof (double));
                memcpy(dRdr,RVECDER,27*sizeof (double));
            }
        }
        else{
            if(RMAT==0){
                cv::Mat matR = cv::Mat( 3, 3, CV_64F, R );
                cv::Mat __rvec(1,3,CV_64F,(double*)rvec);
                cv::Rodrigues(__rvec,matR);
            }
            else{
                memcpy(R,RMAT,9*sizeof (double));
            }

        }


    }

    cv::Point2d imagePoint;
    double X = objectPoint .x, Y = objectPoint .y, Z = objectPoint.z;
    double x = R[0]*X + R[1]*Y + R[2]*Z + tvec[0];
    double y = R[3]*X + R[4]*Y + R[5]*Z + tvec[1];
    double z = R[6]*X + R[7]*Y + R[8]*Z + tvec[2];
    double r2, r4, r6, a1, a2, a3, cdist, icdist2;
    double xd, yd, xd0, yd0, invProj;
    cv::Vec3d vecTilt;
    cv::Vec3d dVecTilt;
    cv::Matx22d dMatTilt;
    cv::Vec2d dXdYd;

    double z0 = z;
    z = z ? 1./z : 1;
    x *= z; y *= z;

    r2 = x*x + y*y;
    r4 = r2*r2;
    r6 = r4*r2;
    a1 = 2*x*y;
    a2 = r2 + 2*x*x;
    a3 = r2 + 2*y*y;
    cdist = 1 + k[0]*r2 + k[1]*r4 + k[4]*r6;
    icdist2 = 1./(1 + k[5]*r2 + k[6]*r4 + k[7]*r6);
    xd0 = x*cdist*icdist2 + k[2]*a1 + k[3]*a2 + k[8]*r2+k[9]*r4;
    yd0 = y*cdist*icdist2 + k[2]*a3 + k[3]*a1 + k[10]*r2+k[11]*r4;

    // additional distortion by projecting onto a tilt plane
    vecTilt = matTilt*cv::Vec3d(xd0, yd0, 1);
    invProj = vecTilt(2) ? 1./vecTilt(2) : 1;
    xd = invProj * vecTilt(0);
    yd = invProj * vecTilt(1);

    imagePoint.x = xd*fx + cx;
    imagePoint.y = yd*fy + cy;

    if( calc_derivatives )
    {
        //DEV FX,FY
        _DERVS_[6]=xd;_DERVS_[7]=0;
        _DERVS_[24]=0;_DERVS_[25]=yd;


        //DEV CX,CY
        _DERVS_[8]=1;_DERVS_[9]=0;
        _DERVS_[26]=0;_DERVS_[27]=1;


        //DERV K1,K2
        for (int row = 0; row < 2; ++row)
            for (int col = 0; col < 2; ++col)
                dMatTilt(row,col) = matTilt(row,col)*vecTilt(2)
                        - matTilt(2,col)*vecTilt(row);
        double invProjSquare = (invProj*invProj);
        dMatTilt *= invProjSquare;
        dXdYd = dMatTilt*cv::Vec2d(x*icdist2*r2, y*icdist2*r2);
        _DERVS_[10]=fx*dXdYd(0);
        _DERVS_[28]=fy*dXdYd(1);


        dXdYd = dMatTilt*cv::Vec2d(x*icdist2*r4, y*icdist2*r4);
        _DERVS_[11]=fx*dXdYd(0);
        _DERVS_[29]=fy*dXdYd(1);

        //DERV P1,P2
        dXdYd = dMatTilt*cv::Vec2d(a1, a3);
        _DERVS_[12]=fx*dXdYd(0);
        _DERVS_[30]=fy*dXdYd(1);

        dXdYd = dMatTilt*cv::Vec2d(a2, a1);
        _DERVS_[13]=fx*dXdYd(0);
        _DERVS_[31]=fy*dXdYd(1);

        //DERV K3
        dXdYd = dMatTilt*cv::Vec2d(x*icdist2*r6, y*icdist2*r6);
        _DERVS_[14]=fx*dXdYd(0);
        _DERVS_[32]=fy*dXdYd(1);

        //DERV T
        double dxdt[] = { z, 0, -x*z }, dydt[] = { 0, z, -y*z };
        for( int j = 0; j < 3; j++ )
        {
            double dr2dt = 2*x*dxdt[j] + 2*y*dydt[j];
            double dcdist_dt = k[0]*dr2dt + 2*k[1]*r2*dr2dt + 3*k[4]*r4*dr2dt;
            double dicdist2_dt = -icdist2*icdist2*(k[5]*dr2dt + 2*k[6]*r2*dr2dt + 3*k[7]*r4*dr2dt);
            double da1dt = 2*(x*dydt[j] + y*dxdt[j]);
            double dmxdt = (dxdt[j]*cdist*icdist2 + x*dcdist_dt*icdist2 + x*cdist*dicdist2_dt +
                            k[2]*da1dt + k[3]*(dr2dt + 4*x*dxdt[j]) + k[8]*dr2dt + 2*r2*k[9]*dr2dt);
            double dmydt = (dydt[j]*cdist*icdist2 + y*dcdist_dt*icdist2 + y*cdist*dicdist2_dt +
                            k[2]*(dr2dt + 4*y*dydt[j]) + k[3]*da1dt + k[10]*dr2dt + 2*r2*k[11]*dr2dt);
            dXdYd = dMatTilt*cv::Vec2d(dmxdt, dmydt);
            _DERVS_[3+j]= fx*dXdYd(0);
            _DERVS_[21+j]= fy*dXdYd(1);
        }
        //DERV R
        double dx0dr[] =
        {
            X*dRdr[0] + Y*dRdr[1] + Z*dRdr[2],
            X*dRdr[9] + Y*dRdr[10] + Z*dRdr[11],
            X*dRdr[18] + Y*dRdr[19] + Z*dRdr[20]
        };
        double dy0dr[] =
        {
            X*dRdr[3] + Y*dRdr[4] + Z*dRdr[5],
            X*dRdr[12] + Y*dRdr[13] + Z*dRdr[14],
            X*dRdr[21] + Y*dRdr[22] + Z*dRdr[23]
        };
        double dz0dr[] =
        {
            X*dRdr[6] + Y*dRdr[7] + Z*dRdr[8],
            X*dRdr[15] + Y*dRdr[16] + Z*dRdr[17],
            X*dRdr[24] + Y*dRdr[25] + Z*dRdr[26]
        };
        for( int j = 0; j < 3; j++ )
        {
            double dxdr = z*(dx0dr[j] - x*dz0dr[j]);
            double dydr = z*(dy0dr[j] - y*dz0dr[j]);
            double dr2dr = 2*x*dxdr + 2*y*dydr;
            double dcdist_dr = (k[0] + 2*k[1]*r2 + 3*k[4]*r4)*dr2dr;
            double dicdist2_dr = -icdist2*icdist2*(k[5] + 2*k[6]*r2 + 3*k[7]*r4)*dr2dr;
            double da1dr = 2*(x*dydr + y*dxdr);
            double dmxdr = (dxdr*cdist*icdist2 + x*dcdist_dr*icdist2 + x*cdist*dicdist2_dr +
                            k[2]*da1dr + k[3]*(dr2dr + 4*x*dxdr) + (k[8] + 2*r2*k[9])*dr2dr);
            double dmydr = (dydr*cdist*icdist2 + y*dcdist_dr*icdist2 + y*cdist*dicdist2_dr +
                            k[2]*(dr2dr + 4*y*dydr) + k[3]*da1dr + (k[10] + 2*r2*k[11])*dr2dr);
            dXdYd = dMatTilt*cv::Vec2d(dmxdr, dmydr);
            _DERVS_[j]= fx*dXdYd(0);
            _DERVS_[18+j]= fy*dXdYd(1);
        }

        //DERV XYZ
        {
        float k1=dist[0];
        float k2=dist[1];
        float p1=dist[2];
        float p2=dist[3];
        float k3=dist[4];
        double m[16]={R[0],R[1],R[2],tvec[0],
                      R[3],R[4],R[5],tvec[1],
                      R[6],R[7],R[8],tvec[2],
                      0,0,0,1};

        double x = R[0]*X + R[1]*Y + R[2]*Z + tvec[0];
        double y = R[3]*X + R[4]*Y + R[5]*Z + tvec[1];
        double z = R[6]*X + R[7]*Y + R[8]*Z + tvec[2];

        float x2=x*x;
        float y2=y*y;
        float z2=z*z;
        float A= (x2/z2) + (y2/z2);
        float R2=x2+y2;
        float U1=A * (2.*k2 + 3.* A*k3) ;
        float U2=(1 + A*(k1 + A *(k2 + A*k3)));
        float U3=A*(2.*k2 + 3.* A*k3);
        float U4=-2.* R2* (k1 + U1)*x;
        float U5=-2.* R2* (k1 + U1)*y;
        float yp1=y*p1;
        float xp1=x*p1;
        float xp2=x*p2;
        float yp2=y*p2;
        float y2p2=y2*p2;
        float xy=x*y;
        float z4=z2*z2;
        float z3=z*z2;

        float derUX= (fx/(z4))*
                (m[8]*U4 + 2.*((k1*m[0] - 3.* m[8]* p2 + m[0]*U1)*x2
                +(k1*m[4] + U3*m[4] - 2.*m[8]*p1)*xy - m[8]*y2p2)* z
                -(m[8]*U2*x - 2.*(m[4]*xp1 + 3*m[0]*xp2 + m[0]*yp1 + m[4]*yp2)) *z2 + m[0]*U2*z3);


        float derUY= (fx/(z4))*
                (m[9]*U4 + 2.*((k1*m[1] - 3.* m[9]* p2 + m[1]*U1)*x2 +
                (k1*m[5] + U3*m[5] - 2.*m[9]*p1)*xy - m[9]*y2p2)* z -
                (m[9]*U2*x - 2.*(m[5]*xp1 + 3*m[1]*xp2 + m[1]*yp1 + m[5]*yp2)) *z2 + m[1]*U2*z3);

        float derUZ= (fx/(z4))*
                (m[10]*U4 + 2.*((k1*m[2] - 3.* m[10]* p2 + m[2]*U1)*x2 +
                (k1*m[6] + U3*m[6] - 2.*m[10]*p1)*xy - m[10]*y2p2)* z -
                (m[10]*U2*x - 2.*(m[6]*xp1 + 3*m[2]*xp2 + m[2]*yp1 + m[6]*yp2)) *z2 + m[2]*U2*z3);

        float derVX= (fy/z4)*
                ( m[8] *U5+ 2 *((k1 + U1)*y*(m[0]*x + m[4]*y) - m[8] *(2.* xp2*y + p1*(x2 + 3.*y2) ) ) * z -
                (m[8]*U2*y - 2.* (m[0]*xp1 + m[4]*xp2 + 3.* m[4]* yp1 + m[0]*yp2))*z2 + m[4]*U2*z3);

        float derVY= (fy/z4)*
                ( m[9] *U5 + 2 *((k1 + U1)*y*(m[1]*x + m[5]*y) - m[9] *(2.* xp2*y + p1*(x2 + 3.*y2) ) ) * z -
                (m[9]*U2*y - 2.* (m[1]*xp1 + m[5]*xp2 + 3.* m[5]* yp1 + m[1]*yp2))*z2 + m[5]*U2*z3);

        float derVZ= (fy/z4)*
                (m[10]* U5 + 2 *((k1 + U1)*y*(m[2]*x + m[6]*y) - m[10] *(2.* xp2*y + p1*(x2 + 3.*y2) ) ) * z -
                (m[10]*U2*y - 2.* (m[2]*xp1 + m[6]*xp2 + 3.* m[6]* yp1 + m[2]*yp2))*z2 + m[6]*U2*z3);

        _DERVS_[15]=derUX;
        _DERVS_[16]=derUY;
        _DERVS_[17]=derUZ;
        _DERVS_[33]=derVX;
        _DERVS_[34]=derVY;
        _DERVS_[35]=derVZ;
        }




    }
    return imagePoint;
}
#endif
