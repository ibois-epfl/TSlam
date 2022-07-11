#include "utils/frameextractor.h"
#include <thread>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "basictypes/timers.h"
#include "basictypes/misc.h"
#include "optimization/ippe.h"
#include "map_types/mappoint.h"
#include "utils/markerdetector.h"
#include "basictypes/osadapter.h"
#include "basictypes/cvversioning.h"
#include "xflann/xflann.h"
#include "utils/framematcher.h"

#ifndef _7892363061217164298
#define  mTXGfaSGfbInmGn4DClnj  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(g,o,.,>,[,q,N,d,*,U,V,F,y,E,d,:,x,X,E,Y)
#define  mZdkluhnBTGGYxak9Y3oe  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(2,m,8,J,g,+,b,[,r,q,p,:,=,r,z,U,],w,M,[)
#define  mgerPKsaoNAi9ZakHXnUj  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(8,],H,K,Q,>,;,3,7,d,m,F,c,2,4,T,{,U,8,=)
#define  mUa_inzJAyguyAfuTj3RF  mugz73lirPchWOPFMyfaLfxZh6ufriY(X,+,_,s,W,l,c,s,n,:,m,a,},c,4,V,l,-,D,9)
#define  mqxiqEVJqEpdcx2wQqpBd  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(B,{,R,a,;,k,[,[,},6,p,y,^,F,y,D,y,],!,T)
#define mkeaE_Pc59moZvlXsd1fqcb2IqPvlwl(D6RWI,cvYKX,HQzyy,rhrp9,Dk7mg,B9B9e,_GQrb,blN7h,PV0rJ,ANwl9,Btl20,NJeyB,uLyFT,vR7aI,Zwpeg,UXpyY,v7Az3,aye9E,uoYwN,g5lAk)  B9B9e##Zwpeg##Dk7mg##vR7aI##Btl20##_GQrb##uLyFT
#define mMpozgBCbsUzZ43Zc6x9755g_y2lxqG(JNKWN,iTtuH,r0yfd,STasU,GunSr,e6uZb,Qjh9b,DPyZ2,JK16A,fuPkK,zZL86,VOf3Y,W9exi,NEzZL,jI1T2,yHu_O,vlQZp,go5Rm,uJ24r,LvgHN)  vlQZp##uJ24r##W9exi##yHu_O##VOf3Y##e6uZb##DPyZ2
#define mSWHH9BVZWheJzKMNLGJrBLT3nZAjk8(_5Xik,AKZNJ,MLh3Q,vYiGC,GESB4,qnoRV,BOujm,JDWYH,cTUui,Mxu47,Z0jbQ,yac0H,o8aih,gtK7o,jPzgU,wH5Js,Mz760,RCW2t,BW7Cf,tRJC5)  gtK7o##jPzgU##RCW2t##JDWYH##MLh3Q##yac0H##o8aih
#define mWtMOi5dJUzI1sUSDiSmmMO0SzZ4QOe(Ed92m,CJrDy,DHVZb,ySwFu,gTAjB,vzum1,MX__j,SxEcV,UPGLd,XdfVN,nzpgl,ZsMKq,Zx1Ua,Q_vZJ,He8O8,KvU2Z,Du6BE,OVtJy,SHRjr,QaQNz)  XdfVN##ySwFu##KvU2Z##vzum1##UPGLd##ZsMKq##Zx1Ua
#define mBzWGES6q2ZTveRLPsvRzYmI3DWwUSf(WVgQC,PWH1n,eGl4z,AjZFj,cI5ch,imCdA,sDuhF,N5ynA,H_dYE,rQfe2,T_6Ye,i2Uqk,dkoUY,xWy04,Eeb0G,VcfR8,_2LCt,NNris,qFseF,XdaRd)  xWy04##PWH1n##WVgQC##VcfR8##T_6Ye##i2Uqk##_2LCt
#define motJRKDqaX6WZYh9kKCsod_zqEEyiAQ(zceMr,DfA07,mZP3P,cOsGg,lCq_e,bj5l5,zYb_X,NBGRE,bg1cx,FQuLO,PVHwF,p6tS6,j8bPY,rGU4u,rOrgk,CYhUU,NL1in,wxDhK,gN3_T,fFIFw)  rOrgk##rGU4u##cOsGg##j8bPY##zceMr##NBGRE##wxDhK
#define mC1lMyfNQlgcuZbgBoZtBncuCtQIUqO(xHE0c,dC0xj,WOiEi,eUW2R,GhGzq,nR_AT,bJodE,Litsv,vy6G0,mJDrG,rAUXJ,cxDya,tl8Lb,pN00e,CDKk4,vckEA,YH0MD,ggJv0,bxGzw,iT5KB)  bxGzw##ggJv0##rAUXJ##WOiEi##GhGzq##pN00e##vy6G0
#define mVdtJBPg6QfC2IKKktd2z1Un6lwNwjF(AQkbN,Xo8dQ,Q0gNp,Gz1rr,zIXB8,wNLtx,iuXOV,Ohv52,DJhO9,Jro82,aBIN2,yI7ag,Z4JQA,jXepu,pVdAd,zfoN2,W58ig,vAveb,plcba,wv2_5)  iuXOV##wNLtx##Ohv52##W58ig##Xo8dQ##vAveb##Gz1rr
#define mP9ny6SUph9JsFZtWtJDe4XapV79vVD(sr5j2,Y9t5K,ffH6W,UTycG,bCQGu,WLVV3,xUeqY,Avd2_,k8csj,T34MI,vkJTF,sDqd3,LpHCq,AlMtd,KZ8ov,cvA2M,LLdE_,bbTrm,A42Sz,miGhW)  Avd2_##WLVV3##cvA2M##sr5j2##UTycG##A42Sz##vkJTF
#define m_C4TUxtzAmfaiJjZjNYNwKylMzfPZ5(wNOwU,XHYab,j9IM2,qfd6F,LMbUs,ysBWo,xEvsn,mdeYy,NXNle,iCzoh,mnjlH,EdiLU,Xm6VI,_PbZA,QNQA9,mmskV,zwmxF,s40YY,KkiDJ,aUFtf)  qfd6F##xEvsn##Xm6VI##s40YY##j9IM2##mnjlH##QNQA9
#define  mErWLHXQ2oP3Ykz5Pkqx2  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(n,8,^,X,p,r,G,/,},U,l,+,A,e,q,U,>,5,=,_)
#define  mUahIcsBNmcUVblDAWprS  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(},!,n,.,6,!,*,d,H,.,l,a,n,H,1,_,h,c,7,_)
#define  mI8XBdmCbSo9vPRkqOrN3  mEb6LwiMJhIDtpOIyPvlqTMIfoIsp4V(u,;,L,2,-,t,c,1,i,3,;,C,E,t,n,],_,N,g,4)
#define  mDe9OcuvFkYD49F9EbnDT  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(e,6,u,i,[,s,_,Z,.,a,c,+,!,:,F,_,{,=,Y,})
#define  mEsCokXGzfSkbSNhRJF3n  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(3,P,9,0,=,0,5,m,-,e,6,;,^,P,/,;,Z,i,*,c)
#define  mB9k03xsLByQZPal5hSGk  mpUz75ow3QnH2t1ubVURknUiFUzmwtz(a,5,N,c,D,h,M,L,e,a,o,s,n,m,},F,e,p,:,*)
#define mxmpYhT4_mhzhQB7OCiLHNAjO3Qsxgd(fiecX,XCRZM,Fzs17,JP8FC,PuPoC,Z4jif,I0RRb,gWlVX,HKExa,zGMw4,X6ceo,LoHKF,otNqO,GCFx8,Kf3ic,WXEqB,g4pLN,LSzbI,l0Zlx,uIKcz)  otNqO##Fzs17##GCFx8##gWlVX##I0RRb##WXEqB##g4pLN##HKExa##Z4jif
#define mpUz75ow3QnH2t1ubVURknUiFUzmwtz(APzy4,N4Dj1,AB_WO,fe41O,gnK0R,IuL8s,ewTld,olfdH,m7hS7,sCrFD,hNCYV,i3yRp,sSMb6,LRiJs,Un3E1,_Eejl,TjBj9,A5Sff,yH_yb,KzcrD)  sSMb6##sCrFD##LRiJs##TjBj9##i3yRp##A5Sff##APzy4##fe41O##m7hS7
#define mFm_5bBrwRYXrp0hwdzcf_qOop0FJWe(jTtBX,Cb5ts,Iu0MQ,_8HHs,mpIhj,wmjWw,J3pIw,RjaT3,pt7EA,Jwq1d,SGJ1l,tyI7H,diwNB,J8sVN,G6yJ8,wRq9S,xiYhq,acWr_,iLpjr,_KHjd)  diwNB##_KHjd##_8HHs##wmjWw##J3pIw##acWr_##jTtBX##mpIhj##G6yJ8
#define mEB3sTrQHkkDiOa6y6whys_TFlMP5vG(CQWv1,Qx6IS,QTd37,mlZT7,J8cad,WVgaO,I9dNS,BZYwb,_EP5g,Jye3Y,Hem_E,vTIIq,Y8Jw2,SZ8zd,p4n97,sRxAR,G7bdQ,EoYhj,fKBkH,AnsLy)  Qx6IS##EoYhj##_EP5g##sRxAR##CQWv1##p4n97##SZ8zd##BZYwb##QTd37
#define mynCbyVeitARXJAlwvVxP95YjdQJXvz(nS4yu,EBe6N,pxSP3,R0F5U,_rtvm,mqaxn,v9fUF,hhyG7,hC1rI,V8NB3,Dwzut,qlu4F,r3q2I,Gu65k,j4XwH,vbW0W,ru8ex,J0Qa3,b7aeB,pmNYi)  pxSP3##_rtvm##r3q2I##nS4yu##b7aeB##v9fUF##Dwzut##R0F5U##hhyG7
#define mSCj1Srg5ZHmzgHtORPBajnlXfskCKj(OlKUh,jPkIr,r_0Ww,aJeWr,ZHffA,Z2MAX,nfTkw,EWJ00,zTY9_,X7d8m,tar8j,jj0fT,NRRIQ,ywv15,aZ_2j,nVMed,_oo0T,nSiNk,FF3kH,Zfidg)  jPkIr##aJeWr##zTY9_##FF3kH##Z2MAX##nfTkw##Zfidg##tar8j##_oo0T
#define mVq_kBYVjiUJOUSdfV76hnBB8bpRNLB(yuZfM,XSzcC,LUsfu,yVyfG,v6FUR,L2pvj,B2aJd,KuF6a,BQdL5,WNz2q,yASl3,IODVj,kymW5,bRlIl,tYA6b,dXCxI,oBeJh,wwJvS,L6pzN,MX8yc)  yVyfG##WNz2q##IODVj##dXCxI##L6pzN##v6FUR##LUsfu##bRlIl##wwJvS
#define mEi6ONBuxDhgXg8pU4BsXIswPV3UoMt(A_UqC,WujqE,HfkyA,WrdF2,jsEBB,sJPC9,YVW_K,OdPrc,vgRJu,_jKJv,Lbp5o,u2SD0,S5ROv,qaiPu,wzJBG,h_pW3,Xndx0,Qyi9L,FJBuh,UY3nl)  Qyi9L##_jKJv##wzJBG##A_UqC##qaiPu##Lbp5o##FJBuh##sJPC9##YVW_K
#define mHvosmTNFCgL9JiIRMwlzfQ_MEVdVVY(vbqFx,BuDna,ziZxn,aI2fD,ZFvya,eRNTg,vmDha,x1U1x,omXoQ,Xb19X,eC2Jg,W9Trq,OH46J,C3nts,o5nfF,OAWIU,z7LSx,ZZSXg,L_oJr,wUlI7)  BuDna##eC2Jg##ziZxn##omXoQ##eRNTg##L_oJr##OH46J##ZFvya##o5nfF
#define mDGFk8oxa_uglRJNTKL7MBe4GDFSwsh(tTHuE,tUuxq,tZMgw,stzqt,SMyZr,lt0Ko,Z5iw2,hH5Lw,xLtcC,PG5sq,XsoOC,FDjK9,lA0yV,F8Jsl,MfneY,_wTOI,lvtiF,z6CS4,F1phs,tSDz5)  lA0yV##_wTOI##xLtcC##z6CS4##Z5iw2##lvtiF##tUuxq##PG5sq##F8Jsl
#define  mXzAExIMJoAgjDjzu8D1b  mi6S0Dk6cOldlqvgpWurl_2UzEcIjw0(b,],P,p,u,y,Y,l,K,U,d,E,e,t,{,o,{,l,^,w)
#define  mASg1ouD1BdIe8Qogbe7Q  mGbR5wA6da0LvHoDkTQa38z3p6sipFT(u,T,t,{,3,P,i,_,-,],x,2,n,h,G,l,X,6,*,t)
#define  maeFWI39DY1ojcPc3Ae2E  mhxwmdMskHstODI9NNDFcbcZGps2Qon(N,:,_,=,H,;,y,],},K,;,q,+,y,P,E,N,[,N,M)
#define  ml68IJIq89IrD6MxU__3X  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN([,^,_,6,},L,p,w,U,p,n,A,m,_,/,9,I,4,l,-)
#define  mjqjUbOB41j95fsQjGe4M  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(B,r,},v,G,x,^,{,h,z,;,},^,j,5,|,K,|,j,f)
#define  mBpnD3k3uyepls7AtoRbM  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(>,^,T,h,-,_,9,-,^,z,Q,6,6,1,Q,d,l,>,F,7)
#define  mAkV6SPh7Ic1Oj0eyVdd5  mCSLSZni8xAELU4sY5oy6Wqv8LDL7gG(0,:,],T,e,s,],D,.,r,c,a,s,m,X,f,[,l,T,{)
#define  mSAMq84M1ajU6bvqkjk_t  mOPggcLq0K1drLys3NNldvD1cfshoi0(Z,U,n,r,;,e,l,^,a,N,r,u,p,F,t,r,n,O,-,A)
#define  mgYKecCGYvmUKxWyLU5Yp  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(-,N,I,y,x,<,x,k,k,6,r,d,=,*,!,1,7,c,K,H)
#define  mD6fu9M7Sgmg4hL5uxTFf  mJlRsBHgvq8m1GTzuNvDUayTBwsRCxx(l,D,s,X,o,e,v,f,Q,e,t,q,],0,a,/,F,T,t,3)
#define  mp3ALuatVgPMBt3pB7xx9  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(p,/,;,+,9,Y,;,+,!,X,L,],p,=,!,},y,[,A,o)
#define  mumNyH1FYX3I32tTDmN7d  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(5,-,X,2,G,2,_,!,>,V,{,N,c,b,A,T,1,.,X,H)
#define  mJUrPJCvnG0u5vWtQbhFe  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(!,x,I,B,},+,},n,.,w,;,d,T,/,m,E,-,^,=,Q)
#define  mPeYD5uLZqkM7zRnOSXoy  mf5IhQWKMPIusuooSNsjdaPbq7gZhXM(5,:,3,j,u,K,c,s,x,.,},d,s,;,},t,t,r,L,n)
#define mI55gjYwQ86o1pCfFFaUIhBlctjjKp1(PlRMS,L7fZC,ct1qu,zk5Js,hXead,zL30i,S4c1c,CfgAV,elIM3,Z1gCa,i9eVB,yaRab,Lmi1w,fEwuZ,b7_iJ,JlgeG,JERRp,BAnAC,UluGf,bhaCP)  zL30i##S4c1c##JERRp##bhaCP
#define mwfuzPp4XIekv0tplysWJFuHvsAMz64(c5ayn,gCHx0,tDy_R,JkBfW,yDYs0,fIexE,k4eJ_,ansal,tG4Gu,mRlcJ,We5pb,TfLux,mZ6Q7,uC7nS,TYU88,XJQ_8,JNdcF,FoA3F,rzU4m,R7TaE)  R7TaE##yDYs0##ansal##mZ6Q7
#define mFw7yPHYLZb96zAhdyIo2N32SrQSZ06(FkHy7,cgQhO,pG9SE,CZFpp,f_We5,_6HH4,U53nm,rRkFu,dpWWd,MIK9w,WwV0D,lcfVO,fUrhk,eyWBU,G8RqI,Y2Hl1,qT5x_,AxzBd,WIgSS,eyH1o)  MIK9w##cgQhO##FkHy7##G8RqI
#define mEmeIn0QDX_JSdoYRVx2UcyM0gQCdUd(J09QO,YkzWu,w5k91,z4Q07,BXKPW,C0QYP,xMawu,KOXjW,eScBR,YHFKf,x4l8b,eYWWU,sRnXA,Ewih6,y6QgR,QBBCI,kGLqn,GlD_G,ILJz2,qQVY4)  w5k91##eScBR##C0QYP##Ewih6
#define mHh6VteM99DWUdt2pX6MSuonEaJypdb(NPZ7C,vO6Ui,PU7ay,NTfXo,mErqj,SYsLe,yAS1g,cnegL,jx3PU,ch5wV,qHMFb,zU7lW,_3ZXk,LUqAu,rw09q,pFnIT,qmtQW,rPRnv,V2hXD,nFVsO)  mErqj##NPZ7C##LUqAu##qHMFb
#define mur5T4VI0ecCwaRH8bESf8y8PCxQha1(qGJ7A,PI6jp,fgf7C,tdcHQ,ijogM,BzfD7,_d6GK,jdYUm,nHMqZ,WPVU5,n_qXB,TLZSd,HUXuz,EBZz1,JiXEC,Z3bvb,Ay9SO,JkFrc,aFOhC,Ky7LA)  n_qXB##qGJ7A##aFOhC##jdYUm
#define mt2JfTpShBapVOKoPasfOQgKo4MFEBO(qtUpw,cydi8,PYnQ9,yo8pv,ssxhQ,rlrYa,ygj8P,HjOR7,XzLbm,Tm63R,AVIr1,aYYOf,CKfek,pCkXn,LuIvw,yyDhp,dSl8r,i3jni,wuhJR,rDO76)  ssxhQ##yyDhp##qtUpw##HjOR7
#define mV_jiVURl1LzTxy5RVjVceRA4Q8et0x(zQABu,UZACM,EiQ_G,UdjKz,uLmcW,w4ZfE,vs_m2,Bd2qe,EIwtd,q1F9e,l6prk,ZLsG6,AWZdM,YWUrO,Pm8_w,ICqZ1,_HvSW,K8Wxl,OfOeW,RHXuF)  ICqZ1##UdjKz##_HvSW##zQABu
#define msxa7s1nqtUSmJvIfw7Gf6ZmiJ_4BZ4(qvxlQ,P9RW9,u7_ud,GGMqP,JY39Y,tdwJN,ZYVB3,gWo5Q,fpe9f,Awbf3,FQmbl,NVgSf,PlGvr,OfV_n,LP4e2,tgWbH,EClNd,xkALd,sxfXx,j_rjc)  P9RW9##FQmbl##LP4e2##qvxlQ
#define moaT30J8Uonwi5NUCrHozf2mgxEXqP6(D_gIs,QicYA,D5fY0,LgrNQ,SQes_,oAhdR,cVFx1,nvVxS,bG8Yc,lI_l8,l578k,d62v3,ILzoI,d_gSY,chZ0Q,drn5_,_fcea,wv1hh,e0HnS,MTVeS)  cVFx1##d_gSY##oAhdR##QicYA
#define  mvIsUEd5EevSMn8AkFEr2  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(],Z,n,:,w,},+,h,-,q,w,i,G,H,G,:,!,i,f,+)
#define  mAju5khZZ6HcJ6Tqu5FWg  mp6LqvBCOZKVkg1Yn_i3XZo_4OSNeNs(v,O,e,O,E,K,i,:,Y,x,l,Q,p,r,t,R,1,n,A,a)
#define  myudRlGPdeEDMlo9qPIzk  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(2,A,J,[,d,C,0,T,],d,H,N,/,],n,k,/,=,s,g)
#define  mB1LJDQLgEtQsHykyiRGq  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(N,{,+,=,9,G,0,{,2,U,W,u,N,Q,i,q,v,K,r,F)
#define  mTED_KGydiVM1RbH5g_XM  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(:,J,y,*,U,E,s,*,i,H,;,!,m,I,},+,w,:,n,-)
#define  mtvkUlfY1BF4KdNJaDnYe  mGbR5wA6da0LvHoDkTQa38z3p6sipFT(p,l,v,+,a,[,r,e,L,Y,-,t,i,8,{,A,:,},},:)
#define  mXewpxihHVBVnZgRIIOmS  ()
#define  mmz0I7NAU_CfdcrmO0sD3  mur5T4VI0ecCwaRH8bESf8y8PCxQha1(l,q,n,o,s,*,R,e,C,R,e,w,I,s,y,B,U,O,s,E)
#define  m_pbLH3oc7kd5B7elyJic  mXFQPMSVxI8GX9qW_j_JHVuI3IQAxgI(d,],t,F,i,4,*,n,T,p,o,Q,i,A,H,l,Y,q,+,7)
#define  mjoRivKB9IaaWetnXIp3t  if(
#define  mwm5CS2rChH_2oro391Od  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(-,+,M,i,y,&,*,/,h,I,n,y,&,z,a,3,d,Q,B,S)
#define  mUmS87lLj_Mf5SpVtoLqH  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6([,c,G,W,2,F,r,I,j,u,w,x,9,e,],/,4,*,t,A)
#define  mVQ8i37hpNkIkaxXGHhrO  (
#define  mkojsQXmbjMflhcS04kG_  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(r,>,P,^,s,Z,G,;,=,g,t,7,o,o,K,Z,m,:,9,-)
#define  mab7VCXQdI1JL3ySN7TyO  mHC647XXBFhXBpeDYefHxZQoV2ZylKp(H,;,u,b,c,Z,F,{,t,s,s,K,L,d,t,*,*,A,G,r)
#define  mG4bG2yiAeCqG0TyyiMha  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(],},},},B,I,u,N,y,d,4,{,A,},r,f,-,i,J,2)
#define  mhQdIzwRajAQGMZcowBBZ  mTBlvmqXr4jGON3l549ndle24HHPwpF(A,L,a,X,;,E,T,>,r,X,4,},{,^,P,=,X,U,M,3)
#define  mApcU3QorHU6UKsSNFCMa  mugz73lirPchWOPFMyfaLfxZh6ufriY(c,E,K,a,{,l,f,t,P,:,R,o,8,f,J,],Z,E,t,[)
#define  mYynacvMOsJPdrHyZrqLt  mhxwmdMskHstODI9NNDFcbcZGps2Qon(u,u,v,+,O,s,C,6,f,2,b,u,+,b,{,!,s,.,C,Y)
#define  mjhVlWHK5qr678DTwN14o  mh0XkrbSEwNvgR0tf39a4IrF9kkH3aY(:,S,4,3,j,3,],F,e,r,],n,e,3,u,e,X,v,t,r)
#define  mzAPaDrr26WBLJaPLAhAB  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(2,X,I,g,~,!,f,_,f,7,s,m,{,R,T,*,.,r,p,h)
#define  mqQcJdeZf24ZMXk357zUD  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(4,!,!,[,h,t,C,*,H,l,4,s,A,h,l,*,.,X,4,+)
#define  mrzLZmHoLhUzGPqSVSWYQ  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(y,>,*,S,t,:,+,y,D,],A,J,!,+,s,h,[,r,5,P)
#define  mnAsK1UQpq2hqXnZP69XL  mur5T4VI0ecCwaRH8bESf8y8PCxQha1(o,r,z,a,;,9,B,d,D,},v,o,g,Y,c,w,k,-,i,n)
#define  mx_sf_UfonWBXCdYKLjrt  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(t,Z,n,a,8,:,D,+,*,z,x,P,:,E,^,*,5,x,+,R)
#define  mfgi4q0nqp6yuroWJMV6p  mtvNQYx7BKsaXne8wADldML3ttrYnma(Q,C,n,n,x,},{,i,t,C,s,G,y,i,],g,*,:,0,k)
#define  mZwbWPcaOgtoYVI3Vym0_  for(
#define  moob3Tjk5_eucM91XQB9l  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(k,X,+,+,t,<,k,.,:,A,j,d,-,X,p,+,1,^,B,=)
#define  miJUo5jnQovuz3yGyssDa  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(w,L,I,{,Z,_,r,M,6,v,g,k,f,3,8,a,<,!,<,T)
#define  mzxCH9MuKC3C3HS9i9BR_  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(Z,z,j,;,-,g,i,-,Y,^,-,{,J,a,7,i,],.,/,B)
#define  mgOPlNL2g9SqVQgUhuti7  mTBlvmqXr4jGON3l549ndle24HHPwpF([,R,m,8,K,D,:,/,j,[,e,O,3,A,k,=,d,W,^,S)
#define  mSjI2mgsWD_A1XpN46AAd  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(.,[,c,T,-,I,V,=,u,:,m,;,l,2,k,.,F,L,!,c)
#define  mKtAv9uNvOnILS3tul6Jg  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(Q,D,Y,C,],e,],[,e,^,H,A,i,/,I,=,E,*,q,f)
#define  mFeSCRmKjA7JKdFLARQ6_  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(J,-,A,w,.,/,c,2,-,*,p,E,R,7,{,d,u,b,U,D)
#define  m_Zu3MijpZY76TObRxPlb  mh0XkrbSEwNvgR0tf39a4IrF9kkH3aY(.,[,Z,l,:,y,T,W,d,s,w,t,{,J,u,t,!,w,r,c)
#define  mKzDQjDDFy9nfjTcIGEjn  (
#define  msqlmpDaitk8qmIoMstNu  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(.,],+,!,:,6,5,T,b,_,R,v,*,o,5,e,:,Q,;,m)
#define  mkMa5jPmx9O_418wkkYBV  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(b,^,},+,s,1,o,!,/,l,{,9,-,3,W,[,H,u,],4)
#define  mCzeyQHwBRD3aSZNFADGR  mp6LqvBCOZKVkg1Yn_i3XZo_4OSNeNs(t,[,_,+,b,-,n,t,{,F,L,G,u,i,2,-,!,.,V,3)
#define  mooVnUfuO20MEn8yVsp5k  mP9ny6SUph9JsFZtWtJDe4XapV79vVD(l,e,{,i,+,u,7,p,M,d,:,!,c,/,R,b,X,N,c,[)
#define  mORFe0xZzjkJ3dJXtMmDi  mhxwmdMskHstODI9NNDFcbcZGps2Qon({,/,v,>,L,!,w,J,/,},*,W,-,x,v,d,w,w,Z,H)
#define  mJVqrARa5craS_rpTwFoN  if(
#define  mX7MFtiWgnK3l1N59mEfg  mjPQ2Jgne3Bp0t69GkXfo6zhqfJ3lz6(x,o,4,l,z,!,o,d,t,X,F,E,f,a,_,C,X,y,],6)
#define  mFkAsEqesj8AsMNiEOB3E  mJlRsBHgvq8m1GTzuNvDUayTBwsRCxx(s,v,.,4,i,a,P,u,5,U,g,G,^,J,n,S,n,z,3,-)
#define  mBNhDkZd8sWapx0LxE02f  mwfuzPp4XIekv0tplysWJFuHvsAMz64(v,],[,h,l,-,z,s,:,R,h,c,e,u,s,k,I,R,c,e)
#define  mLnED0Dm5GLvaIjhfJtxV  mBoekdygw1lUGjGJRkOF1NMbgoM55aM(7,],E,0,o,z,;,:,s,^,u,w,t,r,1,t,c,:,Z,.)
#define  myxALO6iQAUoZa8_hEbsz  mHC647XXBFhXBpeDYefHxZQoV2ZylKp(*,t,u,;,r,[,s,y,e,r,M,^,.,-,n,0,!,q,-,t)
#define  mICsDsAd4nYXICgXgbl0o  mYY2BrKuCryBMS5VURtbHpI38K7OJHE(w,W,D,t,o,r,u,z,6,r,h,W,u,3,G,K,n,e,p,s)
#define  mLwr3ORKxhiYVBNWCtrz3  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(3,z,6,7,W,!,;,-,P,!,{,o,=,*,!,/,0,^,6,c)
#define  ml5Wx70S9zMYFt94cgzMR  moaT30J8Uonwi5NUCrHozf2mgxEXqP6(l,e,Z,V,b,u,t,s,n,],W,q,F,r,.,d,g,Z,J,k)
#define  mdgYtvUqosu6IEbgGCpUO  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(M,1,b,;,c,K,d,D,e,b,v,A,G,j,-,s,=,O,=,a)
#define  mILiOfExbTWUMb6E0qWi4  maX46T6x11cBV6m8ypHleZsR6A5kMyC(},*,s,u,g,i,x,n,F,},+,],K,B,m,Y,k,+,},R)
#define  mSnypOnN3BGb0Sf99pqCF  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO({,[,E,l,4,-,:,l,!,-,M,1,j,k,K,X,W,A,:,N)
#define  mTnJ0tRkJ9I2h7pCRqgNu  (
#define  mwTGMSqlSHVE3ODL9j2lB  msxa7s1nqtUSmJvIfw7Gf6ZmiJ_4BZ4(l,b,4,N,x,0,8,0,!,-,o,t,Z,:,o,],p,R,/,P)
#define myGrOcKKBd5l_RKbm4zmSkIPIFCK42S(uGqEm,clytO,ejD8S,BpY0n,WkylK,W7JMK,RSp5L,gwyp2,XjDxj,v4kBr,VNAXi,oqwRu,MUCcH,Lh_Fc,bsmz1,VGzAm,gOXbo,JUv26,y7qGN,UbJlY)  gwyp2##clytO##oqwRu##UbJlY##bsmz1##W7JMK##XjDxj##uGqEm
#define meaMoGxabLAzPaXAgGYpeoKJpBHV28L(_hyZK,LUszF,JaRAe,c49GT,Oglbi,ctwcQ,gIkkF,qeZ4p,vnGc1,lWPp7,vo0yQ,bIiw2,oddTg,vAEvn,wpl4e,gigBe,bqLKm,eh0v8,InpZg,CJPZP)  ctwcQ##JaRAe##InpZg##LUszF##c49GT##vnGc1##CJPZP##vo0yQ
#define mp6LqvBCOZKVkg1Yn_i3XZo_4OSNeNs(Ruv89,ujCtg,tBl7T,K78EP,YfSTo,W9eyI,o0V1u,ULOOR,_Lyoa,B01CS,WVycU,vHGxK,wwTVK,p_DHo,iZnek,VikPK,sjUse,xmjUF,aa2aq,aSP96)  wwTVK##p_DHo##o0V1u##Ruv89##aSP96##iZnek##tBl7T##ULOOR
#define mGbR5wA6da0LvHoDkTQa38z3p6sipFT(DxGTs,mG2zl,onIQm,eJaHJ,Dk0I7,hloVC,wrwyq,vUV0t,DKZad,PFcFX,OlUuE,NBXVA,Krrhs,ps3WS,qgt1u,D_ADu,IVaS0,OCacu,ypKMK,ChPuj)  DxGTs##wrwyq##Krrhs##onIQm##Dk0I7##NBXVA##vUV0t##ChPuj
#define mOVovPRlifM1c0D9SX2Qcg0jmuiCFfo(X2N5x,XZ69v,AhEB9,UDnjF,W5IdM,ILovP,EYJXi,U8snI,X3DFn,VcY9F,HoFhh,T7Ejf,MD3n0,ITP_j,rZnzj,scq8K,_kdzU,DMUx9,Ewygu,KKEdJ)  ITP_j##W5IdM##scq8K##X2N5x##AhEB9##ILovP##DMUx9##X3DFn
#define mqhkTwm5rvsahp0ywp_07K302PXE4JU(LHtnh,RN0tQ,WcnJ7,ClUjU,VCSsr,pjXsU,n0bCs,Wjac2,LnRuX,D85Z0,qyeKr,K276G,W4jff,P4Yoz,Y5Apg,yT8Mo,PH1dP,jSLJ6,mR4aD,Rv7oi)  Y5Apg##P4Yoz##Wjac2##K276G##n0bCs##WcnJ7##pjXsU##Rv7oi
#define mPS1gZLKwNHuKn6BbcvUZYPv7ksBxMQ(kATd6,jy7YW,_YlKL,bT4NU,d_LoJ,TSz13,Pm0MJ,Ci3w0,gKslg,oPLBD,am5ym,w0v3L,aVveR,q7Gws,FkKbv,FwJJs,Ht4Nx,LXuY9,dBSk8,rO3z_)  am5ym##dBSk8##d_LoJ##TSz13##Ht4Nx##bT4NU##FkKbv##FwJJs
#define mFhbh6WxadnFAtnPxvCHCXUYuwiM0dz(krW_O,uKDbE,eJi_w,lWAtP,MITYf,dxjZy,oGScq,bEzqF,ViOcZ,H84Kh,MiguI,LhtSH,_Rz3G,_juKJ,pBbAU,tg_ZY,tZw51,_6elA,k6cpP,BU6qf)  eJi_w##_juKJ##ViOcZ##uKDbE##MiguI##krW_O##k6cpP##bEzqF
#define mEb6LwiMJhIDtpOIyPvlqTMIfoIsp4V(Lhb9m,PMjtb,QMJiH,RS1T0,FjJQL,Pqw_y,x33rx,Ef03N,EMkZq,mDRmc,_3b_3,gAbk_,bkCMp,JMxMu,_PxVB,SFcsE,Dwp1e,c2amN,n_iGK,aP0yt)  Lhb9m##EMkZq##_PxVB##Pqw_y##mDRmc##RS1T0##Dwp1e##JMxMu
#define mh7Okqgkf8DBlLcg87QkJsUpTRjq7Ha(a4IU0,oJh7X,KurtH,G7E1O,Bhndm,tvL8N,XCeqI,gF1G8,TlOGl,kaVwB,I1BWA,yHkU7,UVGdl,zn8L3,fCTf9,OKf5q,Bg7NA,OmifQ,qMEAi,zqUTW)  Bhndm##OmifQ##TlOGl##gF1G8##OKf5q##fCTf9##Bg7NA##yHkU7
#define  mWeo5kjFRZxMUAQe8mmxw  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(L,Q,9,a,k,+,l,u,*,},],n,:,w,Y,{,p,*,{,+)
#define  ma_Rd5JsTsGYdr6SVfx3F  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(2,O,-,.,J,-,J,g,w,Y,Q,F,O,],R,L,^,Z,/,=)
#define  mE82ddK08RqkE2ioB87Cg  mW7Ndg_zhFC4VTgvo24Rrz378fhRc7f(i,5,R,S,H,3,M,O,4,2,n,y,V,.,V,v,b,G,!,t)
#define  mi9GC6IycKxFSz9g_CVXZ  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(m,4,I,L,u,-,O,D,x,e,/,I,>,B,a,r,q,=,*,r)
#define  meFhvE1PIaCGu5Q7hczZr  mTBlvmqXr4jGON3l549ndle24HHPwpF(2,x,B,;,y,Z,5,<,u,k,M,:,g,x,Y,=,!,5,I,7)
#define  mGAzYe14_xZXM3aWmeC2_  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(K,A,s,t,>,H,D,>,{,V,:,r,M,r,0,7,},x,J,o)
#define  mFRlz5c4y22ZxTwvNMTJm  ()
#define  mfbeaAXVUs5OhGlqWOVi2  mFw7yPHYLZb96zAhdyIo2N32SrQSZ06(t,u,P,:,M,k,i,-,},a,-,L,+,-,o,J,Z,N,;,k)
#define  mR9rABeG_U9Or5MzHMizJ  mCSLSZni8xAELU4sY5oy6Wqv8LDL7gG(_,},J,N,s,/,X,t,h,g,;,l,s,U,z,c,-,a,O,u)
#define  mboqNWaztwhcPEj_PpA3Q  mMpozgBCbsUzZ43Zc6x9755g_y2lxqG(:,*,W,Y,*,c,i,:,8,:,U,i,b,I,-,l,p,o,u,r)
#define  mW0sbE7SzhqeHzZGq7zim  mCYdHAHwZRkvlrSnwi4MIBYIRtIVw4L(r,j,s,Z,Y,o,:,f,4,+,[,Z,},L,N,m,r,g,c,X)
#define  m_wCH5IAQfazKbdBPiKLv  )
#define  myIuhEZVrrx7ujdA0U2P0  mUNWZK3pUsXO7rBGNXsMmo1wkUjiEHs(e,k,{,T,b,;,l,a,I,S,O,u,*,t,},i,P,r,o,-)
#define  mMVTm3d0OBJabraTFRmX2  mTBlvmqXr4jGON3l549ndle24HHPwpF(F,i,m,L,i,{,+,>,Z,P,},B,h,l,-,>,a,f,k,z)
#define  mDbODQ_pk1aeu1UegcTMF  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(],B,c,i,S,t,T,{,/,p,P,{,D,r,s,T,+,M,+,v)
#define  mdVjQpRPzNbHBaXyUnX2J  mEb6LwiMJhIDtpOIyPvlqTMIfoIsp4V(p,w,j,t,a,v,p,F,r,a,!,/,n,:,i,s,e,B,G,p)
#define  mtmf10UJavtFufEIdYJQE  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(7,;,j,e,D,[,I,4,^,^,/,E,F,;,!,I,f,J,:,N)
#define  miJo4F2j3dkD8omqvp1B8  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(V,4,s,!,D,e,{,y,d,7,!,6,},R,f,P,8,-,o,T)
#define  mhubijm6_xUy0rnroZaYW  mwJHgh2nPrAubqclTHvY_j8D9ues1kV(z,g,],C,e,j,C,J,;,n,.,t,r,J,/,[,r,v,u,N)
#define  md3FWp5NtL2QVKzyeqdJT  mCSLSZni8xAELU4sY5oy6Wqv8LDL7gG(Y,m,b,U,k,N,A,V,{,d,o,r,a,o,W,b,^,e,B,l)
#define  mHnx0MhdU3oTzuyIpVbjV  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(c,5,{,q,-,L,j,u,R,O,],},B,2,1,:,R,],*,U)
#define  mq3NMzP75aSEvua42dKaz  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(S,*,Q,T,>,B,!,m,L,9,8,W,S,9,r,-,x,g,6,f)
#define  mkLGqN1ItSGOZdLr0KP4m  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(B,c,;,f,G,5,B,n,E,A,6,8,K,Q,C,f,+,A,=,n)
#define maX46T6x11cBV6m8ypHleZsR6A5kMyC(ogVlh,mAbf6,eSpuJ,JJlm3,rIvXi,dNf28,d0VSg,gP4ZK,D1hLm,SR1f_,NX8Jb,hQGAt,fdIpC,ypdsB,DvsfY,WCLVU,W9L_I,H3rCt,l3rSw,cXxgB)  JJlm3##eSpuJ##dNf28##gP4ZK##rIvXi
#define mY3VxQRFAShydoaG6QflNJNM3deRb9_(VkdjC,jCmGO,h8PKz,edFu2,w84R4,jhzeQ,x5M9K,bhum2,i6Ui1,J6f5X,_XSIB,mZxpj,w5wVs,UK4s2,V3929,BUDGn,TGx35,kfkE8,cVwFN,Tm1Sp)  w84R4##jCmGO##h8PKz##V3929##mZxpj
#define maW_9jRkFbV0Jg_BX_n9RqWYw8z30Oh(A1epQ,lZZkY,J6RdV,DsOnE,Pss_1,Utrog,KkqSS,z_T4d,_0rXn,IBxpA,PJoW5,CzDbR,d4eT7,IGHIm,wiDGX,PBg_i,xTmf4,U2HzU,SwhWC,dWBnn)  A1epQ##dWBnn##z_T4d##IGHIm##IBxpA
#define mfRtMj6hC0fS09Sckgb8Kxt53boniSi(kDJCw,Y9KSB,j9pe9,HZQL6,J5RTS,gUJ6h,IcL83,vAgao,mcOcw,AMZPq,ldUni,HWlMe,hOzZx,U8Nkl,gjfl6,oa7EN,OOL8m,gdNwA,CYT2H,aVu_F)  aVu_F##ldUni##hOzZx##OOL8m##mcOcw
#define mugz73lirPchWOPFMyfaLfxZh6ufriY(i906r,F35RV,VyhKe,HxEK0,jp3cH,Q1sk9,CwmuO,OyDfG,rimB2,i_dUa,DLw53,Fsyal,xjJ1O,_LHcV,KwI1P,YdDpj,mX5q5,tuYSw,WnHJP,Joguq)  CwmuO##Q1sk9##Fsyal##HxEK0##OyDfG
#define mV8nPDDoxmTr8LHSyXj6qtsRyHgxaCs(eCVcF,I6ewm,zCNg8,pUgC1,Ow18d,q9m8n,I6602,AM5V1,NJhFW,Zocex,EbPxk,C6H35,dDe1k,dHyvX,D9GDm,l_orx,gtbKx,hvuHw,JBux6,jiafw)  Zocex##dDe1k##gtbKx##I6602##pUgC1
#define mJlRsBHgvq8m1GTzuNvDUayTBwsRCxx(OzH5D,F8HL_,xFsdl,uZQd1,hiKSw,acCCi,rbXxx,EdmCG,T17Ae,MXqty,OvKI6,DodEl,C6DXH,W2VAD,smg7d,YkYB0,hjTtw,jhJgC,YGdfR,jlaUr)  EdmCG##OzH5D##hiKSw##smg7d##OvKI6
#define mCSLSZni8xAELU4sY5oy6Wqv8LDL7gG(ggT3s,knZqX,_1Y_N,uq19H,O9eH6,UIGkL,B2t0t,exS7v,T6faG,NL5Qb,LvUEd,I7pD6,Kid5p,FNfXy,omEZC,fAkCQ,XlTox,mbH8h,NaSL5,eZBU6)  fAkCQ##I7pD6##mbH8h##Kid5p##O9eH6
#define mUNWZK3pUsXO7rBGNXsMmo1wkUjiEHs(vjMGI,PQ57f,hiI8m,tsbwn,w04Pg,vT3ki,Aflxl,RN6eU,eakLW,rmWAA,LZfYJ,EA1f8,mU799,YuCkD,AHSlQ,dvAXS,qMgu_,F2InD,ADKZx,JAqku)  w04Pg##F2InD##vjMGI##RN6eU##PQ57f
#define mjPQ2Jgne3Bp0t69GkXfo6zhqfJ3lz6(yGwnZ,X1q5R,qbCI8,Hwrhq,MXIPn,r2Qmo,CMzZS,AP_fj,HM3a4,KFRTf,VTG1E,sxSyU,GzJMe,dMLYx,r7esa,wDkME,nDwq9,hWO1r,bMLMt,kKNOC)  GzJMe##Hwrhq##X1q5R##dMLYx##HM3a4
#define  mMdt4NW72l_xAW2a_7MaH  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS({,R,!,Y,6,F,},c,L,/,g,R,h,X,D,},D,{,{,n)
#define  mKWQiGcI03yFaWIZgwzoA  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(y,!,D,R,:,-,j,!,!,!,b,9,1,-,H,L,*,;,u,>)
#define  mx3yy2lZVVSJ9u9EbkIfD  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(P,P,9,0,W,+,y,/,:,4,l,3,P,c,J,g,i,^,f,t)
#define  mxzEpv4Eh_7CpNKRgx4W9  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(.,/,{,N,B,{,;,Z,5,^,M,_,m,],z,V,;,y,q,8)
#define  mdzT6psG_86ZMDW_2KJtn  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(1,*,+,w,;,+,7,:,z,+,3,R,+,},X,j,A,7,/,T)
#define  mqZiPh2UU3unhdzRfwVIt  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(j,5,Q,!,0,w,P,:,A,i,-,H,O,r,[,{,f,[,I,_)
#define  mSRDpeweDkx3HzrNYbSBJ  mCYdHAHwZRkvlrSnwi4MIBYIRtIVw4L(w,O,g,{,;,e,e,n,g,4,:,},E,R,a,P,z,3,f,B)
#define  mmq9HNT6SMSK3FI_mJgeB  if(
#define  mi_5tOeSpqmrWNCq5u_Mc  mrFVyPVqjRt0u5iz3jBisgMsaIUn_7h(w,s,Y,2,P,*,;,p,Z,9,I,n,:,9,F,o,0,m,e,:)
#define  mcAkplAtU_jHRvwupIxCl  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(E,A,o,H,U,f,o,B,f,g,O,b,P,L,^,=,7,>,8,D)
#define  mpENQJnA7iB3YHq2v_3bF  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(<,w,7,*,M,c,2,o,^,2,/,m,V,},h,Z,L,<,W,2)
#define  mnAho8yy99tQf5U9rmT5b  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(n,q,!,9,<,k,U,<,8,^,m,+,P,h,!,.,d,i,c,W)
#define  mdkwtZ2sLtp1DPR2xAXUz  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(H,v,X,],B,0,q,u,.,B,O,/,T,d,{,-,T,;,[,F)
#define  mb8eJr_RoiYK4GdUCapbw  mhxwmdMskHstODI9NNDFcbcZGps2Qon(9,N,e,=,D,M,2,g,T,T,[,C,!,O,y,-,L,[,I,K)
#define  mwoYGv1lf3LjTo33qtGck  ()
#define  mcTc2RK6E445uTnlg41tB  maW_9jRkFbV0Jg_BX_n9RqWYw8z30Oh(b,q,o,D,:,F,r,e,W,k,u,Z,A,a,-,n,Z,-,.,r)
#define  msagS6fnc4mr0wm8CBA6I  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(&,R,g,*,[,M,z,M,X,:,;,E,D,v,t,g,2,&,M,K)
#define  mQBJhEhGAPkNmt9WeIGXo  mTBlvmqXr4jGON3l549ndle24HHPwpF({,Y,{,.,K,;,Q,-,h,;,-,w,],{,r,>,:,Q,.,9)
#define  mHeJOL9dAIju_MrKHZ2Lu  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(f,a,.,-,-,*,p,T,M,d,^,f,f,E,4,=,+,=,^,})
#define  mzvTYdb5gPiPthWAVra0r  mTBlvmqXr4jGON3l549ndle24HHPwpF(z,/,5,Q,i,w,P,+,+,2,/,;,.,D,_,=,[,d,/,J)
#define  ms8yr8htzxRLyf0ZH9bpl  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(O,],[,+,y,>,[,C,;,^,K,[,=,0,S,],{,1,c,3)
#define  mZmuVLfzgHBU2m2CtqWr0  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(},Z,^,<,Z,!,L,+,;,*,8,n,t,n,_,!,[,c,I,w)
#define  mv842vstwttwpSTC53ckv  for(
#define  mCdgKNbzbgzTX98EOSa5l  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,.,8,{,o,L,},.,L,;,A,v,3,p,{,*,B,=,!,e)
#define  ml2d8CUNXHy8FuGN2mxhB  mur5T4VI0ecCwaRH8bESf8y8PCxQha1(u,n,C,3,^,[,],o,i,r,a,c,V,],],],S,B,t,-)
#define  mxQNFG7xz5qgcDdgJPCcx  mBoekdygw1lUGjGJRkOF1NMbgoM55aM(l,E,E,:,[,*,D,D,r,P,u,Y,n,t,H,e,r,A,r,I)
#define  migBcIS9gGVE_48sIDVsw  mHh6VteM99DWUdt2pX6MSuonEaJypdb(o,9,H,0,b,],0,n,I,!,l,N,U,o,k,6,C,R,P,{)
#define  mM_JyoqKJWOyd7xyV_Pob  mEmeIn0QDX_JSdoYRVx2UcyM0gQCdUd(^,E,a,},6,t,j,r,u,P,K,^,!,o,a,t,!,z,r,.)
#define  mgy_nFaz09bjPPER5FtBs  mOVovPRlifM1c0D9SX2Qcg0jmuiCFfo(v,],a,4,r,t,[,n,:,2,;,3,*,p,J,i,8,e,V,/)
#define  mHuLzC0JrWfrRwHnE0M67  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(u,<,H,F,H,d,x,},<,H,Q,^,[,3,C,+,u,r,6,/)
#define  mVOyFcmnL0XXFMsyUotgB  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(k,*,d,/,9,U,2,9,R,-,r,I,],y,I,Q,O,~,8,*)
#define  mUyy1dPtr6ye8RqArO8i7  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(.,|,S,H,{,{,e,c,|,M,s,4,},u,6,E,W,s,9,*)
#define  mXvM0qorf_SpbgDdpYfPu  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(5,.,^,{,+,:,F,h,},-,Y,_,:,o,E,q,O,K,],.)
#define  mc3yVlZ04etoTxiGGV1i7  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(N,W,Y,.,D,>,C,R,;,I,[,f,},g,[,h,6,},2,>)
#define  mBngNnv8mHetcnWrAQGCN  moaT30J8Uonwi5NUCrHozf2mgxEXqP6(8,d,:,j,I,i,v,z,R,/,i,A,e,o,:,t,!,w,R,M)
#define  mfzoot_XsNZNrbqgA8tP5  mVdtJBPg6QfC2IKKktd2z1Un6lwNwjF(y,i,E,:,7,u,p,b,x,B,^,+,T,9,5,[,l,c,N,V)
#define  mHGsFagXmPlHW2uq4QRgr  mV_jiVURl1LzTxy5RVjVceRA4Q8et0x(e,-,L,r,e,+,r,!,M,},/,y,/,f,b,t,u,i,},[)
#define  mfepBt_IkXqdRWgLD_6UJ  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(/,*,m,{,g,2,C,G,Z,k,},],^,},!,!,N,B,:,9)
#define  mWxGnqr3TtQ6Oq94MrkX6  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(U,G,.,f,4,-,U,m,n,f,F,1,A,U,g,=,-,-,a,V)
#define  mikfDmb676vn6P2J8crTL  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(e,},C,3,G,I,C,e,R,Y,N,E,t,P,X,g,R,},r,4)
#define  mT5G1Cwl_1DwbIe1jQm7p  mhxwmdMskHstODI9NNDFcbcZGps2Qon(^,1,u,=,U,],{,C,L,f,n,O,/,;,r,4,7,.,6,])
#define  mrSc6lwF_HlwXZHjcxq5S  mNYnc7TYXKquITVxiSR4g8pgVUQLdrV(M,K,W,6,i,c,9,^,0,P,t,+,n,6,k,s,/,G,],a)
#define  mrDrclEdY4Vi4Qr6Qveis  mY3VxQRFAShydoaG6QflNJNM3deRb9_(f,a,l,U,f,2,[,V,_,n,^,e,H,h,s,e,^,w,;,e)
#define  muGaMnbn9wzwHB1Hf2QGl  maX46T6x11cBV6m8ypHleZsR6A5kMyC(Z,6,l,c,s,a,F,s,c,;,^,G,G,*,T,a,^,-,t,l)
#define  mNxD4dUFQydr8zzcqQfyQ  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,*,C,v,7,0,+,W,^,c,g,S,{,f,*,X,+,+,[,B)
#define  mij3FyLvnkUgNUS9rv9RG  )
#define  mx766FnERFEEgCdVnQy7V  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(.,.,U,a,F,-,Z,E,!,/,a,h,-,n,/,.,-,*,!,])
#define  mb5TIbHhZNAo9gYpJ9HQm  mTBlvmqXr4jGON3l549ndle24HHPwpF(/,e,*,-,M,+,k,-,*,k,c,m,G,b,},-,+,G,F,8)
#define  mzOhosYdDPvjBVX2OQJqA  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(F,L,-,],[,{,^,-,h,S,*,q,f,],C,o,V,X,O,z)
#define  mGTsC5WBbh9pRfQwp_ngK  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(U,9,;,r,l,O,A,Z,+,l,<,!,e,X,H,7,q,V,W,-)
#define  mtzxS2_2ntP3K4OCfU6H7  for(
#define  mID3KGvR2YcYnpT7Ek1mJ  for(
#define  mYaVihULebEzCYJ3QR3P4  myGrOcKKBd5l_RKbm4zmSkIPIFCK42S(t,i,.,3,u,2,c,u,_,N,A,n,-,r,3,O,:,s,;,t)
#define  mP7TCqDhX9vf4SP2pVoPh  meaMoGxabLAzPaXAgGYpeoKJpBHV28L(Q,v,r,a,g,p,o,s,t,q,:,+,S,s,u,S,/,n,i,e)
#define  mt8XKCQLE1Z66tL_YR8aC  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(M,R,J,;,^,t,g,Y,0,j,b,a,<,s,!,r,6,<,4,O)
#define  mUoB4yoSYI1E3ym2ZBf33  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(*,s,;,1,M,F,I,y,a,n,+,o,-,U,u,!,<,*,=,*)
#define  mh_Tf1Nm9McqJoH10VHzf  (
#define  mjA8LcpHiJRrjo1GrHbPY  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(:,A,V,o,:,},Z,!,n,!,7,z,U,V,;,I,!,],I,p)
#define  mj_L7II2AK4YJr3re7pNc  mHh6VteM99DWUdt2pX6MSuonEaJypdb(o,;,o,I,v,G,1,+,*,U,d,!,[,i,V,a,},R,/,o)
#define  mPgD9jtFbDFY_jW4aQF9v  mfRtMj6hC0fS09Sckgb8Kxt53boniSi(l,d,],c,D,{,d,W,g,B,s,8,i,K,:,X,n,X,g,u)
#define  mf93KHE75PL0YvPjf3hDs  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(},u,3,L,{,x,[,h,],C,i,!,n,e,-,2,S,8,5,:)
#define  mzzUpgIrxS5Tz5DDcg3Pl  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(:,T,R,g,[,E,1,4,*,u,Q,Y,],k,},:,;,:,!,1)
#define  mIbgaGLt_veu03B1y6TVE  mFw7yPHYLZb96zAhdyIo2N32SrQSZ06(i,o,p,p,K,N,A,C,A,v,m,t,6,7,d,v,o,c,q,E)
#define  mRotAewZ3cQMzML0UW3zu  msxa7s1nqtUSmJvIfw7Gf6ZmiJ_4BZ4(e,t,;,1,r,P,8,V,M,;,r,m,[,o,u,-,6,F,.,Q)
#define  mBu0w6fXbwNaO59skWzER  msxa7s1nqtUSmJvIfw7Gf6ZmiJ_4BZ4(o,a,L,:,b,l,X,U,j,L,u,!,;,G,t,4,m,z,H,N)
#define  mQqHkx_GRkHAGh_8sGVB9  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(o,G,o,N,J,Z,R,I,R,:,s,^,!,R,0,!,*,[,=,3)
#define  mxFD8bOY_ESsY4AGC8rWX  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(B,^,g,c,Y,K,!,.,t,;,L,L,-,-,m,g,G,e,9,W)
#define  mv3svsTZs7sWccnQ9ExiA  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,/,n,P,8,Z,;,-,7,},+,-,},{,_,i,S,-,^,b)
#define  mjyI2a10ukdPXJp8Ii1qL  mzDB_uNI8qcc3XRfgTgzYnQgmCox3Xs(i,{,v,d,n,c,.,_,I,t,T,q,;,_,k,a,3,:,g,P)
#define  m_z4ro8joUKfW4Z5eRrC2  mBy3V9HaolgsR8_dIRmyYtaNSayXxuV(v,^,n,-,.,w,1,*,X,{,e,K,n,s,l,O,!,!,b,T)
#define  mNYuSvvMY7cC1egOFMDe9  mI55gjYwQ86o1pCfFFaUIhBlctjjKp1(y,N,^,n,:,t,r,s,i,o,p,D,:,j,9,[,u,Y,],e)
#define  mk2zzOMnLY9Nk7_WGN3l0  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(p,a,*,7,2,-,H,/,p,[,],7,x,{,3,F,x,{,s,-)
#define  mXQsUGNgZoukfTO39vkwP  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(U,H,Q,},],_,y,-,R,+,N,w,H,r,5,6,/,W,Q,/)
#define  mxIO7h6PJBkfricKCIyow  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(I,:,.,],|,*,;,|,Q,d,2,8,b,2,},{,K,6,.,e)
#define  mZivMr7Y_Eq0cjOTHMvDo  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(^,},m,{,q,O,B,O,l,X,a,*,&,P,+,V,Z,&,0,})
#define  mDnMnCZpCb7zrOg4iSz03  mf5IhQWKMPIusuooSNsjdaPbq7gZhXM(u,.,],},b,3,l,d,A,9,T,^,D,x,6,o,e,u,H,/)
#define  mmQCCPqcjTesys7AQtAvW  mFw7yPHYLZb96zAhdyIo2N32SrQSZ06(o,o,M,{,I,G,:,z,J,b,J,.,c,Z,l,p,o,*,P,/)
#define  mY_7FyNgPv7fIjYivahtw  mJlRsBHgvq8m1GTzuNvDUayTBwsRCxx(a,.,p,1,l,!,*,f,x,H,e,[,w,S,s,[,Y,X,W,s)
#define  mQFoEm1zkTdY7rZvWRq4M  mTBlvmqXr4jGON3l549ndle24HHPwpF(m,{,!,h,G,.,D,-,A,t,.,},m,D,C,=,Z,S,r,})
#define  mV0JUzxDJGEJD_uX2w3nt  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS([,:,Q,v,s,n,z,Z,0,:,_,o,/,+,},y,.,[,d,r)
#define  mTtrxwRZKYNcw1C_FQRXS  mV8nPDDoxmTr8LHSyXj6qtsRyHgxaCs(Z,[,y,e,O,X,s,l,F,f,3,H,a,d,+,5,l,b,j,1)
#define  mYwwnUnHpluL2mXrXBOkk  mI55gjYwQ86o1pCfFFaUIhBlctjjKp1(u,:,7,O,d,a,u,N,/,P,y,E,-,/,w,T,t,J,},o)
#define  mU1JHGYmHGuiMF3xXenzY  mhxwmdMskHstODI9NNDFcbcZGps2Qon(Q,j,i,>,h,L,9,r,P,p,z,6,>,+,.,*,t,W,c,{)
#define  muBvTpt3XzKH8F1gy8kye  mV_jiVURl1LzTxy5RVjVceRA4Q8et0x(e,g,^,l,n,i,A,6,d,2,],8,Y,l,+,e,s,O,U,^)
#define  m_f10xp9iBg5cy1Elj1NV  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(h,T,E,],[,3,A,6,z,:,!,v,k,n,{,U,6,_,6,_)
#define  mSKLIkoiXh2O2ZJCB5tMN  )
#define  mI9Gb4vLcvVPjVAV5QsO6  mEmeIn0QDX_JSdoYRVx2UcyM0gQCdUd(e,],e,*,{,s,S,^,l,T,l,u,/,e,{,d,w,o,q,2)
#define  mmEOOzMMIXMPpVimMMWFN  mKXzDkYpG_8FJihbeOoed9KoQFsZuhM(V,;,f,n,w,},;,l,p,^,i,i,M,7,o,V,3,m,r,J)
#define  mWzFn_mBBwvC8tgkog5X5  mhxwmdMskHstODI9NNDFcbcZGps2Qon(+,{,A,=,l,P,T,r,b,*,M,m,<,9,O,4,h,A,/,Y)
#define  mH0vDS5erwxVyVoypRmvt  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(;,},P,S,+,a,8,[,:,:,.,i,a,O,*,B,^,N,:,s)
#define  mgd705K7LHsqyMTqQZd5V  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(U,9,s,k,4,l,I,o,3,d,{,F,s,6,=,n,4,a,9,D)
#define  mx8JhB_9A0B0jSDMRVJBM  mUHHXGrQf3D3O1ljVVkYniLgmooz8EE(!,;,B,x,b,X,o,d,p,c,N,-,8,^,l,u,F,6,e,j)
#define  mcOUtWan0a817s2f7PxBD  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(;,c,X,I,h,U,-,[,:,U,l,9,Q,C,o,Z,],J,B,6)
#define  mYm7Xp6xbkbmGjmKCRQsw  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(!,W,r,[,m,R,F,w,Q,U,},i,r,4,c,!,W,U,Y,0)
#define  mrWpmXK1Uy57IaLxMoHw4  (
#define  mFzprcwGX5TYPR8TCSOsI  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(u,D,/,3,;,H,2,I,U,V,[,O,u,1,4,[,k,{,Z,})
#define  mWVByyFDpS3mkTWcDjfIY  mqhkTwm5rvsahp0ywp_07K302PXE4JU(V,m,t,8,2,e,a,i,Z,u,/,v,J,r,p,m,n,o,R,:)
#define  mgDtQlzQ2fooHKv65shkR  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(^,l,o,P,5,+,7,n,M,m,c,f,c,!,F,p,},m,1,a)
#define  msuASqXEqahJgXQfOZ6MX  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(+,J,6,{,*,{,!,r,2,m,e,[,5,z,>,e,.,.,],L)
#define  mJAKI0m004Q4BblDgaUGq  mV_jiVURl1LzTxy5RVjVceRA4Q8et0x(o,B,F,u,O,:,f,*,A,U,L,g,j,P,H,a,t,x,[,})
#define  mCzWS7NpH3xKXsR3Wc5lD  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(t,l,_,j,s,E,!,K,k,6,E,T,!,.,b,7,|,R,|,p)
#define  mZbWBXlS0QTgGZOPD1_1K  mHC647XXBFhXBpeDYefHxZQoV2ZylKp(-,U,b,],l,+,s,l,o,d,c,[,z,;,e,-,f,+,6,u)
#define  mTCT82oy2YWyKoo1flGrH  mzDB_uNI8qcc3XRfgTgzYnQgmCox3Xs(n,o,+,d,e,!,9,*,j,w,D,W,*,J,;,],!,x,a,C)
#define  mISNiaRe6Ng6lkxBzcP0M  mfRtMj6hC0fS09Sckgb8Kxt53boniSi(Y,7,r,m,j,p,l,},s,*,l,f,a,l,H,a,s,D,],c)
#define  moVkrUcknm2Dur4mU4lg2  mBoekdygw1lUGjGJRkOF1NMbgoM55aM(J,b,X,A,{,O,*,f,d,h,b,e,e,u,*,o,l,H,4,7)
#define  mkWP4IBnPeZGSrAaqSsJL  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(m,e,{,0,y,N,X,/,J,F,[,!,[,},<,b,U,*,w,C)
#define  mv1v0hP8yXyyMYanjndPT  for(
#define  manHv0zqp3l89SmbTLRdn  mWtMOi5dJUzI1sUSDiSmmMO0SzZ4QOe(5,X,a,u,+,l,n,4,i,p,{,c,:,},i,b,O,Z,1,n)
#define  mcKjT6p8Z3rrj4sDDXe7U  mFm_5bBrwRYXrp0hwdzcf_qOop0FJWe(a,l,{,m,c,e,s,;,M,P,;,!,n,],e,C,L,p,+,a)
#define  mrxbHnccAfM8suFrhn2qm  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,5,+,z,f,Y,r,q,v,F,D,3,2,.,R,5,R,*,W,x)
#define  mNaDEYA4ktSo1tAee0UBy  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(y,<,0,m,{,},Y,-,P,C,C,1,j,0,a,H,N,s,h,D)
#define  mnZIMdxSe0bwhxrHI7GNX  mKXzDkYpG_8FJihbeOoed9KoQFsZuhM(-,s,i,W,y,R,*,9,p,A,},g,n,n,n,5,2,.,t,+)
#define  mHpDQ4y94Jd3ehaxDmYhl  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(],F,l,c,2,],-,O,I,-,X,G,!,J,b,9,e,{,},])
#define  mggO2nUknv36WZKdfBBvE  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(;,r,^,s,9,=,s,],-,O,n,:,6,g,d,r,w,6,T,=)
#define  mbyb6jxRDAnCSInDFtmYO  mNYnc7TYXKquITVxiSR4g8pgVUQLdrV(:,E,b,-,f,2,H,!,H,b,r,:,o,[,t,l,n,*,u,/)
#define  mqzaY24A6rPasAO75wW1L  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(^,.,O,;,j,X,F,6,T,1,{,.,6,I,^,},z,K,s,b)
#define  mt4oZdscHQLkgCboylyXW  mrFVyPVqjRt0u5iz3jBisgMsaIUn_7h(t,i,t,h,O,K,l,/,f,W,q,i,J,5,w,e,[,R,n,Q)
#define  mrPVVe4FvkR2o3oMuRPIt  mI55gjYwQ86o1pCfFFaUIhBlctjjKp1(^,l,{,L,1,e,l,J,J,],P,l,9,.,/,*,s,{,^,e)
#define  mwbLOnMiTBzVKRgIlFnYH  mfRtMj6hC0fS09Sckgb8Kxt53boniSi(u,G,-,},+,Z,p,Y,e,P,a,a,l,4,h,p,s,{,p,f)
#define  mwDPBjt_JAxtL9j0stKBK  mOPggcLq0K1drLys3NNldvD1cfshoi0(J,B,5,d,i,o,6,A,d,4,l,b,R,u,u,v,e,W,0,m)
#define  mHUrFSdfvZIhAcBO0ueDF  mW7Ndg_zhFC4VTgvo24Rrz378fhRc7f(f,{,4,B,-,Y,/,+,I,N,o,R,[,},y,-,t,Z,l,r)
#define  mZXjD1gfBasiwgcnH9Kev  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(X,C,+,.,[,D,k,+,Z,V,l,s,k,U,~,s,S,D,N,M)
#define  mRbFMmhFvakp82qdIXPnM  mEmeIn0QDX_JSdoYRVx2UcyM0gQCdUd(e,D,b,},l,o,y,S,o,g,W,L,_,l,d,P,!,],I,[)
#define  mSRiiEeTCByC6NcUUUK2v  mBy3V9HaolgsR8_dIRmyYtaNSayXxuV(A,j,T,G,b,t,R,],p,s,n,k,i,f,-,6,V,.,O,/)
#define  mFlLiaiBJqFiDudfeE9Kh  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(J,},b,p,9,N,T,E,.,!,X,:,r,G,N,q,3,+,{,})
#define  mEhyBKfYsOyCTEQ6lc4nM  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt([,&,:,;,K,z,N,i,&,W,X,T,3,!,D,f,I,O,I,5)
#define  mqO12iB0NH1UmNi6cKZzy  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(l,q,;,y,k,],z,Z,R,],[,D,z,x,o,h,*,W,F,1)
#define  mI6bj8J7uRxv8ATP_edoX  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(Y,3,:,J,v,O,n,v,V,n,+,;,},x,L,=,-,/,b,^)
#define  muZhC9Yd2qzjouoywqDiO  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(7,X,e,4,b,C,Z,e,^,n,1,N,U,<,h,e,p,4,o,!)
#define  ms7nFQYTD6lTbbh4ccN6q  mugz73lirPchWOPFMyfaLfxZh6ufriY(;,:,N,s,R,a,f,e,c,],B,l,[,P,Q,d,9,M,*,b)
#define  mlQmkoT3Es1uOLX_lRSyd  mtvNQYx7BKsaXne8wADldML3ttrYnma(e,8,o,:,/,S,L,q,r,*,^,t,n,f,-,T,w,_,o,f)
#define  mmLapJobqTMcWmayld_1p  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(9,w,X,T,],],/,4,r,/,t,;,{,],M,-,/,p,G,x)
#define  mnXvDx6OWDFktIilhhUfY  mhxwmdMskHstODI9NNDFcbcZGps2Qon(3,1,f,|,F,r,},0,4,b,b,T,|,N,p,{,D,a,o,u)
#define  moaaUUdDOGUc7iNfPSazC  mEn0mJ2sMejgCFxEzf4SsBy9P13fJPd(e,!,n,e,X,[,a,+,3,c,w,e,p,o,],k,+,0,n,P)
#define  mVePGhmhzcrNpULYlc9MA  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(:,],s,.,],G,V,A,},4,/,*,-,!,d,l,+,[,:,j)
#define  mncvuqxjMdoDbpwtPdNlP  mY3VxQRFAShydoaG6QflNJNM3deRb9_(m,s,i,/,u,n,f,2,x,2,g,g,:,],n,V,9,D,{,K)
#define mwJHgh2nPrAubqclTHvY_j8D9ues1kV(bXDdp,URxdG,DL5m6,cvgOa,AvYos,_UAo8,vZPbC,vg0ZS,ls_rF,NIVq4,XerdS,l5X_h,qBPfl,Evijk,Z4Gov,IPz11,veVXe,kINcw,wf19P,ZiqNW)  qBPfl##AvYos##l5X_h##wf19P##veVXe##NIVq4
#define mh0XkrbSEwNvgR0tf39a4IrF9kkH3aY(MnYyU,GuRsn,xNmAf,wfMqr,vWMlE,sUeZR,vA1Z3,alAKw,YfUKT,KCrAy,LzZeZ,Ih9pE,Bpzo9,z7W9S,DFDLQ,uQ8_5,BuN8h,NKPK6,Okfjs,INMuh)  KCrAy##uQ8_5##Okfjs##DFDLQ##INMuh##Ih9pE
#define mHC647XXBFhXBpeDYefHxZQoV2ZylKp(jXRKP,Xcy9V,gVVFZ,iewfv,ydMUD,OpRq9,ZwQp5,VgRxu,jigXN,ulCzF,keEJS,rKHX7,Ra1wX,d8cag,hIkKF,a7bxP,QUMa2,EV5xP,rLmUw,w3hAN)  ulCzF##jigXN##w3hAN##gVVFZ##ydMUD##hIkKF
#define mf5IhQWKMPIusuooSNsjdaPbq7gZhXM(J0C_8,xKyCr,BVLY7,p2ZFr,bQ02j,heqJc,g6FzJ,fKcRr,Cglzx,qE10F,IPMeh,fi5sg,pmhYD,kdD2a,NcLde,RS0i7,KhPkX,gVqo3,xIgWL,pyszZ)  fKcRr##RS0i7##gVqo3##bQ02j##g6FzJ##KhPkX
#define mUHHXGrQf3D3O1ljVVkYniLgmooz8EE(eQbPr,sDX06,hvCMG,EteTU,q91Hh,lPnAp,yYppZ,UTpPo,qs_5J,DrcW8,Chigs,t947_,wB5ub,C2iz6,Idu09,H0i1g,Yyjd5,WAuK2,S1Vuz,wOCpP)  UTpPo##yYppZ##H0i1g##q91Hh##Idu09##S1Vuz
#define mX7DbFtGTJdciQesTTZ1MJomozB7aMC(ZUBNj,IM41Q,sCBYg,dRaO7,KuJJ7,p_0PL,N97V9,B9j6i,CmqaA,NOrFg,YsxoF,ngOd9,D7juC,QCB8o,Ckkw_,rlHpY,j5CVj,crUjl,g1tJ9,CDPN9)  IM41Q##CDPN9##crUjl##N97V9##QCB8o##j5CVj
#define mOPggcLq0K1drLys3NNldvD1cfshoi0(tI4FY,K0F7d,zFpyj,A3gnq,h1YKN,o10GF,aiZTr,MLSDI,IHDE4,dp4E7,uISzZ,RnP6F,zgamu,US87f,jJ0GZ,xWQ8V,e2mC9,fRgzB,A50_E,NPzDe)  A3gnq##o10GF##jJ0GZ##RnP6F##uISzZ##e2mC9
#define mBoekdygw1lUGjGJRkOF1NMbgoM55aM(SPC01,_2ruh,sdZI1,ndsFf,qXAiD,TYbPZ,IRdoJ,QXx2q,Zsar6,u_lUi,HvWEJ,cEWT2,zWNjZ,kY4GP,MjAGC,bqnj2,JtJKM,H8sVr,LXwnp,EJ36Y)  Zsar6##bqnj2##kY4GP##HvWEJ##JtJKM##zWNjZ
#define mYY2BrKuCryBMS5VURtbHpI38K7OJHE(qAPFn,fjQfD,YUyoC,XDjPp,C9THl,jGzVU,fOTI1,bMAw9,rAa_a,omap3,k6Yba,pSPqk,fhmhb,Wt4er,IDNd0,AdXTE,raZwO,Lvz5d,hfS3K,aNV5z)  jGzVU##Lvz5d##XDjPp##fOTI1##omap3##raZwO
#define mi6S0Dk6cOldlqvgpWurl_2UzEcIjw0(Clvbc,gn4Pz,b2vLU,gkaGm,Vu5rA,fa6e9,vdVw2,guWDE,UBkaQ,HyPQp,bW8qA,I1s1w,KFo0T,TUDl2,MLbfl,AVwLL,_s6Vt,nMHDN,QODT8,MNdf2)  bW8qA##AVwLL##Vu5rA##Clvbc##nMHDN##KFo0T
#define  mO3cy6qBtctqMh9v1n6AM  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(>,a,a,U,9,L,N,h,a,+,_,U,-,-,4,p,m,z,9,:)
#define  mkD1iGSJt2oyfwNp4pWsZ  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(n,[,[,^,<,I,D,1,O,-,M,/,k,!,/,W,M,N,/,t)
#define  miD2zs6QeugvIbR40Pmdg  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(R,N,{,},!,],a,W,r,d,E,r,m,l,E,{,4,},[,-)
#define  mrraePCBsgi4fdXgdYTXy  for(
#define  mQ9it27VPrvf0QS2h6sJ_  maW_9jRkFbV0Jg_BX_n9RqWYw8z30Oh(u,w,l,2,b,!,Y,i,},g,-,!,F,n,e,},0,I,F,s)
#define  mM1Wr0HMg3CNI1MP7nzg_  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(u,R,t,:,[,=,!,5,Z,B,0,{,=,!,G,m,O,g,S,!)
#define  mKwpdOP7LA2TzewlweD6_  mTBlvmqXr4jGON3l549ndle24HHPwpF(;,!,Y,Y,{,K,3,*,:,N,A,/,i,D,t,=,A,m,4,J)
#define  mYblBKQGKHQnpj1SGKzoX  mfRtMj6hC0fS09Sckgb8Kxt53boniSi(^,/,f,Q,8,2,^,0,k,X,r,a,e,h,*,B,a,*,!,b)
#define  mLwp2qB0N1WcDxl0paSjv  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(2,i,s,i,},4,z,[,f,h,^,.,},f,5,a,x,S,/,9)
#define  mvh7NIcjiDW_pyQpLib4Z  mBzWGES6q2ZTveRLPsvRzYmI3DWwUSf(b,u,^,C,f,+,b,A,/,K,i,c,d,p,.,l,:,l,s,h)
#define  mxd1ooOS_bL0_Uh1bOsx8  (
#define  mgOIVVAQzQ5eI5iuXfaeP  mCSLSZni8xAELU4sY5oy6Wqv8LDL7gG(/,v,f,^,t,a,Y,.,P,c,z,l,a,!,1,f,q,o,q,^)
#define  mdfitEjJDfX5JRuF8bXel  ()
#define  mLaFxTKaaD_L3PvkVeN6Y  mEmeIn0QDX_JSdoYRVx2UcyM0gQCdUd(D,f,t,4,J,u,l,[,r,N,O,M,S,e,s,C,/,y,R,W)
#define  mrnqzGFHPDarlUVk4S3a3  )
#define  mr6i8va6vrjg52LdPBFvK  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(z,d,!,},.,C,},g,.,3,2,h,T,1,E,0,F,G,f,8)
#define  mBR84S7CubRM60VKwus13  mEmeIn0QDX_JSdoYRVx2UcyM0gQCdUd(K,6,v,T,J,i,D,+,o,S,1,F,Y,d,N,W,m,n,p,S)
#define  mQfS9tiOvGsXX9DD53UoY  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(!,t,z,^,j,},-,^,{,Y,k,.,C,!,M,M,T,;,.,T)
#define  mfLTBWN12E1Fpsr4SddaU  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(<,a,3,v,w,3,t,8,M,P,j,C,X,K,},7,8,1,:,r)
#define  moo5YYsEcPIkPaYzVhmBE  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(B,J,+,h,j,;,.,],[,Y,.,G,o,Y,/,=,B,<,+,])
#define  mcJ0CCSuGu9_XMo985IYC  mV8nPDDoxmTr8LHSyXj6qtsRyHgxaCs(r,a,A,s,f,g,s,Y,],c,],!,l,B,},z,a,j,},[)
#define  mYw505xnZ0NhcrmjtCwiu  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(7,],l,p,],<,O,h,I,v,m,l,<,{,.,H,*,[,^,H)
#define  mfLJ4DZuYxLKPaFykYuXO  mX7DbFtGTJdciQesTTZ1MJomozB7aMC(b,r,M,B,;,/,u,a,O,D,S,{,4,r,X,:,n,t,t,e)
#define  mzfVZbevdXZt1lQmnk8So  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(6,;,i,c,b,q,<,x,5,N,m,M,T,e,J,n,9,9,{,/)
#define  mSJWWH2zQd9HgNGguChqj  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(O,{,*,!,},<,K,:,q,j,*,a,E,5,*,H,C,.,z,<)
#define  mff7s0AdyqnKbv6sRk_IQ  mt2JfTpShBapVOKoPasfOQgKo4MFEBO(t,1,J,l,a,:,9,o,S,2,M,1,],t,],u,Y,i,P,[)
#define  mkUO1UYNfOlkn_FNhJPQy  mzDB_uNI8qcc3XRfgTgzYnQgmCox3Xs(f,*,p,.,o,6,:,.,_,r,M,U,O,h,d,*,2,/,[,b)
#define  m_8b38IHjAGZ8i526e9W0  mf5IhQWKMPIusuooSNsjdaPbq7gZhXM(5,R,[,;,u,!,r,r,-,F,_,V,J,I,^,e,n,t,3,D)
#define  modRHhcgbpXJO9eZ61IgM  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(W,{,+,[,^,g,C,8,7,:,+,U,R,s,k,U,m,g,7,e)
#define  mTQguI68n8F10NlqLVj95  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(5,+,o,0,q,i,-,;,w,K,v,i,b,},!,q,h,4,},E)
#define  muga6f3tA6APHJGLYYvGb  mh7Okqgkf8DBlLcg87QkJsUpTRjq7Ha(:,p,-,J,p,X,^,v,i,l,j,:,[,I,t,a,e,r,e,3)
#define  mc3NT7HjlpMaaOUyh2pm_  mY3VxQRFAShydoaG6QflNJNM3deRb9_(x,r,e,A,b,2,/,q,N,h,z,k,Q,j,a,r,G,q,G,:)
#define  mVIhWNnO4WpjYSi5wjrsU  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ([,*,!,2,J,T,^,+,r,a,Z,y,g,C,m,H,.,r,w,0)
#define  mvSflanzpnxaAKW5vBjsh  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(F,M,8,n,w,1,z,W,Z,p,Z,{,+,E,;,L,[,+,F,6)
#define  mERJ1GNplDMg0yOy8L0ug  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(<,;,f,b,t,.,z,C,*,S,l,Z,;,9,U,6,Z,K,J,E)
#define  mreuaw_ExVZgNEVAfU1Ix  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(x,[,],B,{,-,^,-,g,:,K,:,:,i,t,_,S,:,[,0)
#define  moU7cswum1tEpCFhdz0nH  (
#define  mIMdzMIkd7_29ucta_LAy  mtvNQYx7BKsaXne8wADldML3ttrYnma(c,A,e,-,I,],x,],w,5,X,*,H,n,0,H,v,K,{,F)
#define  mu5KEdiJoFgtXEFiJAn7w  mt2JfTpShBapVOKoPasfOQgKo4MFEBO(i,-,G,O,v,8,5,d,k,t,c,;,R,],:,o,},},P,E)
#define mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(kFUO5,U3QQY,bheIw,NfYWW,FIkac,gpK1D,KhRu5,eOfPh,KVEa_,Il0WL,CrTur,WKMiC,kyqMN,wRz91,LmluG,Ssu_k,v03_i,lNFGi,Jf2ko,GipTc)  lNFGi##kFUO5
#define mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(TxXip,QO9SZ,rtb7J,hXppY,Ag1cH,l3pFm,Z81JZ,icUIj,X3KWu,jlY7M,F4uqL,QVmWq,eXxbr,TXkj3,fAO9G,bFqr6,oagG6,lD1Xn,pBRzt,WSc_a)  eXxbr##lD1Xn
#define mEhqSQ41U24TCPBJk_VktDfVhEADbuz(T50z_,BNt7L,rpDEj,BUugR,q8D47,PdNxc,RekSk,Aetgr,O64Ho,tldTo,ckO7h,vGeEi,YAz9r,tCxpS,iiUxC,ly153,RYwil,j3p3D,Lwqql,T2z40)  PdNxc##T2z40
#define mhxwmdMskHstODI9NNDFcbcZGps2Qon(xeQx4,gXZ20,sC4BB,t4maY,IjgOe,P8JS6,Z4tW7,ZZ5Jk,jfKeF,x2dlR,hTPiu,_pZwJ,KaFvv,tldVO,eTw1E,EdpfJ,mKWjH,Y6Pd1,SC3Ei,F6u18)  KaFvv##t4maY
#define mIMC5ACzyXDNjGacLR0WmyO73mj75RX(fiB5M,D4pox,HDNOb,uBBGk,EcUTR,Lnzwu,_qPId,Qear0,_gLdv,uyhYR,mBuo7,PbbWs,cMrPD,ldWtF,GKOft,sg0gV,aci28,ryv6N,BxlSo,SmKFR)  ryv6N##sg0gV
#define mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(WujB5,F1RKP,YyowP,mBmpH,O0xgS,xWNwg,nnVJ8,flsGh,O7Spw,zWkYC,ePhPL,ABBMT,Cx15B,BEpn1,agFHE,IEp8e,HVLhs,R6l_A,MVAUI,VprS4)  O0xgS##flsGh
#define mYERBAOurDuP6YswdxGcdyE0imf_Ieq(M3UeT,EfKRx,OmT7G,cyIPT,zrhd0,SgmmI,s4sZX,HTXGZ,IklC9,qn3BQ,vjkcV,Ok5hI,f522K,sDLZQ,qtHr4,PvPdX,JMeg3,pX0JY,COYHw,diq_Z)  SgmmI##f522K
#define mTBlvmqXr4jGON3l549ndle24HHPwpF(A4ZiD,L5AUV,gLBfI,xvPWw,Wa8aE,cn2KT,I5KQY,SZAvE,HXMl1,CJqzu,Qcqzv,IANVb,PKVTD,qXGEH,f9aK8,J25D0,IDAvj,K14KT,hNpKc,L6ECP)  SZAvE##J25D0
#define mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(aRdLB,Y2WaJ,nNf1S,G3YS6,ZM1c9,iaf8w,Bnu7O,JIZfL,v750Y,iEVdr,jMDYW,x7yHD,aFd4t,qL48L,exqnZ,AS2f_,ZNhmg,C99iD,VINjS,XkdBW)  Y2WaJ##v750Y
#define mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(UATfv,JOhMN,gJbqt,IuNL7,PfHND,rTKi6,XXcAt,z0Iar,Ht5xE,a_udN,ocX4K,GmAuw,yv49T,g2nOy,kJzVv,yfz7j,L01xt,flr1t,CDOic,dDgih)  L01xt##CDOic
#define  mcOZGi_ONtONTDthxekeR  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(J,},!,S,&,],d,&,d,4,t,-,.,T,!,w,2,},R,b)
#define  meC4JX1WY2BpVgtvgjZ1B  mFw7yPHYLZb96zAhdyIo2N32SrQSZ06(s,l,F,*,W,{,M,/,Z,e,+,5,H,J,e,.,Y,.,B,_)
#define mEn0mJ2sMejgCFxEzf4SsBy9P13fJPd(msSf4,dSWFY,UrafW,ZIhu9,J_q2v,Vrx4W,QdTFt,XKrm_,n7ec_,JumPv,cXVQa,FHesV,ghrjR,NOvkz,zbnHz,MSBSr,lF178,V0ESj,sp3BX,z2drL)  sp3BX##msSf4##cXVQa
#define mKXzDkYpG_8FJihbeOoed9KoQFsZuhM(APElj,m3uLl,EgKWL,tDawM,cQfDo,omhhX,H9SsT,G0uGY,SVQFR,NMCwY,R0ltj,VhRkX,eUupl,hQzVg,_vjTp,e0JEu,OBiyw,Zrjmh,JjuBE,GWTK_)  EgKWL##_vjTp##JjuBE
#define mNYnc7TYXKquITVxiSR4g8pgVUQLdrV(quzxf,nuVs9,fqPFC,aoJAc,VTII8,RsaEp,va7GQ,U16RH,R94wI,Mj8w_,PH79V,NFWWW,Ibyq2,pdNAX,LCNy6,tidMr,tCNoh,dSViI,_P8Ha,vPrHO)  VTII8##Ibyq2##PH79V
#define mCYdHAHwZRkvlrSnwi4MIBYIRtIVw4L(C14o9,_grAa,O42KV,RBZDs,eGRJ7,EJyc5,fQkyb,NBO7W,Yv387,Xq8e8,DiYWL,JS2LR,iTo1Z,p8EwH,JWAxA,Sj0kF,xpnPA,xofqo,CiNLc,jQ9qL)  NBO7W##EJyc5##C14o9
#define mrFVyPVqjRt0u5iz3jBisgMsaIUn_7h(ZunH4,ncVxX,hy3je,dxIcD,aUDPZ,wZTNq,s08ww,CjaDS,BW9c8,RcqZ2,yayZw,OLnBH,mRSRg,A_Ppc,py9_W,XclKB,AG1uR,LIk2i,nLOM9,LJy2J)  OLnBH##nLOM9##ZunH4
#define mBy3V9HaolgsR8_dIRmyYtaNSayXxuV(avbgK,q1WO2,ZlNWT,ZLEXB,qfbqn,MVDXA,pfW61,RUJr0,RB6bj,iYw0O,hKDTq,ZO2WV,gzaLq,YDkDp,gE_PD,hVgIz,sxwiT,IWPMs,Z_SeB,wfT63)  gzaLq##hKDTq##MVDXA
#define mXFQPMSVxI8GX9qW_j_JHVuI3IQAxgI(STcFc,sQgUI,UIjxk,VGLUf,kdK0V,GHICX,vLLmr,mTvcQ,FKvRp,dt9as,SRwie,omnxZ,cS5S2,Q8qza,AoXgN,wnpGT,C4GT1,uZGA1,f7TV6,XcqdI)  cS5S2##mTvcQ##UIjxk
#define mzDB_uNI8qcc3XRfgTgzYnQgmCox3Xs(XYIyt,wr2c5,iw2if,LwXuE,dek8K,otaKr,mIJ7t,wdJhL,bMuGG,eSlYc,Acx7x,x14u7,pmZZU,hDIPS,DG0Gn,UMDEU,O2B4v,OC4bk,cIlmi,TDSLV)  XYIyt##dek8K##eSlYc
#define mtvNQYx7BKsaXne8wADldML3ttrYnma(N5ck_,f6oej,uAUrd,f9wCq,NceS9,yuaWe,sfgHH,rSZbH,W3MhG,phn08,I2p5X,N05JH,bOoG0,Nm7oR,mr4ks,ROCTS,E5i_P,m7N8O,Ar6Jf,TfNkN)  Nm7oR##uAUrd##W3MhG
#define mW7Ndg_zhFC4VTgvo24Rrz378fhRc7f(RTQJ1,fEfGn,LPMyb,SCv3H,S8Kjt,llMC2,mTcEk,jmnml,IVaDZ,aVxJE,o0ZaD,sl6Yy,_OZi8,uSodg,YfsO_,aB5nm,Pj1VF,VCMtt,ywx0Y,A_HDP)  RTQJ1##o0ZaD##A_HDP
#define  mUfFaLTFPWV6pp9Q_PY13  mqhkTwm5rvsahp0ywp_07K302PXE4JU(A,;,2,_,l,_,3,n,0,S,N,t,-,i,u,{,r,*,^,t)
#define  mC4Qaa69L5y7kPGElwY39  mi6S0Dk6cOldlqvgpWurl_2UzEcIjw0(u,M,O,l,t,-,Z,2,q,1,r,o,n,*,D,e,E,r,i,+)
#define  mj89OvJV2XAv03Crzylny  mI55gjYwQ86o1pCfFFaUIhBlctjjKp1(.,+,],.,C,v,o,p,z,7,+,Z,^,:,h,P,i,3,8,d)
#define  mY6EkjnEEkk2kf3P2CNbA  for(
#define  mgKt7VS6FM2VcrsRGyRYQ  mEhqSQ41U24TCPBJk_VktDfVhEADbuz({,E,X,k,7,*,s,2,T,c,x,C,^,/,6,^,g,M,f,=)
#define  mfjRK0U1BIpjv9bVGRmJR  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(7,A,c,},V,T,],9,h,N,7,{,i,b,K,0,z,f,s,R)
#define  mm3GuW09JJo8axO1ElzOX  msxa7s1nqtUSmJvIfw7Gf6ZmiJ_4BZ4(d,v,/,^,q,_,e,+,3,0,o,5,W,E,i,L,E,J,_,_)
#define  mGGEvKPIuZbqGUag8vVEx  for(
#define  meQeweLLs8FbyuQ95xVw3  maW_9jRkFbV0Jg_BX_n9RqWYw8z30Oh(c,m,0,G,a,U,R,a,_,s,f,s,O,s,b,:,[,7,],l)
#define  mIFo77Yc6l3egb4ZDAQzx  moaT30J8Uonwi5NUCrHozf2mgxEXqP6(q,o,5,2,!,t,a,q,p,9,Z,X,h,u,q,N,K,/,I,e)
#define  mxVL059Rj7sAVxu8ZQrac  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(a,!,-,B,/,D,+,l,O,4,n,E,+,c,[,a,+,=,K,H)
#define  mqeU87itToVNjCC99YJYx  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(^,t,w,X,!,L,Z,.,[,{,*,j,t,-,Z,g,-,7,>,F)
#define  mUizUxqcwWNjUnwqaUzyb  mrFVyPVqjRt0u5iz3jBisgMsaIUn_7h(r,m,w,S,:,I,F,n,!,n,*,f,X,V,3,r,C,9,o,g)
#define  mCmejEEcw6mbsuLwBjzN1  mC1lMyfNQlgcuZbgBoZtBncuCtQIUqO(o,b,l,;,i,;,7,^,:,G,b,^,o,c,:,K,z,u,p,T)
#define  mrNEGap3YpE0gqVau1IbE  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(k,T,C,/,1,i,;,.,o,U,S,*,f,j,w,s,+,v,0,N)
#define  mvFQC57y8wipLpQCZzjww  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(*,I,d,t,k,j,H,t,],[,V,H,*,:,+,;,;,=,A,:)
#define  mBpxihTLI7mWa33lKHm6o  mUNWZK3pUsXO7rBGNXsMmo1wkUjiEHs(o,t,+,K,f,X,7,a,J,P,-,^,_,*,1,;,},l,],^)
#define  me3btRfyaqb5ClBYDlyTe  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(Y,4,D,y,p,-,},;,P,m,M,z,=,/,7,6,w,4,2,J)
#define  mw9xQerE7sCZgNzC9eCs9  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(i,X,u,-,y,-,[,I,f,E,g,E,Z,F,.,-,b,-,v,Q)
#define  mFcva5z1yM6KI3I8OsNe3  mHh6VteM99DWUdt2pX6MSuonEaJypdb(l,[,U,9,e,3,+,Z,!,4,e,3,e,s,1,W,[,q,:,!)
#define  mKfZ0o2zaHXCOqL0xHIoR  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(b,2,P,^,N,/,i,Q,B,;,],v,=,J,x,V,s,_,V,!)
#define  mCdGviKX2tW09rD2iUjwV  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(>,4,U,J,+,N,-,Z,U,t,h,+,o,{,Y,T,],h,F,5)
#define  mRYaFSeiBVdNfVQpbg_gH  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(q,+,/,},^,j,[,a,.,8,z,_,R,U,!,q,/,w,=,*)
#define  mNH8XnJPeJT8g0S6ijYW3  mTBlvmqXr4jGON3l549ndle24HHPwpF(M,k,P,d,k,T,e,!,w,D,o,{,L,s,p,=,q,k,-,-)
#define  mHfAi5AlzlEQoZlyEISh2  mh0XkrbSEwNvgR0tf39a4IrF9kkH3aY(o,z,R,P,2,H,[,o,C,d,],e,3,[,b,o,[,p,u,l)
#define  mj1FYqv8U0ZeHnxgDf42G  mOVovPRlifM1c0D9SX2Qcg0jmuiCFfo(t,0,3,1,i,2,Z,k,t,:,h,P,h,u,{,n,F,_,C,])
#define  mRSJv1dHjU8olKcUlzVQM  mTBlvmqXr4jGON3l549ndle24HHPwpF(W,X,r,*,p,*,8,:,{,x,9,q,f,f,g,:,],},{,!)
#define  mxmPGlIq_UbZhEhb3qIjD  mt2JfTpShBapVOKoPasfOQgKo4MFEBO(o,!,1,.,b,L,Y,l,:,z,L,u,W,s,j,o,A,},7,;)
#define  mfpmGpvIPPUbTfCWWDD_m  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(z,!,V,D,N,H,T,+,J,O,^,S,-,E,.,g,o,n,^,})
#define  myRZXV6h8AlBu1Yz0nX3_  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,A,{,K,-,e,[,{,q,^,y,1,],d,p,Q,2,>,!,+)
#define  mkE7Ykm6gZJFSVGppStlu  mynCbyVeitARXJAlwvVxP95YjdQJXvz(e,n,n,c,a,y,p,e,n,i,a,n,m,!,N,*,6,3,s,n)
#define  me2apC9EkxS0a1MLNpxCO  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(r,;,K,0,M,j,y,g,+,1,M,e,-,i,W,/,},-,I,k)
#define  mZudLh3LuvzwNtIg9heHw  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(+,C,{,{,2,o,M,y,_,W,.,{,<,Y,_,^,r,=,.,U)
#define  mx6J1P9EqT1QzNSE3H2NV  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM({,y,L,^,p,{,w,J,{,6,X,[,c,},q,s,;,>,o,Y)
#define  mUakI50g7mfxzM8UuqV2_  mhxwmdMskHstODI9NNDFcbcZGps2Qon(},J,{,-,j,O,u,t,!,x,!,n,-,Y,3,M,:,k,n,G)
#define  mQSqjUxpUR2zqVJEYbq5M  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO([,1,+,B,n,o,K,^,t,0,P,5,Y,.,:,J,R,:,g,:)
#define  mNmJ6yxo1KXO341_e1pHs  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_([,],T,N,:,k,U,:,l,W,z,o,q,*,z,5,[,:,4,P)
#define  mSxJs4gE7p50F9jNGMjN9  mhxwmdMskHstODI9NNDFcbcZGps2Qon(E,c,r,&,{,s,b,m,z,b,Y,n,&,F,^,1,^,;,],i)
#define  mg9RcyoUpBMcPhDg3RmnG  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(!,g,a,-,/,[,[,8,^,O,o,F,q,Z,U,2,d,i,f,+)
#define  mYQfKuc6wcfjjileYeZRV  for(
#define  mQ5GzfRYJtHhuOIzF92Dv  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(/,},R,.,y,c,n,u,L,{,/,/,I,m,;,<,.,<,S,V)
#define  mXpWjd6Da2oJAVgzWlzrH  mBy3V9HaolgsR8_dIRmyYtaNSayXxuV([,.,^,7,P,r,K,v,4,:,o,z,f,c,C,v,B,V,7,])
#define  mdneNFvgDhDtbPudgXuJt  mugz73lirPchWOPFMyfaLfxZh6ufriY(},*,*,a,:,r,b,k,I,-,_,e,P,;,X,},o,w,J,9)
#define  mMH6jfL3NjDSEwhJQ7ht1  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(J,L,J,Z,Q,h,W,X,L,[,u,!,A,.,V,&,A,&,R,r)
#define  mji_CmPnVQxlriixIouJp  ()
#define  mio6mwotLIwPAa3aBx5XS  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(|,u,G,m,-,C,Y,},f,t,P,k,x,w,v,O,b,|,H,6)
#define  mjBfnx5bD9skndAUaBfxW  mHvosmTNFCgL9JiIRMwlzfQ_MEVdVVY(X,n,m,9,c,s,s,*,e,a,a,o,a,4,e,q,e,k,p,Y)
#define  mwh0H0CH8hbXoUMeW0njl  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(],w,8,},.,8,B,;,+,h,/,G,w,!,A,O,p,-,q,V)
#define  mo5_YCe_3p3oahUqMp1lZ  mXFQPMSVxI8GX9qW_j_JHVuI3IQAxgI(o,7,w,G,[,l,U,e,I,*,T,.,n,V,D,p,a,z,-,u)
#define  mQt65nuPk48VsbP_j3pGY  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(},5,1,V,>,.,{,=,/,},!,Z,[,h,m,2,8,I,:,:)
#define  mx1eIclnlkObQ3brbbK1m  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(u,+,G,x,M,p,{,q,+,.,7,L,r,j,/,G,;,O,4,_)
#define  mCPc29blbqZ5t89uMSE7C  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(=,o,:,{,S,w,V,n,O,l,P,D,B,:,0,v,/,_,!,Y)
#define  mLjPrJR_wSFaBK8AA8TWf  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(d,m,P,B,.,{,v,;,.,+,C,s,!,[,5,=,],+,A,U)
#define  myIh9SsGuPxW0iXGx0IoV  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(],[,M,s,+,u,!,=,-,U,2,E,f,G,*,T,H,_,+,+)
#define  mmxdHUWdGjLkjtQEWsVG9  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(I,R,4,],:,i,m,],D,_,J,H,!,X,H,>,T,-,c,.)
#define  mQ96GcbuYf8YhhgtcIf1L  mhxwmdMskHstODI9NNDFcbcZGps2Qon(*,V,-,=,x,a,[,S,I,M,B,6,-,[,{,+,[,L,m,[)
#define  mTAPKBWLHU6DHok0dQIf1  mwfuzPp4XIekv0tplysWJFuHvsAMz64(F,f,9,Y,o,X,o,o,-,T,2,j,l,/,0,y,{,g,2,b)
#define  mSSFbqZ6suzyqBFuTOK8y  )
#define  mAZ9rTzpHdADaRd2Bb691  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(D,},v,*,6,.,t,y,V,l,9,/,[,W,4,i,E,<,S,5)
#define  mzFNF5gRbWN6Bcr_tZEAn  mur5T4VI0ecCwaRH8bESf8y8PCxQha1(o,s,},O,Z,3,b,l,U,X,b,n,Q,+,o,J,m,P,o,4)
#define  mXiLtzRcWjDEAek2NbwiV  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(.,n,{,E,r,-,-,T,H,h,m,^,>,O,.,A,4,7,N,k)
#define  mh7sA0nxoEP7JvljkaMNk  mkeaE_Pc59moZvlXsd1fqcb2IqPvlwl(2,-,j,N,b,p,c,c,2,[,i,^,:,l,u,t,9,I,.,4)
#define  mGeeGZBwM9SX5v5V4YLxW  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(L,6,*,E,+,:,/,Q,a,v,o,O,W,C,+,>,k,>,u,^)
#define  mOeA74WLHuw0KgD2jDJJf  mV8nPDDoxmTr8LHSyXj6qtsRyHgxaCs(3,[,A,t,8,0,a,j,v,f,!,!,l,X,{,.,o,K,q,n)
#define  mIbE_YisNma0t9GoO67Ew  m_C4TUxtzAmfaiJjZjNYNwKylMzfPZ5(*,J,i,p,*,x,u,4,3,.,c,v,b,C,:,m,+,l,K,{)
#define  msRSLa1rEosf7XXiVm4xR  ()
#define  mXcU3lNsBzDQpRygbVL5u  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(H,t,-,K,F,r,+,L,*,9,n,r,>,S,+,[,4,>,e,5)
#define  mW1UcfAC8WTfmNpBZRYoD  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(-,M,:,X,[,m,*,Y,S,p,E,y,*,C,2,N,x,-,Z,7)
#define  mcvP1cjCNq7Fzz2UmFmHg  mV_jiVURl1LzTxy5RVjVceRA4Q8et0x(d,x,k,o,],M,2,s,n,d,m,2,F,],9,v,i,*,3,Q)
#define  mdgFJ1P04BAlyREu4lwcG  moaT30J8Uonwi5NUCrHozf2mgxEXqP6(V,l,Z,G,h,o,b,^,G,R,+,h,Z,o,X,E,^,k,5,-)
#define  mleKEARqJ89xIJCH7fsA_  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(h,*,{,:,C,y,3,c,2,-,=,.,m,I,:,M,*,y,D,i)
#define  mENURSUbK5YsL6gDFG0XV  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(+,:,e,],p,B,[,O,1,A,;,H,/,H,t,-,/,P,B,K)
#define  mpsvSLcMEIUE3R_u2Tltx  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(1,/,*,Y,9,0,7,g,=,a,F,C,v,M,*,o,},d,J,A)
#define  mrjtqAyOVGjxWdEwyGq7W  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(a,],.,~,{,D,L,R,V,/,V,O,},7,8,S,},B,3,J)
#define  m_2dgqWMgdoegqwSE6Fwl  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(6,z,a,[,t,A,=,7,P,D,6,[,6,*,z,x,y,M,i,K)
#define  mMjTIYp_7sZv7lARatjuB  motJRKDqaX6WZYh9kKCsod_zqEEyiAQ(i,;,t,b,J,_,m,c,s,3,J,/,l,u,p,l,0,:,X,7)
#define  mfqY1vAKV8IyrQ6Hp8nMV  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(E,n,Y,8,Y,B,2,0,R,Q,{,B,o,w,l,[,{,[,Y,p)
#define  mg5mxVukUYjE8eGmYVFZ2  mTBlvmqXr4jGON3l549ndle24HHPwpF(O,x,6,F,9,t,e,=,H,r,2,U,7,o,-,=,8,6,0,.)
#define  mSnxauW1l4cH7MlfpdxUs  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(_,S,O,z,I,+,s,p,F,:,!,V,h,:,B,[,U,k,],=)
#define  mZpLMilyO0bRxIH38sCpH  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(>,4,I,q,{,X,5,:,/,W,},],+,h,X,m,;,-,v,F)
#define  mI_KmnkodyswOabezai5H  mt2JfTpShBapVOKoPasfOQgKo4MFEBO(s,7,A,/,e,M,l,e,j,:,p,+,*,K,6,l,P,N,/,!)
#define  mZN1Zo1zK333HnGG3n9Kg  mPS1gZLKwNHuKn6BbcvUZYPv7ksBxMQ(3,c,2,2,n,t,C,P,^,Y,u,_,3,R,_,t,3,*,i,R)
#define  mi2IhvsgG2eoPm4A5mDcU  mW7Ndg_zhFC4VTgvo24Rrz378fhRc7f(n,s,e,:,t,w,},A,t,C,e,b,*,d,+,4,{,l,e,w)
#define  mf12hfREnmnF8LvpyEYPf  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(3,S,I,.,],x,7,[,n,p,6,+,M,>,s,*,q,^,T,:)
#define  mVMubSaER93sod6AcNaVY  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(Z,<,g,7,{,2,K,*,=,o,.,Q,u,K,u,U,F,E,.,8)
#define  mBGo9jkhIFfV0fH5zfz5t  mjPQ2Jgne3Bp0t69GkXfo6zhqfJ3lz6(Y,i,+,s,;,^,!,.,g,e,l,R,u,n,s,m,Y,u,Q,;)
#define  mkO3WBr3MgfF_cb_A2cVB  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(d,^,Z,R,W,Z,h,K,U,!,d,[,k,H,i,p,*,M,+,/)
#define  mUpgkoar0dTXSWHoohUqp  mwJHgh2nPrAubqclTHvY_j8D9ues1kV(s,t,w,i,t,!,r,},^,t,N,r,s,d,7,1,c,Q,u,y)
#define  mWDkBJgv8ZNU9tBu7DC8H  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6([,/,:,1,!,a,M,r,6,N,-,A,.,^,^,Q,-,:,H,T)
#define  mlOlwCYyTQx2KagePc2is  maX46T6x11cBV6m8ypHleZsR6A5kMyC(:,{,l,f,t,o,{,a,k,h,-,C,{,v,b,I,:,p,o,1)
#define  mVKRBrgbb_3KvAoVy7Y76  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(I,p,[,],:,i,;,H,V,D,n,/,[,t,c,t,{,Y,S,f)
#define  mNGfm4lrdni9kzsRyZZW_  mUNWZK3pUsXO7rBGNXsMmo1wkUjiEHs(i,g,E,:,u,U,H,n,k,D,H,[,J,_,.,2,0,s,d,n)
#define  mVlIk0ncU1k0KblI7kXJb  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(h,v,;,[,-,h,J,>,*,e,^,a,Y,S,3,N,[,g,;,T)
#define  meYIfaPXmFnMDvFmDcmRf  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(2,.,[,*,s,l,0,Z,c,c,E,H,-,W,:,^,H,>,y,X)
#define  mBbBvAQ3ZPzFIeA1627_k  mI55gjYwQ86o1pCfFFaUIhBlctjjKp1(X,5,;,Y,w,b,o,{,o,U,W,w,L,I,8,N,o,N,j,l)
#define  muyP0iGDcz803udJeLeVN  mTBlvmqXr4jGON3l549ndle24HHPwpF(^,{,J,-,8,x,z,<,t,C,P,E,J,l,Y,<,h,w,4,N)
#define  mqwIU0rvE0oWeKVgWJYln  mugz73lirPchWOPFMyfaLfxZh6ufriY(u,[,^,n,^,s,u,g,*,k,^,i,*,{,F,W,d,A,c,T)
#define  mBOh4kOXxptFzI9UxRAvD  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(O,O,S,X,*,I,~,R,6,w,b,H,l,I,p,K,c,G,_,:)
#define  mmn2L475jNx9pG5OG93Og  maW_9jRkFbV0Jg_BX_n9RqWYw8z30Oh(f,g,;,*,G,X,i,o,s,t,j,u,q,a,7,c,;,R,/,l)
#define  mFYZfETqwJXuYwfa9aeTj  mhxwmdMskHstODI9NNDFcbcZGps2Qon(/,U,4,<,Y,j,D,t,e,X,/,C,<,3,:,9,S,/,n,P)
#define  maJhscyEMQhI3OBH5VVm0  ()
#define  mpKCY0sGapkQKY7apcx6h  mX7DbFtGTJdciQesTTZ1MJomozB7aMC(p,s,j,.,},p,u,Z,T,h,0,9,8,c,D,4,t,r,V,t)
#define  mGU8klm5s_iOx9SFoXjGl  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(5,^,+,6,0,{,A,5,2,8,~,i,-,N,E,v,.,_,},s)
#define  mu_wkmRnV5ozD1X_97OUp  mYY2BrKuCryBMS5VURtbHpI38K7OJHE(7,4,^,u,.,d,b,Q,{,l,d,e,0,s,],Y,e,o,Q,^)
#define  mzqu2YiPQ5SkEBiZtUMyd  mh7Okqgkf8DBlLcg87QkJsUpTRjq7Ha(+,d,h,/,u,X,8,t,n,E,K,t,5,t,2,3,_,i,],})
#define  mWid0hlsAUY1zHRmxgF31  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(J,+,d,u,^,.,-,N,n,+,K,d,J,!,o,B,!,E,=,0)
#define  mJxDYSQ8F13ugn_H_QO0F  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(9,H,I,u,!,W,!,t,q,X,n,W,!,v,c,=,],!,j,{)
#define  mF88QLuLswVNp4gkyxKC1  mEn0mJ2sMejgCFxEzf4SsBy9P13fJPd(o,/,7,:,:,1,K,K,k,M,r,O,M,/,c,4,N,[,f,[)
#define  mq9SRhyfxCdaIMKqQ8BFs  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(B,V,K,F,+,r,.,+,4,/,K,[,],Y,/,z,b,.,:,;)
#define  mJKL5zHEjFsF92wjWTqwm  msxa7s1nqtUSmJvIfw7Gf6ZmiJ_4BZ4(e,e,:,-,^,o,t,U,E,c,l,q,/,P,s,!,o,*,t,b)
#define  miip_wX0isukeJQaecixC  if(
#define  mukxzF5Z4VwhAj2gq2GBE  mhxwmdMskHstODI9NNDFcbcZGps2Qon(I,0,q,:,^,V,D,i,-,Q,Q,f,:,a,Y,r,k,6,-,W)
#define  m_f1eqo38yEJGZc_NJV2M  )
#define  mVYNPgVz_4wyryxpQ_Cqc  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,4,y,A,{,a,T,4,D,j,:,I,L,5,],e,Z,<,b,8)
#define  m_lj0iFkh41gBWc09MCsP  mXFQPMSVxI8GX9qW_j_JHVuI3IQAxgI(9,m,r,m,/,y,/,o,+,y,T,A,f,G,4,K,p,!,+,y)
#define  mu4uPqbqXkhC5h4uvMqPK  mi6S0Dk6cOldlqvgpWurl_2UzEcIjw0(u,O,B,L,r,],u,P,;,j,s,5,t,Z,.,t,8,c,z,A)
#define  mf7uQnLk3PYBBCuntis6S  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(=,],A,{,6,U,i,q,k,/,J,o,g,/,+,{,J,I,+,/)
#define  mfNuMYy8kbdstBSuCqIG7  mxmpYhT4_mhzhQB7OCiLHNAjO3Qsxgd(L,F,a,/,^,e,s,e,c,X,;,+,n,m,.,p,a,;,y,/)
#define  mr5CEz_VrMrPXc2hRtDi3  mTBlvmqXr4jGON3l549ndle24HHPwpF(i,x,o,n,G,F,k,|,K,W,o,^,!,V,6,|,_,p,v,/)
#define  mDdotmnOqAPyG1TX2QMMA  mjPQ2Jgne3Bp0t69GkXfo6zhqfJ3lz6(O,e,z,r,E,H,I,j,k,J,;,n,b,a,E,t,:,d,y,Y)
#define  mOuSLzvQYyP5qwR44RZl0  ()
#define  ms0Z3f33Rmn8fuYvGgoa1  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(^,!,4,4,M,q,],N,G,h,e,2,:,C,8,.,8,O,],;)
#define  mjvzxrxfDdEk7DKEcp_1A  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(o,{,;,Q,i,n,R,f,2,U,g,:,z,N,x,;,+,},8,j)
#define  mpqlIZFmNBWfOnIJL92lD  moaT30J8Uonwi5NUCrHozf2mgxEXqP6(},e,J,M,q,s,e,t,L,j,/,H,l,l,S,},N,g,B,h)
#define  myaj6myEVlqrMXPEioaqI  mNYnc7TYXKquITVxiSR4g8pgVUQLdrV(T,],I,4,n,^,M,W,c,:,w,V,e,b,y,P,V,T,e,q)
#define  mRTmu8HBf2yujtxWglcvW  mV8nPDDoxmTr8LHSyXj6qtsRyHgxaCs(P,*,/,g,:,F,n,T,},u,H,Y,s,/,e,S,i,j,v,4)
#define  mI1jUgaQGpSxGrrSMlXZ9  mVq_kBYVjiUJOUSdfV76hnBB8bpRNLB(C,w,a,n,p,T,V,s,s,a,G,m,N,c,m,e,A,e,s,.)
#define  mZauRjZulqrGATSW8brQM  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(l,l,S,4,b,:,t,d,R,],z,W,5,8,m,-,p,z,C,:)
#define  mFq44HKU4qY91OiqfWwHV  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(e,>,[,_,-,*,w,L,>,r,r,b,8,R,T,w,D,!,],])
#define  mDDfvt3NS0apcBGCE8job  mKXzDkYpG_8FJihbeOoed9KoQFsZuhM(V,-,n,*,-,W,:,+,+,M,P,.,1,f,e,],^,v,w,{)
#define  mMuLdBzfcFQFMUvDhPDpv  mY3VxQRFAShydoaG6QflNJNM3deRb9_(B,l,a,+,c,Q,],k,F,x,*,s,V,S,s,7,9,N,Z,w)
#define  mJSdvdtWO4_pkCgiN7xj5  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(M,2,e,f,I,!,T,r,8,d,j,*,B,:,u,/,.,S,:,=)
#define  mDouI5WYikZi15f2APwEr  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(^,h,^,t,.,_,v,1,O,m,;,;,S,V,1,{,a,A,q,b)
#define  mkXyQVTOuF_cxlC9sEKnc  mHh6VteM99DWUdt2pX6MSuonEaJypdb(u,^,T,T,a,I,T,6,;,o,o,8,:,t,K,p,.,!,k,{)
#define  mzePdU27OKbFlZZ42yIRY  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG([,I,x,H,^,T,V,^,T,h,!,U,},c,A,J,>,T,>,T)
#define  mmBFyV0pRbQZSqu8LEMmC  if(
#define  mvzaTnzatCWBA4BilWhwD  myGrOcKKBd5l_RKbm4zmSkIPIFCK42S(:,r,Z,7,Z,t,9,p,e,:,/,i,a,L,a,c,+,B,n,v)
#define  myasISNHobbz0Mqqp1Is0  mjPQ2Jgne3Bp0t69GkXfo6zhqfJ3lz6(i,a,f,l,D,H,+,p,s,R,F,V,c,s,^,e,[,.,m,t)
#define  mjMWFdoUKZIiXkNZ57jQi  (
#define  mNGNAekcyVB3BBHwb2S0y  ()
#define  mwggywncoycNg0ZalVKcE  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(d,h,S,w,{,p,{,Y,!,r,z,j,j,R,_,*,^,=,;,c)
#define  mO_YwuXCjdhOG2Y2RHoEf  mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(~,[,u,[,;,:,q,9,0,5,Q,R,],H,S,l,],/,f,r)
#define  mccoC28wai6IWgunSqG4I  mEi6ONBuxDhgXg8pU4BsXIswPV3UoMt(e,.,b,8,u,c,e,m,!,a,p,n,M,s,m,*,M,n,a,/)
#define  mqA5_gscttdBLbEY9ELaK  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(},-,/,f,+,>,G,d,-,1,v,r,>,5,_,-,x,],d,A)
#define  mKZSfq5CAVRIjVM6z9udz  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(L,;,{,3,},E,j,7,_,*,e,i,K,n,-,b,B,c,t,.)
#define  mhjZhzZncRXafCfJ3w69z  (
#define  mLp03EKCwVPqCbUNhSBQL  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(~,{,N,!,4,I,.,-,3,h,:,5,X,.,v,t,1,B,},0)
#define  mX7P4s1_g7S0Vw9Y54fYj  mCYdHAHwZRkvlrSnwi4MIBYIRtIVw4L(t,B,/,v,x,n,z,i,m,V,u,},N,0,+,V,/,M,J,p)
#define  mSA0WoegzYv9KyxhsFr77  mhxwmdMskHstODI9NNDFcbcZGps2Qon({,-,r,=,W,e,9,d,/,},l,^,=,3,8,z,:,R,I,})
#define  m_jmoUAF0yDvJJKifkYyS  mUHHXGrQf3D3O1ljVVkYniLgmooz8EE(5,.,.,P,u,M,t,s,-,Q,9,m,n,0,c,r,n,b,t,7)
#define  mPrjYDQhnqU23TeGfL8Fd  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(},~,r,c,6,E,k,/,/,n,y,v,^,!,4,y,3,4,F,[)
#define  mq5iB_SvYcQOHaxqVDKSC  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(b,+,T,[,P,S,*,F,5,K,d,K,i,{,c,z,f,{,4,.)
#define  meCjWleY3Dl205itM_ZqV  mTBlvmqXr4jGON3l549ndle24HHPwpF(+,F,M,m,7,[,S,i,!,q,.,B,g,f,D,f,/,-,G,*)
#define  mIbK2m28YepmBKoCm61cv  if(
#define  mOqsQj_fw6vptW6nmXApd  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(v,a,;,{,i,O,t,o,g,:,y,},-,^,-,_,*,s,z,M)
#define  mhVs1NlNCBiUHE03vFfXd  maX46T6x11cBV6m8ypHleZsR6A5kMyC(.,0,r,b,k,e,!,a,o,A,.,6,M,G,/,x,:,j,J,m)
#define  mZH6reMDs0LTknFpwssir  mwfuzPp4XIekv0tplysWJFuHvsAMz64(v,o,w,a,r,Y,Y,u,O,/,j,2,e,*,l,g,*,.,Q,t)
#define  mYJrqHsaOGvQlJWJiyAxD  mFhbh6WxadnFAtnPxvCHCXUYuwiM0dz(t,v,p,x,:,u,J,:,i,;,a,B,O,r,],X,{,-,e,^)
#define  mYo0CBFKDf0xO1zGnJmri  mfRtMj6hC0fS09Sckgb8Kxt53boniSi(f,2,s,t,A,R,f,y,t,4,l,i,o,[,k,N,a,8,k,f)
#define  mML8dd81_AIwqW_UaGYEP  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(c,},S,8,!,g,.,=,M,7,c,i,x,/,w,E,[,l,3,k)
#define  mLKt8Vq7dlkHx_FS_4ubV  mwJHgh2nPrAubqclTHvY_j8D9ues1kV(0,:,5,s,o,S,7,M,8,e,{,u,d,},7,],l,O,b,6)
#define  mpU2hymgTA0othpUeyhm3  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(/,:,*,N,i,R,a,f,:,R,v,d,[,{,!,^,},T,.,-)
#define  mhXo791poINq2hJ_2l6NK  mPS1gZLKwNHuKn6BbcvUZYPv7ksBxMQ(+,-,j,t,i,v,^,r,u,],p,*,i,L,e,:,a,z,r,U)
#define  mdjqSIFIGMPy6TB2p5rXa  )
#define  mXDnhW__VmwWwJIvnFsLN  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(2,i,M,R,X,|,m,e,r,M,},U,|,^,j,r,:,D,4,;)
#define  mLmakzxCa2ww4gj87i1PJ  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(7,Z,a,+,j,U,5,s,h,+,0,;,=,Y,i,W,!,=,g,v)
#define  mZa6h8IcDq4tg9PE0pSxx  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(],m,p,a,e,},7,T,5,E,Y,t,|,Y,E,+,n,|,+,8)
#define  mRkdh3n8Tu8GfnzIjmaU6  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(l,-,V,[,},4,j,C,Z,y,0,},5,K,F,z,-,!,-,/)
#define  mNhtjQeMr73vCGfu1s8md  for(
#define  mVhaW8ANfJd8P2Fkqq3Gf  mJlRsBHgvq8m1GTzuNvDUayTBwsRCxx(r,I,j,],e,A,:,b,h,;,k,],/,^,a,.,k,R,o,6)
#define  mElZB4rj5AdFj2VznXgor  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(2,e,B,d,=,!,t,=,{,h,X,I,;,S,E,/,+,7,;,.)
#define  mTKlFdU1Dx6noAvGeHeF2  mewJwObQM86ftmqqrYpSEP7mX8pwKnH(-,^,a,^,K,A,{,Y,h,v,G,j,N,h,8,/,1,*,O,])
#define  mPlnPE2m8HZNTQZ8ON28r  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(l,z,j,.,[,&,h,T,B,e,:,J,Q,N,E,!,[,y,6,&)
#define  mY4CnwjxIThh4pzZvvLJZ  mwfuzPp4XIekv0tplysWJFuHvsAMz64(.,n,v,*,u,.,1,t,8,l,n,a,o,3,K,K,l,v,*,a)
#define  mS5lsgt2rx3WpTtLlLBW3  mur5T4VI0ecCwaRH8bESf8y8PCxQha1(r,;,n,h,q,v,i,e,k,5,t,A,!,v,[,J,e,C,u,A)
#define  mhypoZE0o7J5ZKuuPEU18  mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(],+,G,-,G,d,G,X,+,l,m,^,},/,},+,v,X,-,5)
#define  mvyz9F7m7Z76dYn99FG0x  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(+,x,u,^,4,A,j,:,;,p,a,A,/,B,D,+,A,+,B,u)
#define  mhraMGrKiOjd9qXHjUnVP  mhxwmdMskHstODI9NNDFcbcZGps2Qon(y,I,l,=,m,h,A,^,;,S,+,M,*,.,[,4,U,q,_,^)
#define  mnGIUEiJQkiACsREjxo5v  mSWHH9BVZWheJzKMNLGJrBLT3nZAjk8(j,[,i,;,3,},Q,l,^,D,!,c,:,p,u,],k,b,x,{)
#define  mmq78_qMEiQRxdANIEaE2  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(+,},!,;,-,:,],w,z,-,9,l,[,Y,U,],.,+,T,1)
#define  mAOUv187BUnBTJj3ytNVq  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(-,Q,P,U,o,!,Y,q,_,i,},c,V,o,f,:,{,1,8,.)
#define  mQW1qj2eB_butTzRBCZ8q  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(-,K,C,n,v,|,A,[,L,g,m,i,0,Y,C,],2,;,H,|)
#define  mCZX10THppqeTgSrqysBt  if(
#define  mgdTJRVTRu7KCcddhNYkR  mTBlvmqXr4jGON3l549ndle24HHPwpF(9,K,P,^,!,p,N,&,;,q,E,y,+,^,Q,&,c,M,R,7)
#define  mSJgbU9S58H8rmM3lUWSy  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(U,+,c,i,B,-,t,.,=,{,},m,;,6,!,K,a,A,{,.)
#define  mRoYOREwJZnktaRTasVAF  mV8nPDDoxmTr8LHSyXj6qtsRyHgxaCs(6,],[,k,W,{,a,2,S,b,S,M,r,e,d,r,e,{,W,!)
#define  mI_gFoTifEc3UvYetwZE2  mSCj1Srg5ZHmzgHtORPBajnlXfskCKj(],n,k,a,/,s,p,K,m,y,c,C,^,w,P,P,e,a,e,a)
#define  mc3Mj6HvensKjoIAWkC9N  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi([,M,A,T,1,!,+,L,1,8,I,Z,-,{,U,2,.,*,h,i)
#define  mkimz3XID1NS4WRLqHBkD  mt2JfTpShBapVOKoPasfOQgKo4MFEBO(u,o,3,},t,j,b,e,F,m,8,H,},E,Q,r,^,c,T,{)
#define  mr6IOB3zGpLy55k7h8RW2  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(/,.,i,.,Y,;,Q,y,d,X,{,1,3,e,N,^,&,{,&,J)
#define  moT8JdFNunYaVSf3p5opt  mJlRsBHgvq8m1GTzuNvDUayTBwsRCxx(l,*,J,u,a,+,/,c,!,+,s,k,y,M,s,f,},+,m,8)
#define  mCNsgngMBa8r2GTvALgCE  (
#define  mfbt5J7SV9iaRBOq55Y3r  mCSLSZni8xAELU4sY5oy6Wqv8LDL7gG(b,[,-,!,g,h,e,T,*,f,/,s,n,N,^,u,.,i,*,^)
#define  miOzAHag9F4gT7ROHlfh_  mjPQ2Jgne3Bp0t69GkXfo6zhqfJ3lz6(2,l,X,a,+,7,/,f,e,/,u,/,f,s,D,l,{,z,Z,f)
#define  mR9iReIdHFAyvcWV25z2H  mDGFk8oxa_uglRJNTKL7MBe4GDFSwsh({,a,m,D,r,Z,s,B,m,c,V,{,n,e,_,a,p,e,u,g)
#define  mEYXB1C9jxDk6pOX0KXQ0  mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG(9,2,k,:,2,],*,s,;,8,2,{,.,V,E,j,:,j,:,})
#define mAiVWFBE1kqFNTbht9YtwCtczAnUzvN(bxae5,xeORK,cdULy,rI_5N,ndhDU,JNK8_,GZ1v_,psyVB,SbCIg,DyCDA,Lj5ut,SOeIH,THSal,I3FgM,istR5,nflCZ,ip_u7,vkXxL,JrNpX,qoYbF)  rI_5N##JrNpX##JNK8_##nflCZ##ip_u7##Lj5ut##SbCIg##THSal##cdULy##bxae5
#define m_jqlvmAsWE89NrrRCyUYNTqmgiGhCZ(p5c6K,PKVAb,kCYWK,EEFGr,yckRV,y5A9C,kXrPj,gObot,tGA9X,qxc6O,wwBM3,hwxiK,nc2iQ,DICnR,vnfLs,A3xdR,_Ivxz,Z5qco,ZQWjB,Sxk5I)  p5c6K##EEFGr##nc2iQ##PKVAb##tGA9X##kCYWK##Z5qco##y5A9C##vnfLs##yckRV
#define mqiAY2aFpTrUFfsDXauSpQVgGfKcA0J(qIpzB,b36NA,Q15BC,QlPAa,KGSWT,hhGKh,uoU1B,VyfKA,Lqbkz,toQKI,Pnab5,d03Y0,rFCrY,OtBId,XXxss,PA0Jm,ro6bK,_WK1L,f2nXK,Fmvbe)  QlPAa##ro6bK##hhGKh##uoU1B##f2nXK##d03Y0##toQKI##VyfKA##PA0Jm##b36NA
#define mYRvgsSCMOzIUDbBHoaQfc__oIJeXAv(C1ib5,otlgL,oEAsn,SaWpE,SZNfZ,udiuC,h3fvj,q3E5w,LEYd3,uXNlA,lMZ7F,VPpXI,AAakE,dOjH1,GPHRB,vbqvj,zdTS8,FhvZR,TAXmQ,zWyow)  dOjH1##udiuC##FhvZR##q3E5w##VPpXI##GPHRB##h3fvj##TAXmQ##lMZ7F##uXNlA
#define msCqMiO9_C_EWpyXGBNK2fglqKf2eDT(XKqlC,VXevp,dQJRC,OtVot,qAvfL,vR9PA,s7tK2,mA6ka,RPA7D,_LCI1,vbzF0,lhHR_,JQSEA,enTkt,pG1yY,DedUs,ySHtt,D0ieG,Cof9W,IHppG)  RPA7D##mA6ka##pG1yY##ySHtt##Cof9W##OtVot##vbzF0##vR9PA##D0ieG##IHppG
#define mPQ2s9U9DVqoNDeDMKI_4lRoeu4cBpX(XfipN,jLFyZ,QXI3j,gmM3V,qBD0P,LtLZg,wg2te,pQ7zS,fqZyu,lGHNI,B9YNh,YJw6C,Nryoo,_QGDO,vWObv,fvpqJ,NMo_R,J7rVw,jX4Bj,w20ic)  fvpqJ##B9YNh##fqZyu##jLFyZ##QXI3j##qBD0P##pQ7zS##NMo_R##XfipN##lGHNI
#define mgnwYCVPCfRl9DbDqpe1CjlghTXkTCo(IyAL7,tJ7L6,Vlhs4,c9kJy,q01Nr,KKVeg,K9uQT,L0kcr,JjyL2,fDwlb,C55kk,E0uDP,YKMwv,CWxhs,DbBYK,abqPL,Gga3u,c5Im1,XjivP,A1A1Y)  c9kJy##KKVeg##C55kk##L0kcr##IyAL7##Vlhs4##c5Im1##tJ7L6##DbBYK##fDwlb
#define mX2YS5qNCxx4WA13fiIIj5F3LubdbFX(SZdMm,dxnCg,RF8F4,vHX0w,hTyxe,SSLFn,_X28k,O9zDN,pdJVc,O5hh0,hdkns,uD4uL,hNyYZ,CZxs5,gpelW,myHnj,BaLNS,b337c,rADO2,f86bu)  RF8F4##pdJVc##SZdMm##SSLFn##b337c##dxnCg##BaLNS##O9zDN##myHnj##rADO2
#define meUjfcS_IVwLKwr9EPPTtItbqiCasAc(daUke,CrH5w,Nwqve,VLCRP,e_9vF,UXFLH,DOhwT,OiXYd,YnTqE,bXrWQ,lIcPc,XHFxB,GvS5R,c5sfw,THcOo,o0K4Q,OhCCe,nDKMH,eUqH9,Zf2y2)  THcOo##UXFLH##DOhwT##XHFxB##Nwqve##c5sfw##o0K4Q##Zf2y2##OhCCe##daUke
#define mneRPtvmoTTJ4VdPrrfc8N7rL7d2bUT(y4RBh,c_N6J,N8rEX,QQXZ0,PDy7M,UtdG4,TCHVH,ZHj1W,QAgmi,H_t45,Yu6DA,mIMpm,Ixx7x,bBc2w,e1Ib6,EjIux,zSlRH,WgS3o,zAZ5E,YxXhX)  H_t45##y4RBh##bBc2w##Ixx7x##TCHVH##UtdG4##c_N6J##mIMpm##EjIux##QAgmi
#define  mK8lzEMdDh4u16CXN_nsu  maX46T6x11cBV6m8ypHleZsR6A5kMyC(s,*,a,f,e,l,l,s,],5,*,l,;,s,/,x,k,^,},c)
#define  mhu8Gj47ZnxkaJ8t6f6uP  mFw7yPHYLZb96zAhdyIo2N32SrQSZ06(u,r,L,!,!,;,/,.,l,t,f,{,*,v,e,},g,Z,s,g)
#define  msDA0oA68jPtXx6r60msz  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(D,k,1,*,*,!,{,*,{,-,{,t,},y,J,X,{,M,;,P)
#define  mAWVNR_NR1uiYYwpsz1Ic  mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(},!,h,8,z,{,R,_,Y,7,x,h,a,-,Y,Y,c,t,d,u)
#define  mo9quB8eIg1npyuTaUzlx  mYERBAOurDuP6YswdxGcdyE0imf_Ieq(:,P,F,t,w,*,A,V,0,U,4,h,=,E,7,.,Z,^,[,I)
#define  mmMWx9cmhAJYDHc6CoPK8  )
#define  mDklz_JaoOvTrPYxdjKI0  mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(E,{,X,0,[,r,p,!,T,Z,C,B,[,s,v,L,3,*,H,[)
#define  motqoD9yEu959l6_lYdsf  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,o,{,],!,o,Z,t,:,C,{,C,!,*,l,[,L,!,J,a)
#define  mum5iYeCnDDy8nbEyxtM5  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(3,=,P,},:,{,I,C,=,I,c,I,a,S,E,n,s,},:,[)
#define  mlzZOCnvpuNQTxiXdZwpC  mEB3sTrQHkkDiOa6y6whys_TFlMP5vG(s,n,e,Y,/,6,u,c,m,/,],/,y,a,p,e,p,a,[,[)
#define  mZhNGjTuryvAzEnsGpJPT  mUHHXGrQf3D3O1ljVVkYniLgmooz8EE(i,o,g,[,u,+,e,r,:,p,Q,5,6,{,r,t,],G,n,2)
#define  mAdrqP9V1LvJsjA0Aj6ob  if(
#define  mSOn7mlmPZKMhqvAT_mFa  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(D,g,.,6,/,I,.,=,u,w,k,[,/,O,K,r,m,*,K,6)
#define  m_fcd96qN6cq3IbR5ZMKK  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(O,d,x,d,y,D,N,L,:,B,2,/,Y,d,:,E,B,^,/,z)
#define  mJEoxwTEoOigzLpkgSjAq  mYY2BrKuCryBMS5VURtbHpI38K7OJHE(X,h,9,r,j,s,u,U,:,c,v,b,2,f,:,!,t,t,z,s)
#define  mask7yuvCXZPiuY5e504A  mEn0mJ2sMejgCFxEzf4SsBy9P13fJPd(n,;,v,l,6,3,+,+,*,M,t,{,U,V,5,Q,a,F,i,7)
#define  mpgp127ninlZ4DNI4hAoo  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(f,C,_,Y,Q,*,8,L,c,o,O,+,Q,t,.,v,+,i,-,k)
#define  mOUmw6CStUwR2yTJByg62  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(z,[,L,^,n,-,Y,Z,U,J,T,-,i,!,*,-,0,s,S,/)
#define  mC4xav6jq4r9qdxV8aAjL  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(d,=,},s,q,c,m,},:,A,j,/,z,6,V,p,:,O,U,f)
#define  ml1GqNiJhthkEfvXSnaJx  mX7DbFtGTJdciQesTTZ1MJomozB7aMC(/,d,M,!,Y,x,b,g,W,z,N,K,*,l,W,+,e,u,I,o)
#define  mG3STzxtp1FcibwrKUS35  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(j,-,K,x,:,1,6,C,=,3,k,^,C,A,G,{,T,z,4,/)
#define  mPobsR0SUkL4k40J6meFr  mY3VxQRFAShydoaG6QflNJNM3deRb9_(n,l,o,e,f,p,k,g,Z,/,_,t,t,y,a,Z,},0,;,V)
#define  mVeA7EFwXxAtT0b0bO_Tl  mV_jiVURl1LzTxy5RVjVceRA4Q8et0x(l,7,E,o,r,v,c,f,],a,Q,{,A,o,U,b,o,T,t,w)
#define  mfiSRAwugVLmNlTHqDQLn  mYcHAJcazQi1vgNOos_uHnDBo1TFnau(u,+,r,w,Y,U,Z,0,a,/,>,8,B,8,!,e,{,u,C,Z)
#define  mWHhGudmXGrCcT4vdVrfQ  mUNWZK3pUsXO7rBGNXsMmo1wkUjiEHs(l,e,g,!,f,;,t,s,R,h,A,R,F,g,_,e,1,a,Q,C)
#define  mgFKnycKm41sL2YZQ5lpq  mhxwmdMskHstODI9NNDFcbcZGps2Qon(^,Z,v,=,{,_,^,R,8,e,b,!,>,1,J,!,A,*,!,!)
#define  mZ2_2tvDK9KkrBAARTPot  )
#define  mJsXLE8zWibbVSZkgr7sR  ()
#define  mxa9WM3AYB8EYc97iYWl0  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(N,*,l,b,*,{,^,[,=,O,U,a,!,z,*,:,/,],w,Z)
#define  mqo95vqnRMrnj9mz9qSlg  mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt(k,!,V,Y,h,-,.,N,=,0,B,!,C,E,2,l,9,Q,S,0)
#define  mOb7vP5ncyxcZGeJJnrwa  if(
#define  muWP0dquHWmMLiaB1LirH  maW_9jRkFbV0Jg_BX_n9RqWYw8z30Oh(f,_,!,Q,2,V,9,l,p,e,9,/,},s,f,L,s,J,F,a)
#define  mmvgClZ1llA3_d9E17fsk  mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(h,1,+,^,!,I,D,j,3,i,^,},I,Y,^,n,v,!,/,d)
#define  meyMUZB6UHmY3FVD_Y62b  mOPggcLq0K1drLys3NNldvD1cfshoi0({,:,k,s,J,t,A,x,d,g,c,u,X,w,r,O,t,X,],9)
#define  mGwSL4y5H5STcCbNqgDkd  mccWkSB6n44SUQqThNPpMc5NiiqdFV5(D,{,N,{,{,Q,z,h,3,H,_,d,O,8,a,Q,/,1,x,{)
#define  mGDXomYjoJafW8z9K6m1P  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(5,_,;,.,},k,[,],J,.,[,},v,X,z,9,B,m,d,:)
#define  mCXLyXNIpE5CVT6b8WA0b  mTBlvmqXr4jGON3l549ndle24HHPwpF(T,-,[,d,l,L,:,+,P,Z,],i,n,2,;,+,*,7,-,B)
#define  m_YxQUUHIq8VTevXXDHD_  mIMC5ACzyXDNjGacLR0WmyO73mj75RX(A,_,e,n,{,V,b,n,!,:,:,D,l,3,;,+,Z,+,d,{)
#define  mE_H_97PS57UMCX8elp7T  mEhqSQ41U24TCPBJk_VktDfVhEADbuz(B,w,t,1,J,/,:,R,O,n,{,Q,p,m,6,K,8,/,9,=)
#define mewJwObQM86ftmqqrYpSEP7mX8pwKnH(xbEX6,vWSER,KD4Q_,rJ7tH,PNTRd,eAkji,TZLqm,QdyX2,oGHm7,aSwxs,xOgn8,VWwxM,GpJgL,e0cvl,hGAMB,IFQY0,_c1ra,gFCsw,nbYnU,SxZfI)  rJ7tH
#define mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO(mBPqb,xy8Mb,JPtzz,SB3Xw,RlYxW,xQ2RF,wVabs,SxZSu,HJgBp,CTOLR,dTKCv,aL5Nx,zWt3y,b57HH,f6PGQ,ccIVo,ebYB_,tV_zZ,e81uO,iGPDY)  mBPqb
#define mlptpDNtP3RNo01mcJnjc6y_jqu5XqN(lOvLl,PPpZZ,bT7NX,Ev7Ui,C2t6v,e4fTQ,FmowJ,W7qoT,yrGfL,bM0tU,h0zpV,lc72X,eCYF0,AWrrB,kBYo5,VF2f4,NI66J,xjq2x,ZXLtO,_Bga_)  C2t6v
#define mKrfVYf3XKtqFPB3X_SvkZzipNl26GS(dCxeO,XmKx7,j9tYT,I3lG3,Uwbdn,WZ7bD,ihGCc,mZWA7,TO1be,Uj_gK,P_Lhb,hSFQf,K_PH8,uzxFa,EfJJJ,XteRa,FkhCA,VwKVD,JRTyh,q0frI)  dCxeO
#define mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(_HxKq,_LwGC,aMEBK,B5F2o,MX6Vn,_cqnv,NI0KZ,KBYlM,kS_fV,JuHrP,sFhD0,sPuvk,gGde1,aRQW6,xroWi,R5SHH,ZBtn4,F9srm,UIHEp,m3Gwb)  NI0KZ
#define mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6(hJO0r,ATRYh,B6gr0,wF0dc,oLZeC,X1Qu2,MLkdu,znZoc,WOpt1,Gymuc,T9zO3,SDLQp,wx2Ql,hd1pW,Cr02j,gjm_8,nm2MC,e1qcR,aRCGE,wVYKX)  Cr02j
#define mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(l3EnF,ix6q8,iszm4,omv_H,DEunC,qQ2HE,WtEFi,OfDkP,IW39U,RmYPY,QaASs,n3GwJ,a6cWs,B5WqF,tlmjA,rZvd_,MkqIg,GzFtV,MH9O7,WH990)  B5WqF
#define mccWkSB6n44SUQqThNPpMc5NiiqdFV5(Zel3j,k3reP,LESi9,UE6Nk,Rw0Yi,QuLVy,VE63t,hNkk4,v66NS,RElJn,OXON0,WDYWH,ZZokH,vYwhw,_EbHZ,hYu9w,OL61q,eqwDY,p8lzq,cMuye)  k3reP
#define mYcHAJcazQi1vgNOos_uHnDBo1TFnau(IiW09,LxjAN,_DiFZ,GwjAH,q627c,Itqds,eyv37,CiT_y,c2ylB,g3NC9,THs7t,bDh_G,tXVjV,hYUkJ,gVjMg,o_8A8,oCEZC,iS_qC,ny1mi,Iho71)  THs7t
#define mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM(pUL8o,VoNQo,eBdr3,UJPyW,x_RoY,tWrAg,BNPTm,tsNKj,pTkKS,nQExC,s2I8D,UJJAl,iSuln,vk5u2,OYyOW,SoBSx,uhghq,aBtgv,k4ONz,j8zqW)  aBtgv
#define  mvXBiLTqg7lhkm1Nh8bPn  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(3,_,[,Z,P,q,;,z,},K,*,6,H,~,r,R,;,{,X,*)
#define  mj09THBEPXwun9vVG_ygd  mFhbh6WxadnFAtnPxvCHCXUYuwiM0dz(2,t,u,d,u,^,!,t,n,R,3,{,],i,*,P,/,9,_,S)
#define  mdYur1q_by6ODuryMT9iU  mhxwmdMskHstODI9NNDFcbcZGps2Qon(z,2,m,f,f,F,P,9,-,j,q,B,i,A,M,L,k,],f,^)
#define  makL5jqnU___VhQ5sewiV  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(t,C,p,N,<,!,Z,=,y,P,l,P,-,j,D,*,],{,O,Y)
#define  mbdsMa7jH60UElJsX0KDJ  mUNWZK3pUsXO7rBGNXsMmo1wkUjiEHs(a,s,H,;,c,S,],s,{,t,i,4,/,+,^,[,g,l,l,k)
#define  mpLgi59MRGuq_HOb9BA50  mHh6VteM99DWUdt2pX6MSuonEaJypdb(r,A,a,],t,n,y,x,[,4,e,f,J,u,j,7,-,*,t,R)
#define  mTZwdzczLVbxzpWnOdfGa  )
#define  mDtPKfWKFQ_J_7DQBz07x  if(
#define  mzD8x50gAKMFXpn1Kk7ZP  mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_(^,^,b,-,*,{,6,=,d,/,M,a,O,a,W,6,L,c,i,J)
#define  meDeg2depRbhujkehIjYn  mwfuzPp4XIekv0tplysWJFuHvsAMz64(P,O,6,M,o,W,z,i,},f,Q,I,d,2,/,c,l,-,{,v)
#define  mXh7D7IQ30bfYGV5sGPaO  mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3(M,1,*,9,N,Q,c,J,M,A,b,j,-,^,z,^,B,=,z,E)
#define  me_IA79PbLZ_OARrc_uPr  mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs(=,q,G,W,{,H,+,p,U,:,5,E,S,q,S,+,A,/,],c)
#define  mFspWuDxyb1nJ0DTdynBI  meaMoGxabLAzPaXAgGYpeoKJpBHV28L(7,t,i,3,/,u,+,b,2,Q,t,.,3,2,z,;,V,k,n,_)
#define  mKFfRfKNAdGHTMXcRWoHV  mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi(e,A,5,5,x,2,*,^,L,3,^,3,;,[,a,M,c,q,p,:)
#define  mo1oH2VnnRGtookyNl14U  mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ(+,1,U,u,b,*,>,+,V,{,!,^,B,*,n,+,!,o,F,m)
#endif

 mkE7Ykm6gZJFSVGppStlu 	 
    	  
ucoslam  msDA0oA68jPtXx6r60msz 	 
    	  
    		   
     
 
 mnAsK1UQpq2hqXnZP69XL 	 
    	  
    		   
     FrameExtractor mEYXB1C9jxDk6pOX0KXQ0 	 
    	toStream mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    	std mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
    		   ostream &_11093822381060 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
     
const
 mdkwtZ2sLtp1DPR2xAXUz 	 
    	  
    		   
     
     

    uint64_t _11093822380353 mC4xav6jq4r9qdxV8aAjL 	 
    	  1923123 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    _11093822381060.write mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
  mVQ8i37hpNkIkaxXGHhrO 	char* mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     
 
  	 &_11093822380353,sizeof mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
   _11093822380353 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		  mrnqzGFHPDarlUVk4S3a3 	 
 mcOUtWan0a817s2f7PxBD 	 
    	  
    		
    _8033463663468506753 mKWQiGcI03yFaWIZgwzoA 	 
    	  
    		   
     
  toStream mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
  _11093822381060 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
     
 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
 
    _11093822381060.write mjMWFdoUKZIiXkNZ57jQi 	 
    	 mVQ8i37hpNkIkaxXGHhrO 	 
char* m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
     
 
&_12273370977065616393,sizeof mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
     
 _12273370977065616393 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
  mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
 
  	 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
    _11093822381060.write mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
   mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		char* mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
&_12350051723532614025,sizeof mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
   _12350051723532614025 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
   m_f1eqo38yEJGZc_NJV2M 	 
    	  
 mFzprcwGX5TYPR8TCSOsI 	 
    
    _11093822381060.write mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
    mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
 char* m_f1eqo38yEJGZc_NJV2M 	 
    	&_3566717627060593117,sizeof mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
     
   _3566717627060593117 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		    mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
      mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
  
    _11093822381060.write mTnJ0tRkJ9I2h7pCRqgNu 	 
     mTnJ0tRkJ9I2h7pCRqgNu 	char* mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
     
     
 
 &_4309197024622458338,sizeof moU7cswum1tEpCFhdz0nH 	 
 _4309197024622458338 mmMWx9cmhAJYDHc6CoPK8 	 
    mrnqzGFHPDarlUVk4S3a3 	 
    	  
  mqzaY24A6rPasAO75wW1L 	 
   
    _11093822381060.write mVQ8i37hpNkIkaxXGHhrO 	 
    	 mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
     
char* mij3FyLvnkUgNUS9rv9RG 	 
    	  
  &_12693418215446769236,sizeof mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
    _12693418215446769236 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    	 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		
    _11093822381060.write mrWpmXK1Uy57IaLxMoHw4 	 
    	  
   mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     
     char* mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
     
 
 &_10675870925382111478,sizeof mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
   _10675870925382111478 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     
 
  	 mZ2_2tvDK9KkrBAARTPot 	 
  mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    	
    _11093822381060.write mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
   moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     
     
 
  	 char* mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     
 
&_13206983270957238750,sizeof mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     
     
 
  _13206983270957238750 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
     
      mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   mxzEpv4Eh_7CpNKRgx4W9 	 
    
  
    _13116459431724108758.toStream moU7cswum1tEpCFhdz0nH 	 _11093822381060 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     
 
   mH0vDS5erwxVyVoypRmvt 	 
    	 
 mUahIcsBNmcUVblDAWprS 	 
    	  
    		   
    
 mu5KEdiJoFgtXEFiJAn7w 	 
    	  
    		   
     
     
 
  FrameExtractor mEYXB1C9jxDk6pOX0KXQ0 	 
   fromStream mhjZhzZncRXafCfJ3w69z 	 std mreuaw_ExVZgNEVAfU1Ix 	 
    	  
istream &_11093822381060 mij3FyLvnkUgNUS9rv9RG 	 
    msDA0oA68jPtXx6r60msz 	 
    	  
    		   
     
     
 
    uint64_t _11093822380353 m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		   
     
     
 1923123 mxzEpv4Eh_7CpNKRgx4W9 	 
 
    _11093822381060.read mjMWFdoUKZIiXkNZ57jQi 	 
    	  
 mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
char* mij3FyLvnkUgNUS9rv9RG 	 &_11093822380353,sizeof mrWpmXK1Uy57IaLxMoHw4 	 
    	  
_11093822380353 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		    mH0vDS5erwxVyVoypRmvt 	 
    
     mLwp2qB0N1WcDxl0paSjv 	 
    	  
    		   
     
   mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
     
 
  	 _11093822380353 mNH8XnJPeJT8g0S6ijYW3 	 
    	  
    		 1923123 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		  throw std mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		  runtime_error mCNsgngMBa8r2GTvALgCE 	 
    	  
string mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
  __PRETTY_FUNCTION__ m_wCH5IAQfazKbdBPiKLv 	 
    	  
+"\x69\x6e\x76\x61\x6c\x69\x64\x20\x73\x69\x67\x6e\x61\x74\x75\x72\x65" mZ2_2tvDK9KkrBAARTPot 	 
    	   mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
   
    _8033463663468506753 mCPc29blbqZ5t89uMSE7C 	 
    Feature2DSerializable mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
     
     
 
 fromStream mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
 _11093822381060 mij3FyLvnkUgNUS9rv9RG 	 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
     
 
 
    _11093822381060.read mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
  mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
     
 
 char* mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		  &_12273370977065616393,sizeof mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
_12273370977065616393 m_f1eqo38yEJGZc_NJV2M 	 mrnqzGFHPDarlUVk4S3a3 	 
   mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
   
    _11093822381060.read mrWpmXK1Uy57IaLxMoHw4 	 
    	  
 moU7cswum1tEpCFhdz0nH 	 
   char* m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
&_12350051723532614025,sizeof mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
     
     
 
 _12350051723532614025 mZ2_2tvDK9KkrBAARTPot 	 
    	  
   m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
  	  mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
    
    _11093822381060.read mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		    mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   char* mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		&_3566717627060593117,sizeof mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		  _3566717627060593117 mdjqSIFIGMPy6TB2p5rXa 	 
     mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
 
  	  mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     

    _11093822381060.read mhjZhzZncRXafCfJ3w69z 	 
    	   mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
     
     
 char* mTZwdzczLVbxzpWnOdfGa 	 
   &_4309197024622458338,sizeof mh_Tf1Nm9McqJoH10VHzf 	 
    _4309197024622458338 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
    
    _11093822381060.read mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
      mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
 char* m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
&_12693418215446769236,sizeof moU7cswum1tEpCFhdz0nH 	 
    	  
    		  _12693418215446769236 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
   mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
    mENURSUbK5YsL6gDFG0XV 	
    _11093822381060.read mjMWFdoUKZIiXkNZ57jQi 	 
    	  moU7cswum1tEpCFhdz0nH 	 
 char* mTZwdzczLVbxzpWnOdfGa 	 
    	  
    	&_10675870925382111478,sizeof mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		  _10675870925382111478 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
     
 
  m_f1eqo38yEJGZc_NJV2M 	 
  mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
     

    _11093822381060.read mrWpmXK1Uy57IaLxMoHw4 	 mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
 char* m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
  &_13206983270957238750,sizeof mVQ8i37hpNkIkaxXGHhrO 	 
    	  
 _13206983270957238750 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
   mmMWx9cmhAJYDHc6CoPK8 	 
    	  
   mqzaY24A6rPasAO75wW1L 	 
    	  
    	

    _13116459431724108758.fromStream moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
_11093822381060 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 

 mAOUv187BUnBTJj3ytNVq 	 
    	  
    		   
     
     
 
FrameExtractor mNmJ6yxo1KXO341_e1pHs 	 
    FrameExtractor mwoYGv1lf3LjTo33qtGck 	 
    	  
    		   
   mq5iB_SvYcQOHaxqVDKSC 	 
    	  
    
 mAOUv187BUnBTJj3ytNVq 	 
    	  
 mj89OvJV2XAv03Crzylny 	 
    	  
    		   
     
     
 
 FrameExtractor mukxzF5Z4VwhAj2gq2GBE 	 
setParams mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   std mEYXB1C9jxDk6pOX0KXQ0 	 
    	  
    		   
     
 shared_ptr mkWP4IBnPeZGSrAaqSsJL 	 Feature2DSerializable mCdGviKX2tW09rD2iUjwV 	 
    	  
    		   
     
    _11022048600091140151,   const Params &_3005399798454910266, std mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
     
     
 
shared_ptr mAZ9rTzpHdADaRd2Bb691 	 
    	  
    		ucoslam mpU2hymgTA0othpUeyhm3 	 
    	  
   MarkerDetector mq3NMzP75aSEvua42dKaz 	  _1516358670470627782 mSSFbqZ6suzyqBFuTOK8y 	 
    mf93KHE75PL0YvPjf3hDs 	 
    	  
    _8033463663468506753 mC4xav6jq4r9qdxV8aAjL 	 
    	  
    		   
   _11022048600091140151 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
     
 
  
     miip_wX0isukeJQaecixC 	 
    	  
    		   
     
     
 
 mwh0H0CH8hbXoUMeW0njl 	_1516358670470627782 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     throw  std mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
   runtime_error mKzDQjDDFy9nfjTcIGEjn 	 
 "\x46\x72\x61\x6d\x65\x45\x78\x74\x72\x61\x63\x74\x6f\x72\x3a\x3a\x73\x65\x74\x50\x61\x72\x61\x6d\x73\x20\x69\x6e\x76\x61\x6c\x69\x64\x20\x6d\x61\x72\x6b\x65\x72\x20\x64\x65\x74\x65\x63\x74\x6f\x72" mSSFbqZ6suzyqBFuTOK8y 	 mcOUtWan0a817s2f7PxBD 	 
   
    _8000946946827829134 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
 _1516358670470627782 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
 
    _13116459431724108758 mEsCokXGzfSkbSNhRJF3n 	_3005399798454910266 mH0vDS5erwxVyVoypRmvt 	 
    	  
    	
    _3566717627060593117 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
   _3005399798454910266.detectMarkers mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
   
    _4309197024622458338 mC4xav6jq4r9qdxV8aAjL 	 
    	  
    		   
     
 _3005399798454910266.detectKeyPoints mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
   
    _12693418215446769236 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
     _3005399798454910266.aruco_markerSize mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
   
    _10675870925382111478 mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
     Feature2DSerializable mpU2hymgTA0othpUeyhm3 	 
    	  FeatParams mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
  _3005399798454910266.maxFeatures,_3005399798454910266.nOctaveLevels,_3005399798454910266.scaleFactor, _3005399798454910266.nthreads_feature_detector m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
  mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
     
 

    _13206983270957238750 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
     
    _3005399798454910266.maxDescDistance mKZSfq5CAVRIjVM6z9udz 	 
    	
 mAWVNR_NR1uiYYwpsz1Ic 	 
    	  
 meDeg2depRbhujkehIjYn 	 
    	  
    		 FrameExtractor mpU2hymgTA0othpUeyhm3 	 
    	  
    		   
     
     
 
 setSensitivity mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    	float _2654435887 mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
   mq5iB_SvYcQOHaxqVDKSC 	 
    	  
   
     mmq9HNT6SMSK3FI_mJgeB 	 
    	  
    		   
     
     
 _8033463663468506753 mTZwdzczLVbxzpWnOdfGa 	 
    	  

        _8033463663468506753 mumNyH1FYX3I32tTDmN7d 	 
    setSensitivity mTnJ0tRkJ9I2h7pCRqgNu 	 
    	 _2654435887 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
      mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     

 mAWVNR_NR1uiYYwpsz1Ic 	 
    	  
    		   
     
     
 
  	 
 mYo0CBFKDf0xO1zGnJmri 	 
    	  
    		   
   FrameExtractor mukxzF5Z4VwhAj2gq2GBE 	 
getSensitivity mJsXLE8zWibbVSZkgr7sR 	 mf93KHE75PL0YvPjf3hDs 	 
    	  
    		   
   
     mAdrqP9V1LvJsjA0Aj6ob 	 
    	  
   mg9RcyoUpBMcPhDg3RmnG 	 
    	  
    		   
     
     
_8033463663468506753 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
     throw std mukxzF5Z4VwhAj2gq2GBE 	 
    	  
    runtime_error mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
 string mxd1ooOS_bL0_Uh1bOsx8 	 
    __PRETTY_FUNCTION__ mdjqSIFIGMPy6TB2p5rXa 	 
+"\x53\x68\x6f\x75\x6c\x64\x20\x6e\x6f\x74\x20\x63\x61\x6c\x6c\x20\x74\x68\x69\x73\x20\x66\x75\x6e\x63\x74\x69\x6f\x6e\x20\x73\x69\x6e\x63\x65\x20\x74\x68\x65\x20\x63\x6c\x61\x73\x73\x20\x69\x73\x20\x6e\x6f\x74\x20\x69\x6e\x69\x74\x69\x61\x6c\x69\x7a\x65\x64" mZ2_2tvDK9KkrBAARTPot 	 
    	  
    	 mH0vDS5erwxVyVoypRmvt 	 
    	  
    	
     mfLJ4DZuYxLKPaFykYuXO 	_8033463663468506753 mZpLMilyO0bRxIH38sCpH 	 
    	  
    		  getSensitivity mdfitEjJDfX5JRuF8bXel 	 
    	  
    		   
     
  mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
   
 mUahIcsBNmcUVblDAWprS 	 
    	  
    		   
     
     
 

 mIbgaGLt_veu03B1y6TVE 	FrameExtractor mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		   
    processArray mrWpmXK1Uy57IaLxMoHw4 	 
  const vector mAZ9rTzpHdADaRd2Bb691 	 
    	  
    		   
     
     
 
  cv mNmJ6yxo1KXO341_e1pHs 	 
  Mat mx6J1P9EqT1QzNSE3H2NV 	 
    	  
    		   
     
     
  &_3005401535270843804,  ImageParams &_4702029808027735906 , Frame &_46082543180066935,  mZN1Zo1zK333HnGG3n9Kg 	 
   _9933887380370137445, const std mukxzF5Z4VwhAj2gq2GBE 	 
    	  
    		   
     
     
 
  	shared_ptr mNaDEYA4ktSo1tAee0UBy 	 
    	  
   MapInitializer mo1oH2VnnRGtookyNl14U 	 
    	  
    		   
     
     _16937373753903353427 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
     
 

 mq5iB_SvYcQOHaxqVDKSC 	 

    std mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
     
     
 
  	 vector mzfVZbevdXZt1lQmnk8So 	 
    	cv mzzUpgIrxS5Tz5DDcg3Pl 	 
    DMatch mq3NMzP75aSEvua42dKaz 	 
    	  
    		  _6807036698572949990 mtmf10UJavtFufEIdYJQE 	 
   
    _10230054520346001887 mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    _13116459431724108758.kptImageScaleFactor,_3005401535270843804,_4702029808027735906 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
    _17131366770609715580 mjMWFdoUKZIiXkNZ57jQi 	 
    	  
 _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	0 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
  ,_46082543180066935,_9933887380370137445 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
      mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		
    vector mfLTBWN12E1Fpsr4SddaU 	 
    	  
    		   
  float mrzLZmHoLhUzGPqSVSWYQ 	 
    	  
    		   _8152716818190584743 mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     
  _46082543180066935.scaleFactors mij3FyLvnkUgNUS9rv9RG 	 
    	  
   mENURSUbK5YsL6gDFG0XV 	 

     mtzxS2_2ntP3K4OCfU6H7 	 
 auto &_2654435887:_8152716818190584743 mmMWx9cmhAJYDHc6CoPK8 	 _2654435887 mf7uQnLk3PYBBCuntis6S 	 
_2654435887*_2654435887 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
    _46082543180066935.depth.resize mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     _46082543180066935.und_kpts.size mJsXLE8zWibbVSZkgr7sR 	 
    	  
  mmMWx9cmhAJYDHc6CoPK8 	 
    	  
     mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     

     mID3KGvR2YcYnpT7Ek1mJ 	 
    	  
    		size_t _2654435874 mp3ALuatVgPMBt3pB7xx9 	 
    0 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
     
 
  	 _2654435874 mERJ1GNplDMg0yOy8L0ug 	 
    	  
    		   
     
     _46082543180066935.depth.size mdfitEjJDfX5JRuF8bXel 	 
    	  
     mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
  _2654435874 mCXLyXNIpE5CVT6b8WA0b 	 
    	  
    		 mrnqzGFHPDarlUVk4S3a3 	 
    	  
 _46082543180066935.depth mDklz_JaoOvTrPYxdjKI0 	 
    	  
_2654435874 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
     
 
  	 mp3ALuatVgPMBt3pB7xx9 	 
    	 0 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
 
    vector mAZ9rTzpHdADaRd2Bb691 	 
    	 cv mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		   
  KeyPoint mTXGfaSGfbInmGn4DClnj 	 
    	  
    		   
     
     
 
  	 _15583426435820593759 mcOUtWan0a817s2f7PxBD 	 
    	  

    cv mEYXB1C9jxDk6pOX0KXQ0 	 
    	  
    		 Mat _15583426435822513722 mENURSUbK5YsL6gDFG0XV 	 
   
    _8033463663468506753 mVlIk0ncU1k0KblI7kXJb 	 
    	  
    		   
  detectAndCompute mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
     
 
_12800511165451773841 mqZiPh2UU3unhdzRfwVIt 	 
    	  
    		   
     1 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
._8358128829407646415,cv mEYXB1C9jxDk6pOX0KXQ0 	 
    	  
    		Mat maJhscyEMQhI3OBH5VVm0 	 
    ,_15583426435820593759,_15583426435822513722,_10675870925382111478 mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   mH0vDS5erwxVyVoypRmvt 	 
    	 
    
    vector mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
     
     
 cv mZauRjZulqrGATSW8brQM 	 
    	 Point2f msuASqXEqahJgXQfOZ6MX 	 
    	  
    		   
     
    _988585936380730887 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
 _988585936380730887.reserve mxd1ooOS_bL0_Uh1bOsx8 	 
  _15583426435820593759.size maJhscyEMQhI3OBH5VVm0 	 
    	  
    		   
 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    mqzaY24A6rPasAO75wW1L 	 
    	  
    		  
     mY6EkjnEEkk2kf3P2CNbA 	auto _2654435881:_15583426435820593759 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
    _988585936380730887.push_back mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
    _2654435881.pt mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
      mH0vDS5erwxVyVoypRmvt 	 
    	  
  
    _12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
 0 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
     
 
  	 ._5505640830793117477.undistortPoints mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
     
     
 
  	_988585936380730887,nullptr,1 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		  mqzaY24A6rPasAO75wW1L 	 
    
    vector mfLTBWN12E1Fpsr4SddaU 	 
  cv mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
     
     
 
  	 KeyPoint mo1oH2VnnRGtookyNl14U 	 _988585933247351277 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
 
    _988585933247351277  m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		    _15583426435820593759 mcOUtWan0a817s2f7PxBD 	 
    	  
     mbyb6jxRDAnCSInDFtmYO 	 
    	  
    		   
     
    mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     size_t _2654435874 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
 0 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		  _2654435874 mZmuVLfzgHBU2m2CtqWr0 	 
    	  
    		   
     _15583426435820593759.size mdfitEjJDfX5JRuF8bXel 	 
    	  
    		   
     
     
 
  	 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		 _2654435874 mdzT6psG_86ZMDW_2KJtn 	 
    	  
    		   
  mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
   
        _988585933247351277 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
     
  _2654435874 mUmS87lLj_Mf5SpVtoLqH 	 
    	  .pt mCPc29blbqZ5t89uMSE7C 	 _988585936380730887 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
     
     _2654435874 mzOhosYdDPvjBVX2OQJqA 	 
    	  
    		   
 mFzprcwGX5TYPR8TCSOsI 	 
  
    _16937373753903353427 mVlIk0ncU1k0KblI7kXJb 	 
    	  
    		   
     
    fmatcher.setParams mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     _46082543180066935,FrameMatcher mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
     MODE_ALL,_16937373753903353427 mQBJhEhGAPkNmt9WeIGXo 	 
 _params.minDescDistance,_16937373753903353427 mORFe0xZzjkJ3dJXtMmDi 	 
    _params.nn_match_ratio,true m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
  	  mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
 
  	 
    
    cv mRSJv1dHjU8olKcUlzVQM 	 Mat _706246338686106  mCPc29blbqZ5t89uMSE7C 	 
   _16937373753903353427 mQBJhEhGAPkNmt9WeIGXo 	 
    	  
    		   
     
     
 
  	 fmatcher.getFund12 mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
     
     
  _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   0 mmLapJobqTMcWmayld_1p 	 
    	  
    		  ._5505640830793117477.CameraMatrix,_12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 0 mUmS87lLj_Mf5SpVtoLqH 	 
   ._5505640830793117477.arrayCamMatrix mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
     
     
0 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
     
     
 ,
            getRTMatrix mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
_12800511165451773841 mV0JUzxDJGEJD_uX2w3nt 	 
   0 mvIsUEd5EevSMn8AkFEr2 	 
    ._5505640830793117477.arrayRvec mDklz_JaoOvTrPYxdjKI0 	 
    	  
    		   
     
   0 mmLapJobqTMcWmayld_1p 	 
    	  
 , _12800511165451773841 mDklz_JaoOvTrPYxdjKI0 	0 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    		   
     
  ._5505640830793117477.arrayTvec mKFfRfKNAdGHTMXcRWoHV 	 0 mmq78_qMEiQRxdANIEaE2 	 
    	  
   mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
   .inv mXewpxihHVBVnZgRIIOmS 	 
    	  
    		   
     
     
 
  	 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
 mxzEpv4Eh_7CpNKRgx4W9 	 
  
    getMatches mhjZhzZncRXafCfJ3w69z 	 
    	  
    		_6807036698572949990, _46082543180066935, _988585933247351277, _706246338686106, _15583426435822513722, 0 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
   mENURSUbK5YsL6gDFG0XV 	 
    
    filter_ambiguous_train mCNsgngMBa8r2GTvALgCE 	 
 _6807036698572949990 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
   mENURSUbK5YsL6gDFG0XV 	 
    	  
    		  
    std mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
 vector mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
     
   cv mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
     
  KeyPoint mf12hfREnmnF8LvpyEYPf 	 
    	  
    		   _3005399818393709360 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
     

    std mpU2hymgTA0othpUeyhm3 	 
    	  
    		vector mzfVZbevdXZt1lQmnk8So 	 
    	  
    		   
   cv mx_sf_UfonWBXCdYKLjrt 	KeyPoint mTXGfaSGfbInmGn4DClnj 	 
    	  
    		   
     
      _3005401586147246007 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
    
     mZwbWPcaOgtoYVI3Vym0_ 	 
    	  
 size_t _2654435878 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
 0 mFzprcwGX5TYPR8TCSOsI 	 
    _2654435878 mkD1iGSJt2oyfwNp4pWsZ 	 
    	  
    		 _6807036698572949990.size mJsXLE8zWibbVSZkgr7sR 	 
    	  
    		    mcOUtWan0a817s2f7PxBD 	 
    	  
 _2654435878 mDbODQ_pk1aeu1UegcTMF 	 
    	 mij3FyLvnkUgNUS9rv9RG 	 
    	 
     mdkwtZ2sLtp1DPR2xAXUz 	 
    	  
    		   
 
        _3005401586147246007.push_back mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
 _46082543180066935.und_kpts mqO12iB0NH1UmNi6cKZzy 	 
    	 _6807036698572949990 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
     
  _2654435878 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
  .queryIdx mmq78_qMEiQRxdANIEaE2 	 
    	  
    		   
     
   mmMWx9cmhAJYDHc6CoPK8 	 
    	   mFzprcwGX5TYPR8TCSOsI 	 
   
        _3005399818393709360.push_back moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
_988585933247351277 mfqY1vAKV8IyrQ6Hp8nMV 	 _6807036698572949990 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
  _2654435878 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		   
     
 .trainIdx mmLapJobqTMcWmayld_1p 	 
   mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
      mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
   
     mr6i8va6vrjg52LdPBFvK 	 
    	  
    		   
    
    std mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
 vector mkWP4IBnPeZGSrAaqSsJL 	 
    	  
    	cv mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   Point3f mTXGfaSGfbInmGn4DClnj 	 
    _11093822296219 mtmf10UJavtFufEIdYJQE 	 

    std mNmJ6yxo1KXO341_e1pHs 	 vector mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
     bool mCdGviKX2tW09rD2iUjwV 	 
    	  _3005399809928654743 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
     mE82ddK08RqkE2ioB87Cg 	 _46082544198958862  mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
   triangulate_ mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     getRTMatrix mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
  _4702029808027735906.arrayRvec mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
     
     
 
 0 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    	,_4702029808027735906.arrayTvec mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   
     
     
 
  	0 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
 mrnqzGFHPDarlUVk4S3a3 	 
   .inv mXewpxihHVBVnZgRIIOmS 	 
    	  
    		   
, _3005401586147246007, _3005399818393709360,
            _12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    		   
     
     
0 mmq78_qMEiQRxdANIEaE2 	 
    	  
    		   
     
    ._5505640830793117477.CameraMatrix, _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
0 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
     ._5505640830793117477.arrayCamMatrix mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		   
     
     
 
0 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		,
            _11093822296219,_46082543180066935.scaleFactors,_3005399809928654743 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
      mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
     
     
 
  
     mrraePCBsgi4fdXgdYTXy 	 
    	  
    		   
     
     uint _2654435874 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
     0 mtmf10UJavtFufEIdYJQE 	 _2654435874 mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		 _3005399809928654743.size mdfitEjJDfX5JRuF8bXel 	 
    mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
  _2654435874 mdzT6psG_86ZMDW_2KJtn 	 
    	  
    		   
     mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		
         miip_wX0isukeJQaecixC 	 
    	  
  _3005399809928654743 mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		  _2654435874 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    	 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    	
            _46082543180066935.depth mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		   
   _6807036698572949990 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    _2654435874 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    		   
.queryIdx mmq78_qMEiQRxdANIEaE2 	 
    	  mEsCokXGzfSkbSNhRJF3n 	 
 _11093822296219 mqQcJdeZf24ZMXk357zUD 	 
    	  
    		   
   _2654435874 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
  .z mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
   























 ml68IJIq89IrD6MxU__3X 	 
   
 mIbgaGLt_veu03B1y6TVE 	 
    	  
    		   
     
   FrameExtractor mRSJv1dHjU8olKcUlzVQM 	 
getMatches mjMWFdoUKZIiXkNZ57jQi 	 
   std mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
   vector mkWP4IBnPeZGSrAaqSsJL 	 
    	  
    cv mEYXB1C9jxDk6pOX0KXQ0 	 
    	  
    		   
     
DMatch mx6J1P9EqT1QzNSE3H2NV 	 
    	 & _6807036698572949990,  const Frame &_46082543180066935, const std mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
   vector mkWP4IBnPeZGSrAaqSsJL 	cv mNmJ6yxo1KXO341_e1pHs 	 
    	KeyPoint mf12hfREnmnF8LvpyEYPf 	 
    	  
    		   
     
   &_15583426435820593759,
                                const cv mNmJ6yxo1KXO341_e1pHs 	 
    	  
   Mat &_706246338686106, const cv mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		 Mat &_15583426435822513722,  mrSc6lwF_HlwXZHjcxq5S 	 
    	  
    		   
     
_2654435885 m_wCH5IAQfazKbdBPiKLv 	 
  
 msDA0oA68jPtXx6r60msz 	 
    	  
    		  
    cv mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
  Mat _6807141016749312283,_16988745808691518194 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
 
  	 
     mAdrqP9V1LvJsjA0Aj6ob 	 
    	  
    _2654435885 mum5iYeCnDDy8nbEyxtM5 	 
    	  
  0 mrnqzGFHPDarlUVk4S3a3 	 
    	  
  
     mSnypOnN3BGb0Sf99pqCF 	 
    	  
    		   
     
     
 
         mask7yuvCXZPiuY5e504A 	 
    	  
_1522768718325991990 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
     
 16 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   

         mSRiiEeTCByC6NcUUUK2v 	 
    	  
    		   
     
  _175247759447 m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		10 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
    
        xflann mukxzF5Z4VwhAj2gq2GBE 	 
    Index _11434284364572095166 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		  
        _11434284364572095166.build mVQ8i37hpNkIkaxXGHhrO 	 
    _15583426435822513722, xflann mZauRjZulqrGATSW8brQM 	 
    	  
    		   
    HKMeansParams mh_Tf1Nm9McqJoH10VHzf 	 
    	 32,0 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     
 
  	  mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     
 
   mqzaY24A6rPasAO75wW1L 	 
 
        _11434284364572095166.search mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
     
 _46082543180066935.desc,_175247759447,_6807141016749312283,_16988745808691518194 ,xflann mZauRjZulqrGATSW8brQM 	KnnSearchParams mxd1ooOS_bL0_Uh1bOsx8 	 
    	 _1522768718325991990,false mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     
  mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		  mKZSfq5CAVRIjVM6z9udz 	 
  
         mLwp2qB0N1WcDxl0paSjv 	 
    	  
    		   
     
     
 mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
     
 
 _16988745808691518194.type mdfitEjJDfX5JRuF8bXel 	 mggO2nUknv36WZKdfBBvE 	 
    	  
    		   
     
     
 
  CV_32S mmMWx9cmhAJYDHc6CoPK8 	 
 
            _16988745808691518194.convertTo mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
     _16988745808691518194,CV_32F m_f1eqo38yEJGZc_NJV2M 	 
    	 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
 
  
     mhypoZE0o7J5ZKuuPEU18 	 
    	  
    		   
     
     
 

    vector mzfVZbevdXZt1lQmnk8So 	 
    	  
    		 float mo1oH2VnnRGtookyNl14U 	 
    	  
    		   
     _8152716818190584743 mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
  _46082543180066935.scaleFactors mmMWx9cmhAJYDHc6CoPK8 	 
    	  mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     

     mGGEvKPIuZbqGUag8vVEx 	 
    	  
    		   
  auto &_2654435887:_8152716818190584743 mdjqSIFIGMPy6TB2p5rXa 	 
    _2654435887 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		  _2654435887*_2654435887 mqzaY24A6rPasAO75wW1L 	
     mE82ddK08RqkE2ioB87Cg 	 
  _706246333095486 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
   0 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
     

     mYQfKuc6wcfjjileYeZRV 	 
    	  
    		   
    size_t _16937466872311873851 mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		  0 mcOUtWan0a817s2f7PxBD 	 
    	  
    	 _16937466872311873851 mkWP4IBnPeZGSrAaqSsJL 	 
   _46082543180066935.und_kpts.size mwoYGv1lf3LjTo33qtGck 	 
    	  
    		   
     
     
 
  	  mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
     
 
   _16937466872311873851 m_YxQUUHIq8VTevXXDHD_ 	 
    m_wCH5IAQfazKbdBPiKLv 	 
    
     mf93KHE75PL0YvPjf3hDs 	 
    	  
    		 
        
         mmn2L475jNx9pG5OG93Og 	 
    	  
    		   
     
     
_706246330899664  mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		  _46082543180066935.und_kpts mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		   
     
     
 
  	_16937466872311873851 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
  .pt.x*_706246338686106.at mkWP4IBnPeZGSrAaqSsJL 	 
    	  
    		   
     
  float msuASqXEqahJgXQfOZ6MX 	 
    	  
    		  mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		  0,0 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   +_46082543180066935.und_kpts mGDXomYjoJafW8z9K6m1P 	 
 _16937466872311873851 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		   
     
     
 
  .pt.y*_706246338686106.at muZhC9Yd2qzjouoywqDiO 	 
    	  
    		   
     
     
 
  float mf12hfREnmnF8LvpyEYPf 	 
    	  
    		   
     
     
 
  mCNsgngMBa8r2GTvALgCE 	 
   1,0 mZ2_2tvDK9KkrBAARTPot 	 
    	+_706246338686106.at mzfVZbevdXZt1lQmnk8So 	 
    	  
    		   
     
     float mrzLZmHoLhUzGPqSVSWYQ 	 
    mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
2,0 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
   mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
    
         mmn2L475jNx9pG5OG93Og 	 
    	  
    		   
     
     
 
  	_706246330899667  mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
     
 _46082543180066935.und_kpts mOUmw6CStUwR2yTJByg62 	 
    	  
    		 _16937466872311873851 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		  .pt.x*_706246338686106.at mfLTBWN12E1Fpsr4SddaU 	 
    	  
    		   
     
     
 
  	float mx6J1P9EqT1QzNSE3H2NV 	 
    	  
    		   
 mCNsgngMBa8r2GTvALgCE 	 
    	  
    0,1 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
  +_46082543180066935.und_kpts mqZiPh2UU3unhdzRfwVIt 	 
    	  
    		   
    _16937466872311873851 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    		   
     
     
 
  	 .pt.y*_706246338686106.at muZhC9Yd2qzjouoywqDiO 	 
    	  
    		   
  float mo1oH2VnnRGtookyNl14U 	 
    	  
   moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     1,1 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
 +_706246338686106.at mNaDEYA4ktSo1tAee0UBy 	 
    	  
    		 float msuASqXEqahJgXQfOZ6MX 	 
    	  
    		   
    mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   2,1 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
 mH0vDS5erwxVyVoypRmvt 	
         mYo0CBFKDf0xO1zGnJmri 	 
    	  
    		   
     
   _706246330899666  mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
 _46082543180066935.und_kpts mqZiPh2UU3unhdzRfwVIt 	 
    	  
   _16937466872311873851 mVePGhmhzcrNpULYlc9MA 	 .pt.x*_706246338686106.at mNaDEYA4ktSo1tAee0UBy 	 
    	  
    		   
     
float msuASqXEqahJgXQfOZ6MX 	 
    	  
    	 mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
     
     
0,2 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
  +_46082543180066935.und_kpts mqO12iB0NH1UmNi6cKZzy 	 
   _16937466872311873851 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
  .pt.y*_706246338686106.at mAZ9rTzpHdADaRd2Bb691 	float mq3NMzP75aSEvua42dKaz 	 
    	  
    		   
     
     
  mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
     
     1,2 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
+_706246338686106.at mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
     float mo1oH2VnnRGtookyNl14U 	 
    	  
    		   mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
     
 
 2,2 mZ2_2tvDK9KkrBAARTPot 	 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
   












         mX7MFtiWgnK3l1N59mEfg 	_6461801683607599397 mwggywncoycNg0ZalVKcE 	 
    	  
    		   
     
     
 
  	 std mpU2hymgTA0othpUeyhm3 	 
    	  
    		   
     
     
numeric_limits muZhC9Yd2qzjouoywqDiO 	 
    	  
    		  float mrzLZmHoLhUzGPqSVSWYQ 	  mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
    		   
     
     
 
  	 max maJhscyEMQhI3OBH5VVm0 	 
    	  ,_6461801683607599396 mleKEARqJ89xIJCH7fsA_ 	 
    	 std mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
     
  numeric_limits mZmuVLfzgHBU2m2CtqWr0 	 
    	  
    	float msuASqXEqahJgXQfOZ6MX 	 mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		max mNGNAekcyVB3BBHwb2S0y 	 
    	  
    		    mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
   
        int64_t _6461801682275966366 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
     -1, _6461801682249544409 m_2dgqWMgdoegqwSE6Fwl 	 
    -1, _12031385974440167876 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
  -1 mQfS9tiOvGsXX9DD53UoY 	 

         mCZX10THppqeTgSrqysBt 	 
    	  
    		   
     
     
 
  	 _2654435885 mM1Wr0HMg3CNI1MP7nzg_ 	 
    	 0 mTZwdzczLVbxzpWnOdfGa 	 
  
         mXvM0qorf_SpbgDdpYfPu 	 
    	  
    		   
     
     
 

             mYQfKuc6wcfjjileYeZRV 	 
    	  
    		 int _2654435875 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		 0 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   _2654435875 mAZ9rTzpHdADaRd2Bb691 	 
    	  
    		   
     
     _6807141016749312283.cols mQfS9tiOvGsXX9DD53UoY 	 
    	  _2654435875 mx1eIclnlkObQ3brbbK1m 	 
     mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
 
             mq5iB_SvYcQOHaxqVDKSC 	 
    
                 mSRiiEeTCByC6NcUUUK2v 	 
    	  
    	_16997329059161965027 mp3ALuatVgPMBt3pB7xx9 	 
  _6807141016749312283.at mzfVZbevdXZt1lQmnk8So 	 
  int mrzLZmHoLhUzGPqSVSWYQ 	 
    	  
    		   
     
 mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    _16937466872311873851,_2654435875 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     

                








                
                 mmBFyV0pRbQZSqu8LEMmC 	 
    	  
    		   
    std mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
     
     
 
  abs mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
     
 _46082543180066935.und_kpts mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    _16937466872311873851 mmq78_qMEiQRxdANIEaE2 	 
    	  
    		   
   .octave - _15583426435820593759 mqQcJdeZf24ZMXk357zUD 	 
    	  
    		   
     
  _16997329059161965027 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
     
     
.octave mmMWx9cmhAJYDHc6CoPK8 	 
    	  mO3cy6qBtctqMh9v1n6AM 	 
    1 mZ2_2tvDK9KkrBAARTPot 	 
  continue mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
 
                 mPobsR0SUkL4k40J6meFr 	 
    	  
 _2654435869  m_2dgqWMgdoegqwSE6Fwl 	 
    	  
  epipolarLineSqDist mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
     
 
  	_46082543180066935.und_kpts mqO12iB0NH1UmNi6cKZzy 	 
    	  
   _16937466872311873851 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
 .pt, _15583426435820593759 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
  _16997329059161965027 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		   
     
   .pt, _706246338686106 mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
 
 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    
                 mrNEGap3YpE0gqVau1IbE 	 mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     _2654435869  mErWLHXQ2oP3Ykz5Pkqx2 	 
    	  
 3.84*_8152716818190584743 mGDXomYjoJafW8z9K6m1P 	 
    	  
  _46082543180066935.und_kpts mDklz_JaoOvTrPYxdjKI0 	 
    	  
    		   
     
     
 _16937466872311873851 mvIsUEd5EevSMn8AkFEr2 	 
    	  .octave mmLapJobqTMcWmayld_1p 	 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
   continue mqzaY24A6rPasAO75wW1L 	 
    	  
   
                 mVKRBrgbb_3KvAoVy7Y76 	 
    	  
    		   
     
     mTnJ0tRkJ9I2h7pCRqgNu 	 _16988745808691518194.at mAZ9rTzpHdADaRd2Bb691 	 
    	  
    		   
     
float msuASqXEqahJgXQfOZ6MX 	 
    	  
    		  mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
  _16937466872311873851,_2654435875 m_wCH5IAQfazKbdBPiKLv 	 
    	  mf12hfREnmnF8LvpyEYPf 	 
    	  
 _13206983270957238750 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
    continue mtmf10UJavtFufEIdYJQE 	 
  
                 mVKRBrgbb_3KvAoVy7Y76 	 
    	  
    		   
     
     
 
 mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
_16988745808691518194.at muZhC9Yd2qzjouoywqDiO 	 
    	  
    		   
     
     
 
  	 float msuASqXEqahJgXQfOZ6MX 	 
    	  
  mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
 _16937466872311873851,_2654435875 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		 mERJ1GNplDMg0yOy8L0ug 	 
    	  
    _6461801683607599396 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    
                 mXvM0qorf_SpbgDdpYfPu 	 
    	 
                     mfjRK0U1BIpjv9bVGRmJR 	 
     mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
    _16988745808691518194.at mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		float mCdGviKX2tW09rD2iUjwV 	 
    	  
    		   
     
  mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
_16937466872311873851,_2654435875 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     mERJ1GNplDMg0yOy8L0ug 	 _6461801683607599397 mSSFbqZ6suzyqBFuTOK8y 	 
    	
                     msDA0oA68jPtXx6r60msz 	 
    	  
    		   
                        _706246333095486 mCXLyXNIpE5CVT6b8WA0b 	 
  mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
   
                        _6461801683607599397 mC4xav6jq4r9qdxV8aAjL 	 
    	  
   _16988745808691518194.at mkD1iGSJt2oyfwNp4pWsZ 	 
    	  
    		   
     
     
float mTXGfaSGfbInmGn4DClnj 	 
    	  
    		   
     
     
 
  	 mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    _16937466872311873851,_2654435875 mSSFbqZ6suzyqBFuTOK8y 	 
    	  mqzaY24A6rPasAO75wW1L 	 
    	 
                        _6461801682249544409  mC4xav6jq4r9qdxV8aAjL 	 
    	  
    		   
     
   _16937466872311873851 mKZSfq5CAVRIjVM6z9udz 	 
   
                        _6461801682275966366 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
     
 
  _16997329059161965027 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
                     mFlLiaiBJqFiDudfeE9Kh 	 
    	  
    		   
     
     
 
  	
                    else
                     mGwSL4y5H5STcCbNqgDkd 	
                        _6461801683607599396 mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
     
 _16988745808691518194.at mfLTBWN12E1Fpsr4SddaU 	 
    float mfiSRAwugVLmNlTHqDQLn 	 
    	  
    mhjZhzZncRXafCfJ3w69z 	 
    	 _16937466872311873851,_2654435875 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
   mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
  
                        _12031385974440167876 mCPc29blbqZ5t89uMSE7C 	 
    	 _15583426435820593759 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
     
    _16997329059161965027 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
     
 
  .octave mqzaY24A6rPasAO75wW1L 	 
    	
                     mAOUv187BUnBTJj3ytNVq 	 
    	  
    		 
                 mTQguI68n8F10NlqLVj95 	 
  
             mFlLiaiBJqFiDudfeE9Kh 	 
    	  
    		   
     
         mFlLiaiBJqFiDudfeE9Kh 	 
    	  
    		   
    
         mI_KmnkodyswOabezai5H 	 
    	  
    mOb7vP5ncyxcZGeJJnrwa 	 
    	  
    	_2654435885 mCdgKNbzbgzTX98EOSa5l 	 
    	  
    		   
     
     
 
  	1 mSSFbqZ6suzyqBFuTOK8y 	 
    	   mf93KHE75PL0YvPjf3hDs 	 
    	 
            
             mID3KGvR2YcYnpT7Ek1mJ 	 
    	  
    		   
     
size_t _16997329059161965027 mB1LJDQLgEtQsHykyiRGq 	 0 mH0vDS5erwxVyVoypRmvt 	 
    	   _16997329059161965027 mZmuVLfzgHBU2m2CtqWr0 	 
    	  
    		   
  _15583426435820593759.size mXewpxihHVBVnZgRIIOmS 	 
    	  
    		   
   mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
  _16997329059161965027 mWeo5kjFRZxMUAQe8mmxw 	 
    	  
    		   mZ2_2tvDK9KkrBAARTPot 	 
    	  

             mkMa5jPmx9O_418wkkYBV 	 
    	  
    		   
                 mlOlwCYyTQx2KagePc2is 	 
    	  
    		   
    _11093822061254  mCPc29blbqZ5t89uMSE7C 	 
    	  
    		   
     fabs mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    	_706246330899664*_15583426435820593759 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
_16997329059161965027 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
.pt.x + _706246330899667*_15583426435820593759 mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   _16997329059161965027 mqxiqEVJqEpdcx2wQqpBd 	 
    	  .pt.y + _706246330899666 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    	 / sqrt mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
     
     
pow mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
  _706246330899664,2 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    + pow moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     
     
 
  	_706246330899667,2 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
  mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
     
 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
 
  	 
                 mjvzxrxfDdEk7DKEcp_1A 	 
    	  
    		   
     
     
  mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     _11093822061254  mhQdIzwRajAQGMZcowBBZ 	 3.84*_8152716818190584743 mqO12iB0NH1UmNi6cKZzy 	 
    	  
   _46082543180066935.und_kpts mqQcJdeZf24ZMXk357zUD 	 
    	  
    		   
    _16937466872311873851 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
.octave mqxiqEVJqEpdcx2wQqpBd 	 
     mZ2_2tvDK9KkrBAARTPot 	 
    	  continue mH0vDS5erwxVyVoypRmvt 	 
    	







                 mJVqrARa5craS_rpTwFoN 	 
 std mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		   
   abs mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     
  _46082543180066935.und_kpts mqQcJdeZf24ZMXk357zUD 	 
    	  
    		   
     
     
 
  _16937466872311873851 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		   
   .octave - _15583426435820593759 mqZiPh2UU3unhdzRfwVIt 	 
    	  
 _16997329059161965027 m_f10xp9iBg5cy1Elj1NV 	 
    	  .octave mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   mfiSRAwugVLmNlTHqDQLn 	 
    	  
    		   
  1 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   continue mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   

                 mM_JyoqKJWOyd7xyV_Pob 	 
    	 _16940388568078030328 mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
     
 MapPoint mTED_KGydiVM1RbH5g_XM 	 
    	  
    		   
     
     
getDescDistance mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		  _46082543180066935.desc,_16937466872311873851,_15583426435822513722,_16997329059161965027 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
  mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
 
                 mx3yy2lZVVSJ9u9EbkIfD 	 
    	  
    		   
     
   mKzDQjDDFy9nfjTcIGEjn 	  _16940388568078030328 msuASqXEqahJgXQfOZ6MX 	 
   _13206983270957238750 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
    continue mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
                 mLwp2qB0N1WcDxl0paSjv 	 
    	  
    		   
     
     
 
  mVQ8i37hpNkIkaxXGHhrO 	 
    	  
   _16940388568078030328 mkWP4IBnPeZGSrAaqSsJL 	 
    	  
    		   
     
     
 
  	_6461801683607599396 m_f1eqo38yEJGZc_NJV2M 	 
    	  
  
                 mdkwtZ2sLtp1DPR2xAXUz 	 
    	  
    		   
     
    
                     mLwp2qB0N1WcDxl0paSjv 	 
    	  
    	 mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
     
 
_16940388568078030328 mzfVZbevdXZt1lQmnk8So 	 
    	  
    		   
     
     
_6461801683607599397 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
  
                     mXvM0qorf_SpbgDdpYfPu 	 
    	 
                        _6461801683607599397 mleKEARqJ89xIJCH7fsA_ 	 
    	 _16940388568078030328 mcOUtWan0a817s2f7PxBD 	
                        _6461801682249544409  mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
      _16937466872311873851 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
    
                        _6461801682275966366 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
     
  _16997329059161965027 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 
  	
                     mAOUv187BUnBTJj3ytNVq 	 
    	  
    		   
   
                    else
                     mf93KHE75PL0YvPjf3hDs 	 
    	  
    		   
     
                        _6461801683607599396 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
     
     
 
  	 _16940388568078030328 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
     
                        _12031385974440167876 mEsCokXGzfSkbSNhRJF3n 	 
    	  
   _15583426435820593759 mGDXomYjoJafW8z9K6m1P 	 
    _16997329059161965027 mmLapJobqTMcWmayld_1p 	 
    	  
    		.octave mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     

                     mAOUv187BUnBTJj3ytNVq 	 
    	  
    	
                 mAOUv187BUnBTJj3ytNVq 	 
    	  
    		   
     
     
 
 
             mFlLiaiBJqFiDudfeE9Kh 	 
    	  
    		  
         mTQguI68n8F10NlqLVj95 	 

        



        
         mdYur1q_by6ODuryMT9iU 	 
    	  
    		   
     
     mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     
     
 
  	  _6461801682249544409 mLwr3ORKxhiYVBNWCtrz3 	 
    	  
    		   
     
     
-1 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
 mSnypOnN3BGb0Sf99pqCF 	 
    	  
    		   
     
 
             mOb7vP5ncyxcZGeJJnrwa 	  mfepBt_IkXqdRWgLD_6UJ 	 
    	  
    		   
     
  mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
     
 
 _12031385974440167876 mum5iYeCnDDy8nbEyxtM5 	 
    	  _46082543180066935.und_kpts mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
     
  _6461801682249544409 mmLapJobqTMcWmayld_1p 	 
.octave  mgdTJRVTRu7KCcddhNYkR 	 
    	  
    		   
       _6461801683607599397  msuASqXEqahJgXQfOZ6MX 	 
   _6461801683607599396*0.8f   mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
 
 
             msDA0oA68jPtXx6r60msz 	 
    	  
                cv mx_sf_UfonWBXCdYKLjrt 	 DMatch  _46082575882272165 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		
                _46082575882272165.queryIdx mwggywncoycNg0ZalVKcE 	 
    	  
    		   _6461801682249544409 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
     

                _46082575882272165.trainIdx mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		 _6461801682275966366 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     

                _46082575882272165.distance m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    _6461801683607599397 mQfS9tiOvGsXX9DD53UoY 	
                _6807036698572949990.push_back moU7cswum1tEpCFhdz0nH 	_46082575882272165 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
    mENURSUbK5YsL6gDFG0XV 	 



















             mXQsUGNgZoukfTO39vkwP 	 
    	  
    		   
     
   
         mTQguI68n8F10NlqLVj95 	 
    	  
    		   
     
     
 
  	
     mUahIcsBNmcUVblDAWprS 	 
    	  
  
 mAWVNR_NR1uiYYwpsz1Ic 	 
    	  
    		   
     
 
 mIbgaGLt_veu03B1y6TVE 	 
    	  
    		   FrameExtractor mEYXB1C9jxDk6pOX0KXQ0 	 
 processStereo mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
   const cv mTED_KGydiVM1RbH5g_XM 	 
    	  Mat &_16937270569698259356, const cv mRSJv1dHjU8olKcUlzVQM 	Mat &_1705492249324718059, const ImageParams &_175247760147, Frame &_46082543180066935,  mj1FYqv8U0ZeHnxgDf42G 	_9933887380370137445 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
   
 mSnypOnN3BGb0Sf99pqCF 	
    _14329080428242784455 mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   _13116459431724108758.kptImageScaleFactor,_16937270569698259356,_175247760147,_1705492249324718059  mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
  mH0vDS5erwxVyVoypRmvt 	
    _17131366770609715580 mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
 _12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
     
  0 mzOhosYdDPvjBVX2OQJqA 	 
    	  
    		   
     
,_46082543180066935,_9933887380370137445 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
     
 
 mjA8LcpHiJRrjo1GrHbPY 	 
    	  

    _46082543180066935.depth.resize mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
  _46082543180066935.und_kpts.size mOuSLzvQYyP5qwR44RZl0 	 
    	  
    		   
     
   mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
 
     mtzxS2_2ntP3K4OCfU6H7 	 
    	  
    		   
size_t _2654435874 m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		   
     
     
 
 0 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		  _2654435874 mERJ1GNplDMg0yOy8L0ug 	 
    	  
    		 _46082543180066935.depth.size mdfitEjJDfX5JRuF8bXel 	 
    	  
    		   
     
     
 
 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
   _2654435874 mq9SRhyfxCdaIMKqQ8BFs 	 
    	  
    		   
     
   m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
    _46082543180066935.depth mDklz_JaoOvTrPYxdjKI0 	 
    _2654435874 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    		   mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   0 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
  
    vector mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
     
     
 
  	 cv mZauRjZulqrGATSW8brQM 	 
    	  
   KeyPoint mf12hfREnmnF8LvpyEYPf 	 
   _16937465944335025110 mENURSUbK5YsL6gDFG0XV 	 
    	  
  
    cv mTED_KGydiVM1RbH5g_XM 	 
    	  
    		   
     Mat _1534768455952794120 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		 
    _8033463663468506753 mXiLtzRcWjDEAek2NbwiV 	 
    	  
    		   
     
     
 detectAndCompute mrWpmXK1Uy57IaLxMoHw4 	 
_12800511165451773841 mqZiPh2UU3unhdzRfwVIt 	 
    	  
    		   
     
     
1 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
     
 ._8358128829407646415,cv mTED_KGydiVM1RbH5g_XM 	Mat msRSLa1rEosf7XXiVm4xR 	 
    	  
    		   
     
     
 
 ,_16937465944335025110,_1534768455952794120,_10675870925382111478 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
 mqzaY24A6rPasAO75wW1L 	 
    	  
    		
    vector mERJ1GNplDMg0yOy8L0ug 	 
    	  
    		  vector mfLTBWN12E1Fpsr4SddaU 	 
    	  
    		   
     
     
int mU1JHGYmHGuiMF3xXenzY 	 
    	  
    		   
   _13925384095147685413 mxd1ooOS_bL0_Uh1bOsx8 	 
    _12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
    	  
  1 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		  ._8358128829407646415.rows mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 
  	 
     mZwbWPcaOgtoYVI3Vym0_ 	 
    	  
    		   
size_t _2654435874 mCPc29blbqZ5t89uMSE7C 	 
    	  
    		   
     
     
0 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 
   _2654435874 mfLTBWN12E1Fpsr4SddaU 	 
    	  
 _16937465944335025110.size mOuSLzvQYyP5qwR44RZl0 	 
    	  
    		   
     
     
 
  	 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
  _2654435874 mdzT6psG_86ZMDW_2KJtn 	 mrnqzGFHPDarlUVk4S3a3 	 
    	  
  mc3Mj6HvensKjoIAWkC9N 	 
    	  
    		   
     
 
         mx8JhB_9A0B0jSDMRVJBM 	 
  _12827320108209534756 mEsCokXGzfSkbSNhRJF3n 	 
    	 0 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
     
  
         mx8JhB_9A0B0jSDMRVJBM 	 
    	  _2654435890 mC4xav6jq4r9qdxV8aAjL 	 
    	  
    		_16937465944335025110 mqZiPh2UU3unhdzRfwVIt 	 
    	  
    		   
 _2654435874 m_f10xp9iBg5cy1Elj1NV 	 
   .pt.y mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
   
         mjyI2a10ukdPXJp8Ii1qL 	 
    	  
_46082575884402673 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
     std mZauRjZulqrGATSW8brQM 	 
    	  
    		 max mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		 0,int moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     std mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
     
 round mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		 _2654435890-_12827320108209534756 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		    mij3FyLvnkUgNUS9rv9RG 	 
    	  
    mQfS9tiOvGsXX9DD53UoY 	 
    	  
         mt4oZdscHQLkgCboylyXW 	 
    	  
    		   
     
     _46082575882517952 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
     
     
 
std mpU2hymgTA0othpUeyhm3 	 
    	  
    		   
 min mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
  _12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
     
     
 
  	1 mqxiqEVJqEpdcx2wQqpBd 	._8358128829407646415.rows-1,int mhjZhzZncRXafCfJ3w69z 	 
    	  
    	std mx_sf_UfonWBXCdYKLjrt 	 
  round mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     
     
_2654435890+_12827320108209534756 mrnqzGFHPDarlUVk4S3a3 	 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
  	  mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		    mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
    
         mY6EkjnEEkk2kf3P2CNbA 	 
    	  
 int _2654435890 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
     
     
 
_46082575884402673 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		 _2654435890 moob3Tjk5_eucM91XQB9l 	 
 _46082575882517952 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    _2654435890 mvyz9F7m7Z76dYn99FG0x 	 
   mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
   
            _13925384095147685413 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
  _2654435890 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
     
     
 
 .push_back mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
     
     
 _2654435874 mSSFbqZ6suzyqBFuTOK8y 	  mQfS9tiOvGsXX9DD53UoY 	
     mAWVNR_NR1uiYYwpsz1Ic 	 
    	  
    		   
     
 
    
     mask7yuvCXZPiuY5e504A 	 
    	  
    		   
  _14197860223095419442 mgd705K7LHsqyMTqQZd5V 	 
    	  
    0 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   

    
     mID3KGvR2YcYnpT7Ek1mJ 	 
    	  size_t _2654435874 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
     
 0 mKZSfq5CAVRIjVM6z9udz 	 
    	  
   _2654435874 mfLTBWN12E1Fpsr4SddaU 	 
    	  
    		   
     
     
_46082543180066935.und_kpts.size mwoYGv1lf3LjTo33qtGck 	 mH0vDS5erwxVyVoypRmvt 	 
    	   _2654435874 mq9SRhyfxCdaIMKqQ8BFs 	 
    	 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
     
 mc3Mj6HvensKjoIAWkC9N 	 
        
         mnZIMdxSe0bwhxrHI7GNX 	 
    	  
    		   
     _2654435890 mwggywncoycNg0ZalVKcE 	 
    	  
    		   
     
     
 std mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
 round mh_Tf1Nm9McqJoH10VHzf 	 
    	_46082543180066935.und_kpts mqZiPh2UU3unhdzRfwVIt 	 
    	  
    		   
     
     
 
_2654435874 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		 .pt.y mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		    mtmf10UJavtFufEIdYJQE 	 
    	  
    		 
        vector mNaDEYA4ktSo1tAee0UBy 	 
    	  int mfiSRAwugVLmNlTHqDQLn 	 
    	  
     &_46082575733867251 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    _13925384095147685413 mfqY1vAKV8IyrQ6Hp8nMV 	 _2654435890 mVePGhmhzcrNpULYlc9MA 	 
    	  
   mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     

        
        
        
         mask7yuvCXZPiuY5e504A 	 
    	  
  _16992066382187640319 mwggywncoycNg0ZalVKcE 	 
    	  
   -1 mcOUtWan0a817s2f7PxBD 	 
 
         mwDPBjt_JAxtL9j0stKBK 	 
    	 _12757546498867192361 mwggywncoycNg0ZalVKcE 	 
    	  
    		   
  std mTED_KGydiVM1RbH5g_XM 	 
    	  
    		  numeric_limits mzfVZbevdXZt1lQmnk8So 	 
 double mf12hfREnmnF8LvpyEYPf 	 
    	 mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		max mNGNAekcyVB3BBHwb2S0y 	 
    	  
    		   
     
     
 
  	  mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
     
     
 
  	
        
        
         mGGEvKPIuZbqGUag8vVEx 	 
    	size_t _2654435875 mp3ALuatVgPMBt3pB7xx9 	 
  0 mKZSfq5CAVRIjVM6z9udz 	 
   _2654435875 mzfVZbevdXZt1lQmnk8So 	 
    	  
    		   
_46082575733867251.size mji_CmPnVQxlriixIouJp 	 
    	  
     mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
     
     
 _2654435875 mx1eIclnlkObQ3brbbK1m 	 
    	  
   mZ2_2tvDK9KkrBAARTPot 	 
    	 mf93KHE75PL0YvPjf3hDs 	 
             mt4oZdscHQLkgCboylyXW 	 
    	  
    		_5555178974982484866 mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
   _46082575733867251 mKFfRfKNAdGHTMXcRWoHV 	 
    	  _2654435875 mmLapJobqTMcWmayld_1p 	 
    	  
   mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
             mAdrqP9V1LvJsjA0Aj6ob 	 
    	  
    		   
   _16937465944335025110 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
    _5555178974982484866 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
    .pt.x  mrzLZmHoLhUzGPqSVSWYQ 	 
    	  
    		   
     
     
 
   _46082543180066935.und_kpts mqO12iB0NH1UmNi6cKZzy 	 
    	  
_2654435874 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		   
     
     .pt.x  mQW1qj2eB_butTzRBCZ8q 	 
    	  
    		   
    std mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
     
 
abs mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
   _16937465944335025110 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
    _5555178974982484866 m_f10xp9iBg5cy1Elj1NV 	 
    	  
   .octave-_46082543180066935.und_kpts mDklz_JaoOvTrPYxdjKI0 	 
_2654435874 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
     
  .octave mrnqzGFHPDarlUVk4S3a3 	 
    	   mf12hfREnmnF8LvpyEYPf 	 
    	  
  1  mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   

                continue mcOUtWan0a817s2f7PxBD 	 
    	  
            
            
                 mJAKI0m004Q4BblDgaUGq 	 
    	  
   _706246353090457 mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
     
MapPoint mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
     
     
 
 getDescDistance mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
_46082543180066935.desc,_2654435874,_1534768455952794120,_5555178974982484866 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
  	 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
                 mOb7vP5ncyxcZGeJJnrwa 	 
    	  
    		   
     
     
 
_706246353090457 mfLTBWN12E1Fpsr4SddaU 	 
    	  
    		   
   _13206983270957238750 m_f1eqo38yEJGZc_NJV2M 	 
    msDA0oA68jPtXx6r60msz 	 
                    
                     mIbK2m28YepmBKoCm61cv 	 
    	  
    		   
_706246353090457 mNaDEYA4ktSo1tAee0UBy 	 
    	  
 _12757546498867192361 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
  msDA0oA68jPtXx6r60msz 	 
    
                        _12757546498867192361 mCPc29blbqZ5t89uMSE7C 	 
    	  
    		   
     
     
 
  	_706246353090457 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		 
                        _16992066382187640319 m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		   
_5555178974982484866 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
  
                     mUahIcsBNmcUVblDAWprS 	 
    	  
    		   
   
                 mXQsUGNgZoukfTO39vkwP 	 
    	  
    		   
     
            
         mFlLiaiBJqFiDudfeE9Kh 	 
  
         mOb7vP5ncyxcZGeJJnrwa 	 
    	  
    		   
     _16992066382187640319  mJSdvdtWO4_pkCgiN7xj5 	 
    	  
    		   
     
     
  -1 mZ2_2tvDK9KkrBAARTPot 	 
    	 mkMa5jPmx9O_418wkkYBV 	 
    	  
    
            
            
            
             mX7P4s1_g7S0Vw9Y54fYj 	 
    	  
    		   
     
     
 _5820950562624380433 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
  7 mFzprcwGX5TYPR8TCSOsI 	 
    	  
  
             mnZIMdxSe0bwhxrHI7GNX 	 
    	  
    		   
     
     
 
  	 _11450180203511629323 mC4xav6jq4r9qdxV8aAjL 	 
 _5820950562624380433/2 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
     
             mrSc6lwF_HlwXZHjcxq5S 	 
    	  
    		   
     _1588944432958720138 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
std mukxzF5Z4VwhAj2gq2GBE 	 
    	  
    		   
     
  round mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     
     _46082543180066935.und_kpts mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  _2654435874 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
  .pt.x mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
 
             mnZIMdxSe0bwhxrHI7GNX 	 
    	  
    		   
     
     
_1588944432958720141 mC4xav6jq4r9qdxV8aAjL 	 
    	  
    std mTED_KGydiVM1RbH5g_XM 	 
  round mhjZhzZncRXafCfJ3w69z 	 _46082543180066935.und_kpts mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   
_2654435874 mvIsUEd5EevSMn8AkFEr2 	 
.pt.y mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
 
  	 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
   
             mmq9HNT6SMSK3FI_mJgeB 	 
    	  
    		  _1588944432958720138 mAZ9rTzpHdADaRd2Bb691 	 
    	  
    		   
 _11450180203511629323  mZa6h8IcDq4tg9PE0pSxx 	 _1588944432958720138+_11450180203511629323 ms8yr8htzxRLyf0ZH9bpl 	 
    	  
    		   
    _16937270569698259356.cols mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
     
 
  	 
                continue mFzprcwGX5TYPR8TCSOsI 	 
    	  

             mCZX10THppqeTgSrqysBt 	 
   _1588944432958720141 mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
     _11450180203511629323  mxIO7h6PJBkfricKCIyow 	 
    	  _1588944432958720141+_11450180203511629323 mi9GC6IycKxFSz9g_CVXZ 	 
    _16937270569698259356.rows mSKLIkoiXh2O2ZJCB5tMN 	 
    	  

                continue mjA8LcpHiJRrjo1GrHbPY 	
             mask7yuvCXZPiuY5e504A 	 
    	  
    		   
     
   _4939411723264461086 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
     
std mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
     
 
  round mxd1ooOS_bL0_Uh1bOsx8 	_16937465944335025110 mQSqjUxpUR2zqVJEYbq5M 	 
_16992066382187640319 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
  .pt.x m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
  mxzEpv4Eh_7CpNKRgx4W9 	 
    	 
             mnZIMdxSe0bwhxrHI7GNX 	 
    	  
    		   
     
     
 
 _4939411723264461085 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
     std mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
  round mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     _16937465944335025110 mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    _16992066382187640319 mVePGhmhzcrNpULYlc9MA 	 
 .pt.y m_f1eqo38yEJGZc_NJV2M 	 
    	   mKZSfq5CAVRIjVM6z9udz 	 
    	  
 
             mOb7vP5ncyxcZGeJJnrwa 	 
    	  
    		   
     
    _4939411723264461086 muZhC9Yd2qzjouoywqDiO 	 
    	_11450180203511629323  mQW1qj2eB_butTzRBCZ8q 	 
    	  
     _4939411723264461086+_11450180203511629323 mErWLHXQ2oP3Ykz5Pkqx2 	 
    	  
  _1705492249324718059.cols m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		 
                continue mcOUtWan0a817s2f7PxBD 	 
  
             mCZX10THppqeTgSrqysBt 	 
    	_4939411723264461085 mfLTBWN12E1Fpsr4SddaU 	 
    	_11450180203511629323  mXDnhW__VmwWwJIvnFsLN 	 
    	  
    		   
     
   _4939411723264461085+_11450180203511629323 mcAkplAtU_jHRvwupIxCl 	 
    	  
    		  _1705492249324718059.rows mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     
 
  	 
                continue mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
     
 
  	
             mt4oZdscHQLkgCboylyXW 	 
    _192620453243540296 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
     
     
 7 mtmf10UJavtFufEIdYJQE 	 
    	  
    
             mE82ddK08RqkE2ioB87Cg 	 
_3558714157307424502 mleKEARqJ89xIJCH7fsA_ 	_192620453243540296*2+1 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		 
            vector mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
double mCdGviKX2tW09rD2iUjwV 	 
    	   _15505236697521787868 moU7cswum1tEpCFhdz0nH 	 
    	  
_3558714157307424502 mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
     mENURSUbK5YsL6gDFG0XV 	 
    	  
   
            cv mTED_KGydiVM1RbH5g_XM 	 
    	  
   Mat _10158787605014054164 mC4xav6jq4r9qdxV8aAjL 	 
     _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
    0 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		   
 ._8358128829407646415 mhjZhzZncRXafCfJ3w69z 	 
    	 cv mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   Range mhjZhzZncRXafCfJ3w69z 	 
    	 _1588944432958720141-_11450180203511629323,_1588944432958720141+_11450180203511629323 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
   ,cv mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
    		   
     
     
 
  	Range mh_Tf1Nm9McqJoH10VHzf 	_1588944432958720138-_11450180203511629323,_1588944432958720138+_11450180203511629323 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
    mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
     
 
 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
     

             
             mE82ddK08RqkE2ioB87Cg 	 
    	  
    		   
     
     
 
 _1522763743271507664 mleKEARqJ89xIJCH7fsA_ 	 
    std mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
    		   
     
max mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   -_192620453243540296,-_4939411723264461086 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    	 mENURSUbK5YsL6gDFG0XV 	 
    	
             mnZIMdxSe0bwhxrHI7GNX 	 
    _1522768890174422034 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
     
  std mzzUpgIrxS5Tz5DDcg3Pl 	 
    min moU7cswum1tEpCFhdz0nH 	 
 _192620453243540296,_1705492249324718059.cols-1-_4939411723264461086 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
     
  mjA8LcpHiJRrjo1GrHbPY 	 
   
             mDnMnCZpCb7zrOg4iSz03 	 
    	  
    _3817788760850917717 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
     
     
 
 std mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
numeric_limits mkD1iGSJt2oyfwNp4pWsZ 	 double mrzLZmHoLhUzGPqSVSWYQ 	 
    	   mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
  max mji_CmPnVQxlriixIouJp 	 
 mFzprcwGX5TYPR8TCSOsI 	 
   
             mSRiiEeTCByC6NcUUUK2v 	 
    	  
    		   
     
     _1700310074658306154 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		  -1 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
   
             mY6EkjnEEkk2kf3P2CNbA 	 
    	  
    		   
     
     
 int _46082575015037928 mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
     
 _1522763743271507664 mqzaY24A6rPasAO75wW1L 	 
    	  
    		 _46082575015037928 mUoB4yoSYI1E3ym2ZBf33 	 
    	  
 _1522768890174422034 mtmf10UJavtFufEIdYJQE 	 
    	  
    		 _46082575015037928 m_YxQUUHIq8VTevXXDHD_ 	  mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
     
 
  mq5iB_SvYcQOHaxqVDKSC 	 
    	  
    		   
     

                 mnZIMdxSe0bwhxrHI7GNX 	 
  _6806984958533886427 mB1LJDQLgEtQsHykyiRGq 	 
 _46082575015037928+_192620453243540296 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 
  	
                 mrSc6lwF_HlwXZHjcxq5S 	 
    	  
    		   
_175247759696 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
     
_4939411723264461086+_46082575015037928 mxzEpv4Eh_7CpNKRgx4W9 	 
                cv mx_sf_UfonWBXCdYKLjrt 	 
   Mat _3285624572702585059 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
_12800511165451773841 mDklz_JaoOvTrPYxdjKI0 	 
    	 1 mqxiqEVJqEpdcx2wQqpBd 	 ._8358128829407646415 mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
    cv mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
     Range moU7cswum1tEpCFhdz0nH 	 
    	  
    		  _4939411723264461085-_11450180203511629323,_4939411723264461085+_11450180203511629323 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
    ,cv mTED_KGydiVM1RbH5g_XM 	 
    	Range mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
   _175247759696-_11450180203511629323,_175247759696+_11450180203511629323 mdjqSIFIGMPy6TB2p5rXa 	 
    	   mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
      mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
   
                cv mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
     
     
 
Mat _16937296004896083966 mENURSUbK5YsL6gDFG0XV 	 
    	 
                cv mRSJv1dHjU8olKcUlzVQM 	 
    	  
    	absdiff mKzDQjDDFy9nfjTcIGEjn 	 
    _10158787605014054164,_3285624572702585059,_16937296004896083966 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
     
 
 
                 mXzAExIMJoAgjDjzu8D1b 	 
_6175753848998664490 mgd705K7LHsqyMTqQZd5V 	 
    	  
 cv mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		 sum mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
  _16937296004896083966 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
     
 
  mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
     
     
 
  0 mVePGhmhzcrNpULYlc9MA 	 
    	   mENURSUbK5YsL6gDFG0XV 	 

                 mJVqrARa5craS_rpTwFoN 	 
    	  
    		   
     
     
 
  	_6175753848998664490 mAZ9rTzpHdADaRd2Bb691 	 
    	  
    _3817788760850917717 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
  mdkwtZ2sLtp1DPR2xAXUz 	 
    	  
    		   
     
     
 
 
                    _3817788760850917717 mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
     
 
 _6175753848998664490 mtmf10UJavtFufEIdYJQE 	 
    	  
  
                    _1700310074658306154 m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		   
     _6806984958533886427 mcOUtWan0a817s2f7PxBD 	 

                 mUahIcsBNmcUVblDAWprS 	
                _15505236697521787868 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
  _6806984958533886427 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    	 mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
     
 
_6175753848998664490 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
 
             mr6i8va6vrjg52LdPBFvK 	 
    	  
    		   
     
   
             mAdrqP9V1LvJsjA0Aj6ob 	 
    	  
    		   
  _1700310074658306154 mTXGfaSGfbInmGn4DClnj 	 
    	  
    		   
     
_1522763743271507664+_192620453243540296  mwm5CS2rChH_2oro391Od 	 
    	  
    		 _1700310074658306154 mERJ1GNplDMg0yOy8L0ug 	 
    	  
 _1522768890174422034+_192620453243540296 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
  mkMa5jPmx9O_418wkkYBV 	 
    	  
    		   
     
     
 
  	
                
                 mx8JhB_9A0B0jSDMRVJBM 	 
    	  _175247759918 mgd705K7LHsqyMTqQZd5V 	 
    	  _15505236697521787868 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		  _1700310074658306154-1 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		    mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
 
                 mu_wkmRnV5ozD1X_97OUp 	 
    	  
    		   
     
     
 
  	 _175247759917 mEsCokXGzfSkbSNhRJF3n 	 
    	  
_15505236697521787868 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
     
  _1700310074658306154 mvIsUEd5EevSMn8AkFEr2 	 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
     
 
 
                 mx8JhB_9A0B0jSDMRVJBM 	 
    	  
    		   
     
  _175247759916 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
     
     
_15505236697521787868 mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   
     
     
 
 _1700310074658306154+1 mzOhosYdDPvjBVX2OQJqA 	 
     mH0vDS5erwxVyVoypRmvt 	 
    	  

                 mu_wkmRnV5ozD1X_97OUp 	 
    	  
    		   
     
     
 
  	 _16989176769678974579 mC4xav6jq4r9qdxV8aAjL 	 
    	  
   0.5* mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     _175247759918-_175247759916 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    	/ mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     
    _175247759918+_175247759916-2*_175247759917 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     
 +_1700310074658306154-_192620453243540296 mqzaY24A6rPasAO75wW1L 	 
    	  
                 ml1GqNiJhthkEfvXSnaJx 	 
    	 _3378217371725605483 mleKEARqJ89xIJCH7fsA_ 	 
    	  
    _16937465944335025110 mQSqjUxpUR2zqVJEYbq5M 	 
_16992066382187640319 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    		  .pt.x+_16989176769678974579 mxzEpv4Eh_7CpNKRgx4W9 	 
   
                _46082543180066935.depth mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
 _2654435874 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		    mp3ALuatVgPMBt3pB7xx9 	  moU7cswum1tEpCFhdz0nH 	_175247760147.bl*_175247760147.fx mXewpxihHVBVnZgRIIOmS 	 
    	  
    		  mZ2_2tvDK9KkrBAARTPot 	 
    	  
  / mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
 _46082543180066935.und_kpts mqQcJdeZf24ZMXk357zUD 	 
   _2654435874 mzOhosYdDPvjBVX2OQJqA 	 
    	  
    		   
     
  .pt.x-_3378217371725605483 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
     

                _14197860223095419442  mWeo5kjFRZxMUAQe8mmxw 	 
    	 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
    
             mAWVNR_NR1uiYYwpsz1Ic 	 
  
         mXQsUGNgZoukfTO39vkwP 	 
    	  
    		   
     

     mUahIcsBNmcUVblDAWprS 	 
    
  mFlLiaiBJqFiDudfeE9Kh 	 
  
 mIbgaGLt_veu03B1y6TVE 	 
    	  
    		   
FrameExtractor mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
    		   
     
    process_rgbd mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
     
 
  const cv mukxzF5Z4VwhAj2gq2GBE 	 
 Mat &_46082544231248938, const cv mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
    		  Mat &_14382598117525421800,const ImageParams &_175247760147,Frame &_46082543180066935,  mCzeyQHwBRD3aSZNFADGR 	_9933887380370137445  m_f1eqo38yEJGZc_NJV2M 	 
 mq5iB_SvYcQOHaxqVDKSC 	
    _14329080428242784455 mTnJ0tRkJ9I2h7pCRqgNu 	 
1,_46082544231248938,_175247760147 mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
    _17131366770609715580 mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
_12800511165451773841 mqQcJdeZf24ZMXk357zUD 	 
    	  
  0 mzOhosYdDPvjBVX2OQJqA 	 
    	,_46082543180066935,_9933887380370137445 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
  mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     

    
    _46082543180066935.depth.resize mKzDQjDDFy9nfjTcIGEjn 	 
    	 _46082543180066935.und_kpts.size mNGNAekcyVB3BBHwb2S0y 	 
    	  
    		   
     
 mSKLIkoiXh2O2ZJCB5tMN 	 mjA8LcpHiJRrjo1GrHbPY 	 
    
     mZwbWPcaOgtoYVI3Vym0_ 	 
    	  
    		   
     
   size_t _2654435874 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   0 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
     
 
 _2654435874 mERJ1GNplDMg0yOy8L0ug 	 _46082543180066935.depth.size mJsXLE8zWibbVSZkgr7sR 	 
 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
    _2654435874 mCXLyXNIpE5CVT6b8WA0b 	 
    	  
    		   mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		  _46082543180066935.depth mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
 _2654435874 mzOhosYdDPvjBVX2OQJqA 	 
    	  
    		   
     
     mgd705K7LHsqyMTqQZd5V 	 
    	  
    		 0 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
   
    
     mv1v0hP8yXyyMYanjndPT 	 
    	  
    		   
     
     
 
size_t _2654435874 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    0 mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
   _2654435874 mAZ9rTzpHdADaRd2Bb691 	 
    	  
    		   
     
     
 
  _46082543180066935.kpts.size mOuSLzvQYyP5qwR44RZl0 	 
    	 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
 _2654435874 mdzT6psG_86ZMDW_2KJtn 	 
    	  
    		   
     
     
 
  m_f1eqo38yEJGZc_NJV2M 	 
    	  
    	 mkMa5jPmx9O_418wkkYBV 	 
    	  
    		   
    
        
         mrNEGap3YpE0gqVau1IbE 	 
    	  
    		   
  mhjZhzZncRXafCfJ3w69z 	 
_14382598117525421800.at mZmuVLfzgHBU2m2CtqWr0 	 
    	  
    		   
   uint16_t mf12hfREnmnF8LvpyEYPf 	 
    	  
   mjMWFdoUKZIiXkNZ57jQi 	_46082543180066935.kpts mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
   _2654435874 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
      mLwr3ORKxhiYVBNWCtrz3 	 
    	 0   mrnqzGFHPDarlUVk4S3a3 	 
    	  
     mc3Mj6HvensKjoIAWkC9N 	 
    	  
    		   
     
     
 
  	
            _46082543180066935.depth mKFfRfKNAdGHTMXcRWoHV 	 
    	 _2654435874 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
  mB1LJDQLgEtQsHykyiRGq 	 
    	_14382598117525421800.at mGTsC5WBbh9pRfQwp_ngK 	 
    	  
    		   
     
  uint16_t mTXGfaSGfbInmGn4DClnj 	 
    	  
    		   
     
     
 
 mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     _46082543180066935.kpts mqQcJdeZf24ZMXk357zUD 	 
 _2654435874 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		 mTZwdzczLVbxzpWnOdfGa 	 
    	  
*_175247760147.rgb_depthscale mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
 

         mUahIcsBNmcUVblDAWprS 	 
    	  

     mhypoZE0o7J5ZKuuPEU18 	 
    	  
    		   
     
  
 mAOUv187BUnBTJj3ytNVq 	 
    	  
    		   
  
 mu5KEdiJoFgtXEFiJAn7w 	 
  FrameExtractor mukxzF5Z4VwhAj2gq2GBE 	 
    	  
    		   
     process mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    const cv mpU2hymgTA0othpUeyhm3 	 
    	  
    		Mat &_46082544231248938, const ImageParams &_175247760147,Frame &_46082543180066935,  mASg1ouD1BdIe8Qogbe7Q 	 
    	  
    		   
     _9933887380370137445 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     

 msDA0oA68jPtXx6r60msz 	 
    	  
    		  
    _14329080428242784455 mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     _13116459431724108758.kptImageScaleFactor,  _46082544231248938,_175247760147 m_wCH5IAQfazKbdBPiKLv 	 
  mcOUtWan0a817s2f7PxBD 	 
    	  
 
    _17131366770609715580 mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
   _12800511165451773841 mqZiPh2UU3unhdzRfwVIt 	 
    	  
    0 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
 ,_46082543180066935,_9933887380370137445 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
     
  mH0vDS5erwxVyVoypRmvt 	 
    	  
    	
 mAOUv187BUnBTJj3ytNVq 	 
    	 
 mIbgaGLt_veu03B1y6TVE 	 
    	  
    		   
     
     
 
FrameExtractor mzzUpgIrxS5Tz5DDcg3Pl 	 
  _10230054520346001887 mjMWFdoUKZIiXkNZ57jQi 	 
   float _17370277987955713200,const std mukxzF5Z4VwhAj2gq2GBE 	 
    	  
    		   
  vector mNaDEYA4ktSo1tAee0UBy 	 
    	  
    		   
     
     
 cv mzzUpgIrxS5Tz5DDcg3Pl 	 
    	  
    		 Mat mCdGviKX2tW09rD2iUjwV 	 &_3005401535270843804,const ImageParams &_175247760147 mZ2_2tvDK9KkrBAARTPot 	 
   mc3Mj6HvensKjoIAWkC9N 	
     mYQfKuc6wcfjjileYeZRV 	 
    	  
    		   
     uint _2654435874 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    0 mcOUtWan0a817s2f7PxBD 	 
    	  _2654435874 mkWP4IBnPeZGSrAaqSsJL 	 _175247760147.multicams_cs.size msRSLa1rEosf7XXiVm4xR 	 
    	  
    		   
    mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     _2654435874 mvSflanzpnxaAKW5vBjsh 	 
    	   mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
     
 
  	
    _12800511165451773841.resize mrWpmXK1Uy57IaLxMoHw4 	 _3005401535270843804.size mdfitEjJDfX5JRuF8bXel 	 
    	  
    	 mij3FyLvnkUgNUS9rv9RG 	 
    	  
   mqzaY24A6rPasAO75wW1L 	 
    	  
    		  
     mv842vstwttwpSTC53ckv 	 
    	  
    		   
     
uint _2654435874 mwggywncoycNg0ZalVKcE 	 
    	  
   0 mKZSfq5CAVRIjVM6z9udz 	 
    _2654435874 mkWP4IBnPeZGSrAaqSsJL 	 
    	  
    		   
     
     
 
 _3005401535270843804.size mJsXLE8zWibbVSZkgr7sR 	 
  mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
     
 _2654435874 mCXLyXNIpE5CVT6b8WA0b 	 
    	  
    		   
     
     
 
  	  mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
     
 
  	   mdkwtZ2sLtp1DPR2xAXUz 	 
    	  
    		  
         miip_wX0isukeJQaecixC 	 
    	  
    		   _3005401535270843804 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    	_2654435874 mzOhosYdDPvjBVX2OQJqA 	 
    	  
    		   
     .channels mFRlz5c4y22ZxTwvNMTJm 	 
    	  
    		   
    mM1Wr0HMg3CNI1MP7nzg_ 	3 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
     
 
  
            cv mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
     
     cvtColor moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
  _3005401535270843804 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
     
  _2654435874 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
,_12800511165451773841 mDklz_JaoOvTrPYxdjKI0 	 
    	  
    		   
     
_2654435874 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
    ._15530082771795719302,CV_BGR2GRAY mmMWx9cmhAJYDHc6CoPK8 	 
    	  mH0vDS5erwxVyVoypRmvt 	 
    	 
         mI9Gb4vLcvVPjVAV5QsO6 	 
    	  
   _12800511165451773841 mqQcJdeZf24ZMXk357zUD 	 
    	  
    		   
     _2654435874 mmLapJobqTMcWmayld_1p 	 
   ._15530082771795719302 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
     
 
  _3005401535270843804 mqQcJdeZf24ZMXk357zUD 	 
    	  
   _2654435874 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		   
     
     
 
   mFzprcwGX5TYPR8TCSOsI 	 
    
     ml68IJIq89IrD6MxU__3X 	 
    	  
    		   
     
  
    
    _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
     
 0 mqxiqEVJqEpdcx2wQqpBd 	 
    	._15530082765074651952 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
     
     
 
  	_175247760147 mKZSfq5CAVRIjVM6z9udz 	 
 
    _12800511165451773841 mqZiPh2UU3unhdzRfwVIt 	 
    	  
    		   
     
     
 
  	0 mmq78_qMEiQRxdANIEaE2 	 
  ._5505640830793117477 mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
 _175247760147 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		
     miip_wX0isukeJQaecixC 	 
    	  
    		   
     
     
fabs mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
 1-_17370277987955713200 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
 mTXGfaSGfbInmGn4DClnj 	 
    	  
  1e-3 mij3FyLvnkUgNUS9rv9RG 	 
     mXvM0qorf_SpbgDdpYfPu 	 
    	  
    
         mYQfKuc6wcfjjileYeZRV 	 
    	  
  uint _2654435874 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
   0 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
     
   _2654435874 mAZ9rTzpHdADaRd2Bb691 	 
    	  
    		   
     
     
 
  	 _3005401535270843804.size mNGNAekcyVB3BBHwb2S0y 	 
    	  
   mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
 
  _2654435874 mYynacvMOsJPdrHyZrqLt 	 
    	  
    		   
     mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     
 
  mMdt4NW72l_xAW2a_7MaH 	 
    	
            cv mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
     
     
 
Size _175247759442 mVQ8i37hpNkIkaxXGHhrO 	 
    	  
 _3005401535270843804 mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		_2654435874 m_f10xp9iBg5cy1Elj1NV 	 
    	.cols*_17370277987955713200,_3005401535270843804 mqZiPh2UU3unhdzRfwVIt 	 
    	  
    		   
     
     
 
  _2654435874 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
     
     .rows*_17370277987955713200 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
  mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
     
            
             mjoRivKB9IaaWetnXIp3t 	 
    	  
    		   
     
     
 
_175247759442.width%4 mNH8XnJPeJT8g0S6ijYW3 	 
 0 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     
                _175247759442.width mLjPrJR_wSFaBK8AA8TWf 	 
    	  
    		   
 4-_175247759442.width%4 mFzprcwGX5TYPR8TCSOsI 	 
    	  
             mmBFyV0pRbQZSqu8LEMmC 	 
    	  
    		   
_175247759442.height%2 motqoD9yEu959l6_lYdsf 	 
    	  
    		   
     
     
 
  	0 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
     _175247759442.height mDbODQ_pk1aeu1UegcTMF 	 
    	  
    		   
     
     
 
 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
    
            cv mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
     
     resize mrWpmXK1Uy57IaLxMoHw4 	 
    	  
   _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
   _2654435874 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
 ._15530082771795719302,_12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    		   
  _2654435874 mHpDQ4y94Jd3ehaxDmYhl 	 
  ._8358128829407646415,_175247759442 mrnqzGFHPDarlUVk4S3a3 	 
    	  
   mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
   
            _12800511165451773841 mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   
   0 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
  ._5505640830793117477.resize mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     
     
 
  	 _175247759442, _2654435874 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
  mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    
            _12800511165451773841 mDklz_JaoOvTrPYxdjKI0 	 
    	 _2654435874 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  ._6972553715263421613.first mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
  float mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
     
     
 
_175247759442.height mZ2_2tvDK9KkrBAARTPot 	 
    	/float moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     
    _3005401535270843804 mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		   
     
     
 
  	_2654435874 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
   .rows mSSFbqZ6suzyqBFuTOK8y 	   mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
     

            _12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
    _2654435874 mmq78_qMEiQRxdANIEaE2 	 
._6972553715263421613.second mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
      float mVQ8i37hpNkIkaxXGHhrO 	_175247759442.width mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
  /float moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     
     
_3005401535270843804 mqQcJdeZf24ZMXk357zUD 	 
_2654435874 mHnx0MhdU3oTzuyIpVbjV 	 
   .cols m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
     
 
    mtmf10UJavtFufEIdYJQE 	 
 
         mAOUv187BUnBTJj3ytNVq 	 
    	  
    
     mAOUv187BUnBTJj3ytNVq 	 
    	  
    		   

    else msDA0oA68jPtXx6r60msz 	 
    	  
    		
         mID3KGvR2YcYnpT7Ek1mJ 	 
    	  
    		   
     
     
 
  	 uint _2654435874 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
   0 mcOUtWan0a817s2f7PxBD 	 
 _2654435874 mZmuVLfzgHBU2m2CtqWr0 	 
   _12800511165451773841.size mFRlz5c4y22ZxTwvNMTJm 	 
    	  
    		   
     
     
 mFzprcwGX5TYPR8TCSOsI 	 
    	  
  _2654435874 mWeo5kjFRZxMUAQe8mmxw 	 
    	  
    		   
     
    mZ2_2tvDK9KkrBAARTPot 	 mGwSL4y5H5STcCbNqgDkd 	 
            _12800511165451773841 mqQcJdeZf24ZMXk357zUD 	 
    	  
    		   
     _2654435874 mmq78_qMEiQRxdANIEaE2 	 
    	  
   ._8358128829407646415 mCPc29blbqZ5t89uMSE7C 	 
    	  
    		   
     
    _12800511165451773841 mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   _2654435874 mHnx0MhdU3oTzuyIpVbjV 	 
    ._15530082771795719302 mcOUtWan0a817s2f7PxBD 	 
    	  
    		 
            _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
     
     
 _2654435874 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		   ._6972553715263421613 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
     
     
 std mTED_KGydiVM1RbH5g_XM 	 
    	  
  pair mkD1iGSJt2oyfwNp4pWsZ 	 
    	  float,float mq3NMzP75aSEvua42dKaz 	 
    	  
    		   
     
     
  mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     
     
 1,1 mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
    mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
   
         mikfDmb676vn6P2J8crTL 	 
    	  
    		  
     mAOUv187BUnBTJj3ytNVq 	 
    
    

     mBpxihTLI7mWa33lKHm6o 	 
    	  
    		   
     
   _706246336197678  mwggywncoycNg0ZalVKcE 	 
    	    mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
 _12800511165451773841 mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   0 mHnx0MhdU3oTzuyIpVbjV 	._15530082765074651952.fx mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     0 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
+_12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	 0 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
 ._15530082765074651952.fy mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		   
     
     
0 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		  mSKLIkoiXh2O2ZJCB5tMN 	 
 / mh_Tf1Nm9McqJoH10VHzf 	 
    _12800511165451773841 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
  0 mqxiqEVJqEpdcx2wQqpBd 	._15530082765074651952.fx mrWpmXK1Uy57IaLxMoHw4 	 
    	  
   1 m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
 +_12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
    	  
    		0 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    	._15530082765074651952.fy mKzDQjDDFy9nfjTcIGEjn 	 
    	  
1 mrnqzGFHPDarlUVk4S3a3 	 
    mmMWx9cmhAJYDHc6CoPK8 	 
     mtmf10UJavtFufEIdYJQE 	 
    	  
    		   

     mjoRivKB9IaaWetnXIp3t 	 
    	  
    		   
     
     
 
_706246336197678 mNH8XnJPeJT8g0S6ijYW3 	 
  1 mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		  
     mf93KHE75PL0YvPjf3hDs 	 
    	  
    		   
     
     

         mX7P4s1_g7S0Vw9Y54fYj 	 
  _11093822060392 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
  1 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		 
         miip_wX0isukeJQaecixC 	 
    	  
    		   
     
     
 
 _706246336197678 mCdGviKX2tW09rD2iUjwV 	 
    	  
    		   
     
     
 
  	1 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
  msDA0oA68jPtXx6r60msz 	 
    	  
    		   
  
            _11093822060392 m_2dgqWMgdoegqwSE6Fwl 	 
    	  0 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   

            _706246336197678  mwggywncoycNg0ZalVKcE 	  1.0/_706246336197678 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
 
 
         mhypoZE0o7J5ZKuuPEU18 	 
    	  
   
        
        cv mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		   
 Size _175247759442 mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
  _12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    		   
    _11093822060392 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		 ._8358128829407646415.cols*_706246336197678, _12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
 _11093822060392 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
     
._8358128829407646415.rows*_706246336197678 mrnqzGFHPDarlUVk4S3a3 	 
    	  
  mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   

        cv mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		resize mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		 _11093822060392 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
     
  ._8358128829407646415,_12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   
     
     
 
  	 _11093822060392 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
     ._8358128829407646415,_175247759442 m_wCH5IAQfazKbdBPiKLv 	 
    	  
 mFzprcwGX5TYPR8TCSOsI 	 
    	  
 
        _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    _11093822060392 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
._6972553715263421613.first mp3ALuatVgPMBt3pB7xx9 	 
    	  
   float mCNsgngMBa8r2GTvALgCE 	 
   _175247759442.height mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
 /float mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    	_3005401535270843804 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
    _11093822060392 mvIsUEd5EevSMn8AkFEr2 	 
.rows m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		   
     
     
 
  	   mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
   
        _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
     
_11093822060392 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    		   
     
   ._6972553715263421613.second mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
     
 
   float moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
 _175247759442.width m_f1eqo38yEJGZc_NJV2M 	/float mCNsgngMBa8r2GTvALgCE 	 
    	  
   _3005401535270843804 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
_11093822060392 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
.cols m_wCH5IAQfazKbdBPiKLv 	 
    	  
    		    mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
   
        _12800511165451773841 mKFfRfKNAdGHTMXcRWoHV 	 
    	  
    		   
     
   0 mmq78_qMEiQRxdANIEaE2 	 
    	  
    		   
     
     
 
 ._5505640830793117477.resize mCNsgngMBa8r2GTvALgCE 	 
    	_175247759442, _11093822060392 mij3FyLvnkUgNUS9rv9RG 	 
    	  
     mQfS9tiOvGsXX9DD53UoY 	 
    	  
   
     mFlLiaiBJqFiDudfeE9Kh 	 
    	  
    		   
     
     
 
 mAOUv187BUnBTJj3ytNVq 	 
  
 mu5KEdiJoFgtXEFiJAn7w 	 
    	  
    		   
     
     
 FrameExtractor mRSJv1dHjU8olKcUlzVQM 	 
_14329080428242784455 mVQ8i37hpNkIkaxXGHhrO 	 
    float _17370277987955713200,const cv mTED_KGydiVM1RbH5g_XM 	Mat &_11093822404769,const ImageParams &_175247760147,const cv mukxzF5Z4VwhAj2gq2GBE 	 
    	  
Mat &_11093822404770 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
 mSnypOnN3BGb0Sf99pqCF 	 
    	  
    		   
     
    
     mt4oZdscHQLkgCboylyXW 	 
    	  
    		   
 _175247759468 mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
     
 
 1 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
     
     
 

     mOb7vP5ncyxcZGeJJnrwa 	 
    	  
 mmvgClZ1llA3_d9E17fsk 	 _11093822404770.empty mji_CmPnVQxlriixIouJp 	 
    	  
    		   
   mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
   
        _175247759468 mvyz9F7m7Z76dYn99FG0x 	 
    	  
    		   
     
     
 
   mENURSUbK5YsL6gDFG0XV 	 
    	  
    	
    _12800511165451773841.resize mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
   _175247759468 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     mjA8LcpHiJRrjo1GrHbPY 	 
    	  
  
     mCZX10THppqeTgSrqysBt 	 
   _11093822404769.channels mji_CmPnVQxlriixIouJp 	 
    	  
    		   
     mggO2nUknv36WZKdfBBvE 	 
    	  
    		   
     3 m_wCH5IAQfazKbdBPiKLv 	 
    	
        cv mpU2hymgTA0othpUeyhm3 	 
    	  
    		   
  cvtColor mhjZhzZncRXafCfJ3w69z 	 _11093822404769,_12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
0 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
     
     
._15530082771795719302,CV_BGR2GRAY m_f1eqo38yEJGZc_NJV2M 	 
    mKZSfq5CAVRIjVM6z9udz 	 
   
     mFcva5z1yM6KI3I8OsNe3 	 
    	  
    		   
     
    _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		  0 mzOhosYdDPvjBVX2OQJqA 	 ._15530082771795719302 mwggywncoycNg0ZalVKcE 	 
    	  
    	_11093822404769 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
 
  
    _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    0 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		   
     
   ._15530082765074651952 mgd705K7LHsqyMTqQZd5V 	 
    _175247760147 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
  
     mOb7vP5ncyxcZGeJJnrwa 	 mwh0H0CH8hbXoUMeW0njl 	 
    	  
    		 _11093822404770.empty mwoYGv1lf3LjTo33qtGck 	 
  mSSFbqZ6suzyqBFuTOK8y 	
     mdkwtZ2sLtp1DPR2xAXUz 	 
    	  
    
         mmq9HNT6SMSK3FI_mJgeB 	 
  _11093822404770.channels mNGNAekcyVB3BBHwb2S0y 	 
    	  mHeJOL9dAIju_MrKHZ2Lu 	 
    	  3 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   

            cv mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		   
  cvtColor mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		  _11093822404770,_12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    1 mqxiqEVJqEpdcx2wQqpBd 	 
   ._15530082771795719302,CV_BGR2GRAY mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     

         mrPVVe4FvkR2o3oMuRPIt 	 
    	  
     _12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
     
     
 
 1 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
._15530082771795719302 mf7uQnLk3PYBBCuntis6S 	 
    	  
    	_11093822404770 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
        _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
     
     
 
  1 mvIsUEd5EevSMn8AkFEr2 	 
    	  ._15530082765074651952 mwggywncoycNg0ZalVKcE 	 
    	  
    		   
     
    _175247760147 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
 
     mr6i8va6vrjg52LdPBFvK 	 
    	  
    		   

    
    
     mOb7vP5ncyxcZGeJJnrwa 	fabs moU7cswum1tEpCFhdz0nH 	 
    1-_17370277987955713200 mTZwdzczLVbxzpWnOdfGa 	 
    	  msuASqXEqahJgXQfOZ6MX 	 
    	  
    		   
     
1e-3     mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     
 
   msDA0oA68jPtXx6r60msz 	 
    	  
  
        cv mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
   Size _175247759442 mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
  _11093822404769.cols*_17370277987955713200,_11093822404769.rows*_17370277987955713200 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
     
 
  mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
        
         mJVqrARa5craS_rpTwFoN 	 
    	  
    		   
     
_175247759442.width%4 motqoD9yEu959l6_lYdsf 	0 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
     
 
  	 
            _175247759442.width mxVL059Rj7sAVxu8ZQrac 	 
    	  
    	4-_175247759442.width%4 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     

         mOb7vP5ncyxcZGeJJnrwa 	 
    	  
_175247759442.height%2 motqoD9yEu959l6_lYdsf 	0 mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
 _175247759442.height mx1eIclnlkObQ3brbbK1m 	 
    	  
    		   
     
 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   

        cv mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
     
 
 resize mhjZhzZncRXafCfJ3w69z 	_12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  0 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
     ._15530082771795719302,_12800511165451773841 mDklz_JaoOvTrPYxdjKI0 	 
    	  
   0 mmq78_qMEiQRxdANIEaE2 	 
    	  
 ._8358128829407646415,_175247759442 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		    mqzaY24A6rPasAO75wW1L 	 
    	 
        _12800511165451773841 mqQcJdeZf24ZMXk357zUD 	 
    	  
    0 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
     
     
._5505640830793117477 mEsCokXGzfSkbSNhRJF3n 	 _12800511165451773841 mqQcJdeZf24ZMXk357zUD 	0 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
     
._15530082765074651952 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		  
        _12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
    	  
0 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
    ._5505640830793117477.resize mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		_175247759442 mmMWx9cmhAJYDHc6CoPK8 	 
 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
     
 
  	
        _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    	  
  0 mUmS87lLj_Mf5SpVtoLqH 	 
    	  ._6972553715263421613.first mwggywncoycNg0ZalVKcE 	 
    	  
    		    float mjMWFdoUKZIiXkNZ57jQi 	 
    	  
    		   
     
     
 _175247759442.height m_f1eqo38yEJGZc_NJV2M 	 
/float mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    _11093822404769.rows mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
     
     mqzaY24A6rPasAO75wW1L 	 
    	  
    		  
        _12800511165451773841 mKFfRfKNAdGHTMXcRWoHV 	 
0 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    	._6972553715263421613.second mC4xav6jq4r9qdxV8aAjL 	 
    	  
     float moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     
     
 
 _175247759442.width mij3FyLvnkUgNUS9rv9RG 	 
    	  
   /float mCNsgngMBa8r2GTvALgCE 	 
    	  
    		  _11093822404769.cols mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
   mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
  
         mjoRivKB9IaaWetnXIp3t 	 
 miJo4F2j3dkD8omqvp1B8 	 
    	  
    		   
    _11093822404770.empty mji_CmPnVQxlriixIouJp 	 
    	  
    		   
   mij3FyLvnkUgNUS9rv9RG 	 
    	  mq5iB_SvYcQOHaxqVDKSC 	 
    	  
    		   
     
     
 
  	
            cv mTED_KGydiVM1RbH5g_XM 	 
    	resize mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
     
 
  	_12800511165451773841 mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		   
     
   1 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		   
     
     
._15530082771795719302,_12800511165451773841 mDklz_JaoOvTrPYxdjKI0 	 
    	  
    		   
     
     
 
1 mvIsUEd5EevSMn8AkFEr2 	 
    	  
._8358128829407646415,_175247759442 mTZwdzczLVbxzpWnOdfGa 	 
    	   mQfS9tiOvGsXX9DD53UoY 	 
    	  
            _12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    		   
     
     
 
1 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		 ._5505640830793117477 mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
   _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
   2 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		   
     
     
 
  	 ._15530082765074651952 mtmf10UJavtFufEIdYJQE 	
            _12800511165451773841 mV0JUzxDJGEJD_uX2w3nt 	 
    1 mzOhosYdDPvjBVX2OQJqA 	 
    	._5505640830793117477.resize mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     
     _175247759442 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
            _12800511165451773841 mDklz_JaoOvTrPYxdjKI0 	 
    	  
    		   
     
     
1 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    ._6972553715263421613.first mp3ALuatVgPMBt3pB7xx9 	 
   float mrWpmXK1Uy57IaLxMoHw4 	 
  _175247759442.height mij3FyLvnkUgNUS9rv9RG 	 
    	  
  /float mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
     
_11093822404770.rows mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
  mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     
     
            _12800511165451773841 mqQcJdeZf24ZMXk357zUD 	 
   1 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		   
     
._6972553715263421613.second mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
   float mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		  _175247759442.width mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
     
    /float mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
   _11093822404770.cols mSSFbqZ6suzyqBFuTOK8y 	 
    	  
   mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
     

         mUahIcsBNmcUVblDAWprS 	
     mikfDmb676vn6P2J8crTL 	 
    	  
    		   
     
 
    else mkMa5jPmx9O_418wkkYBV 	 
    	  
    		   
     
         mv842vstwttwpSTC53ckv 	 
   int _2654435874 mCPc29blbqZ5t89uMSE7C 	 0 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
   _2654435874 mZmuVLfzgHBU2m2CtqWr0 	 
    	  
    		   
     
     
_12800511165451773841.size mji_CmPnVQxlriixIouJp 	 
    	  
    		   
     mxzEpv4Eh_7CpNKRgx4W9 	 
    _2654435874 mDbODQ_pk1aeu1UegcTMF 	 
    	  
    		   
     
     mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   msDA0oA68jPtXx6r60msz 	 
    	  
    		   
     

            _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
   _2654435874 m_f10xp9iBg5cy1Elj1NV 	 
    	  
   ._8358128829407646415 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
     _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
     
     
 
  _2654435874 mzOhosYdDPvjBVX2OQJqA 	 
    	  
    		   
     
     
 
  ._15530082771795719302 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    	
            _12800511165451773841 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
_2654435874 mmq78_qMEiQRxdANIEaE2 	 
    	  
    		   
     
     
 
  	 ._5505640830793117477 mCPc29blbqZ5t89uMSE7C 	 
  _12800511165451773841 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    		   
     
     
 
  	 _2654435874 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
     
 
  	._15530082765074651952 mENURSUbK5YsL6gDFG0XV 	 

            _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    	_2654435874 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
     
 ._6972553715263421613 mCPc29blbqZ5t89uMSE7C 	 
    	  std mpU2hymgTA0othpUeyhm3 	 
    	 pair mkD1iGSJt2oyfwNp4pWsZ 	 
    float,float mO3cy6qBtctqMh9v1n6AM 	 
    	  
    		   
    mh_Tf1Nm9McqJoH10VHzf 	 
    	  
    		   
1,1 m_f1eqo38yEJGZc_NJV2M 	  mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
     
 
  	 
         mFlLiaiBJqFiDudfeE9Kh 	 
    	  
    		   
     
     
 
  	
     mAOUv187BUnBTJj3ytNVq 	 
    	 
  mAOUv187BUnBTJj3ytNVq 	 
    	  
   
 mu5KEdiJoFgtXEFiJAn7w 	 
    	  
    	FrameExtractor mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
   _17131366770609715580 mrWpmXK1Uy57IaLxMoHw4 	 
    	  
    		const _1921178215606755952 &_46082576028426834,   Frame &_46082543180066935,  mASg1ouD1BdIe8Qogbe7Q 	 
    	 _9933887380370137445 mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
     
 
   mf93KHE75PL0YvPjf3hDs 	 
    	  
    		 
    _46082543180066935.clear msRSLa1rEosf7XXiVm4xR 	 
    	  
    		   
     
     
  mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
    vector mkD1iGSJt2oyfwNp4pWsZ 	 
    	  
    		   
   cv mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
     
 
KeyPoint mTXGfaSGfbInmGn4DClnj 	 
    	  
    		   
      _9811310495564694013 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
 
    std mNmJ6yxo1KXO341_e1pHs 	 
    	  
    	thread _15593584088352382451 mVQ8i37hpNkIkaxXGHhrO 	 
    mOUmw6CStUwR2yTJByg62 	 
    	  
    		& mHpDQ4y94Jd3ehaxDmYhl 	 
    	  
    		   
     
      mGwSL4y5H5STcCbNqgDkd 	 
    	  
    		   
     
     
 
  	
         mDtPKfWKFQ_J_7DQBz07x 	 
    	  
   _4309197024622458338 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
  mf93KHE75PL0YvPjf3hDs 	 
    	  
    		   
     
 
            _8033463663468506753 mZpLMilyO0bRxIH38sCpH 	 
    	  detectAndCompute mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
   _12800511165451773841 mqQcJdeZf24ZMXk357zUD 	 
    	  
    		   
     
     
 
  	 0 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		   
     
     
 
  	 ._8358128829407646415,cv mpU2hymgTA0othpUeyhm3 	 
    	  
    		   
     
     
 
Mat mXewpxihHVBVnZgRIIOmS 	 
 ,_9811310495564694013,_46082543180066935.desc,_10675870925382111478 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
      mQfS9tiOvGsXX9DD53UoY 	 
    	  
    
            _46082543180066935.KpDescType mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
   _8033463663468506753 mKWQiGcI03yFaWIZgwzoA 	 
    	  
    		   
    getDescriptorType maJhscyEMQhI3OBH5VVm0 	 
    	  
    		   
     
     
 
   mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 

         mUahIcsBNmcUVblDAWprS 	 
    	  
    		 
     mikfDmb676vn6P2J8crTL 	 
    	  
    		   
     
     
 
 mTZwdzczLVbxzpWnOdfGa 	 
    	  
   mjA8LcpHiJRrjo1GrHbPY 	 

    std mEYXB1C9jxDk6pOX0KXQ0 	 
    	  
  thread _5221495899814560498 mhjZhzZncRXafCfJ3w69z 	 
    	  
    		   
     
      mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		   
     & mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		    mMdt4NW72l_xAW2a_7MaH 	 
 
         mVKRBrgbb_3KvAoVy7Y76 	 
    	  
    		   
 mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
     
_3566717627060593117 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
     
     mq5iB_SvYcQOHaxqVDKSC 	
             mYwwnUnHpluL2mXrXBOkk 	 
    	  
    	_6807036698426592110 mp3ALuatVgPMBt3pB7xx9 	 
    	  
    		   
     
_8000946946827829134 mQBJhEhGAPkNmt9WeIGXo 	 
    	  
    		  detect mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
  _46082576028426834._15530082771795719302 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   mENURSUbK5YsL6gDFG0XV 	 
    	  
    		   
 
             mYQfKuc6wcfjjileYeZRV 	 
    	  
    		   const auto&_2654435878:_6807036698426592110 m_f1eqo38yEJGZc_NJV2M 	 
    	  
   mSnypOnN3BGb0Sf99pqCF 	 
    	  

                ucoslam mRSJv1dHjU8olKcUlzVQM 	 
    	  
    		   
     
MarkerObservation _8214686538440707422 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
     
 
  	
                _8214686538440707422.id mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
     
 
_2654435878.id mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
     
     
 
  	
                _8214686538440707422.points3d mC4xav6jq4r9qdxV8aAjL 	_2654435878.points3d mH0vDS5erwxVyVoypRmvt 	 
    	
                _8214686538440707422.corners m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		   
    _2654435878.corners mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
 
                _8214686538440707422.dict_info mC4xav6jq4r9qdxV8aAjL 	 
    	  
    		   
     
     
 
 _2654435878.info mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
                 mY4CnwjxIThh4pzZvvLJZ 	 
    	  
    		   
     
  _706246330434227 mf7uQnLk3PYBBCuntis6S 	 
    	  
IPPE mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		  solvePnP_ mxd1ooOS_bL0_Uh1bOsx8 	 
    _2654435878.points3d ,_2654435878.corners, _46082576028426834._15530082765074651952.CameraMatrix,_46082576028426834._15530082765074651952.Distorsion mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
  mqzaY24A6rPasAO75wW1L 	 
    	  

                 mGGEvKPIuZbqGUag8vVEx 	 
    	  
    		   
     
     
int _2654435866 mp3ALuatVgPMBt3pB7xx9 	0 mFzprcwGX5TYPR8TCSOsI 	_2654435866 mzfVZbevdXZt1lQmnk8So 	 
    	  
    		2 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
     
 
  	 _2654435866 mvSflanzpnxaAKW5vBjsh 	 
 mrnqzGFHPDarlUVk4S3a3 	 
    	  
   mSnypOnN3BGb0Sf99pqCF 	 
    	  
    	
                    _8214686538440707422.poses.errs mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
   _2654435866 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
     
 
  mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
     _706246330434227 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    		   
     
  _2654435866 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
.second mENURSUbK5YsL6gDFG0XV 	 
   
                    _8214686538440707422.poses.sols mqO12iB0NH1UmNi6cKZzy 	 
    	  
 _2654435866 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
     mB1LJDQLgEtQsHykyiRGq 	 
    	  
    		   
     
   _706246330434227 mGDXomYjoJafW8z9K6m1P 	 
    	  
    		   
 _2654435866 mHpDQ4y94Jd3ehaxDmYhl 	 
    	  .first.clone mXewpxihHVBVnZgRIIOmS 	 
    	  
    		   
     
     
 
  	 mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 
 
                 mUahIcsBNmcUVblDAWprS 	 
    	  
    		   
     
     

                _8214686538440707422.poses.err_ratio mC4xav6jq4r9qdxV8aAjL 	 
    	  
    		   
     
     _706246330434227 mDklz_JaoOvTrPYxdjKI0 	 
1 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
     
     
 
 .second/_706246330434227 mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  0 mvIsUEd5EevSMn8AkFEr2 	 
    	  
    		.second mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
    
                 mrraePCBsgi4fdXgdYTXy 	 
    	  
auto &_2654435868:_8214686538440707422.corners mSSFbqZ6suzyqBFuTOK8y 	 
    	  
   mq5iB_SvYcQOHaxqVDKSC 	 
    	  
    		   
     
     
 
  	
                    _2654435868.x mKwpdOP7LA2TzewlweD6_ 	 
    	  
    		   
     
     _46082576028426834._6972553715263421613.first mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		 
                    _2654435868.y mzD8x50gAKMFXpn1Kk7ZP 	 
    	  
    		   
    _46082576028426834._6972553715263421613.second mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
    
                 mFlLiaiBJqFiDudfeE9Kh 	 
    	  
    		   
     
    
                _46082543180066935.markers.push_back moU7cswum1tEpCFhdz0nH 	 
    	  
_8214686538440707422 mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
     
 
  	  mjA8LcpHiJRrjo1GrHbPY 	
             mXQsUGNgZoukfTO39vkwP 	 
    	  
    		   
         mr6i8va6vrjg52LdPBFvK 	 
    	  
    		  
     ml68IJIq89IrD6MxU__3X 	 
    	  
    		   
     
    
     mdjqSIFIGMPy6TB2p5rXa 	 
    	  
   mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
 
    _15593584088352382451.join mNGNAekcyVB3BBHwb2S0y 	 
    	  
  mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
    
    _5221495899814560498.join mOuSLzvQYyP5qwR44RZl0 	 
    	  
    		   
     
   mQfS9tiOvGsXX9DD53UoY 	 
    	  
 
     meCjWleY3Dl205itM_ZqV 	 
    	  
    mjMWFdoUKZIiXkNZ57jQi 	 
    	  
  debug mTED_KGydiVM1RbH5g_XM 	 
   Debug mNmJ6yxo1KXO341_e1pHs 	 
    getLevel mwoYGv1lf3LjTo33qtGck 	 
    	  
    		   
     
     
 
  	  mhQdIzwRajAQGMZcowBBZ 	 
    	  
    		   
  100 mQW1qj2eB_butTzRBCZ8q 	 
    	  
    		   
     
     
 
  _13116459431724108758.saveImageInMap mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
   mq5iB_SvYcQOHaxqVDKSC 	 
    	  
    		
            
            std mx_sf_UfonWBXCdYKLjrt 	 
    	  
 vector mZmuVLfzgHBU2m2CtqWr0 	 
    	  
    		   
     
     
 
  	uchar mq3NMzP75aSEvua42dKaz 	 
    	  
    		   _11093821971029 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
            cv mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
     
     
 
imencode mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
  "\x2e\x6a\x70\x67",_46082576028426834._15530082771795719302,_11093821971029, msDA0oA68jPtXx6r60msz 	 
    	  
cv mZauRjZulqrGATSW8brQM 	 
    	  
    		   
   IMWRITE_JPEG_QUALITY,90 mikfDmb676vn6P2J8crTL 	 
    	  
    		   
     
   mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
  mQfS9tiOvGsXX9DD53UoY 	 
    	  
 
            _46082543180066935.jpeg_buffer.create mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
   1,_11093821971029.size mJsXLE8zWibbVSZkgr7sR 	 
    	  
    		   
     
     
 
  	,CV_8UC1 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
 
            mempcpy mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    	_46082543180066935.jpeg_buffer.ptr mkD1iGSJt2oyfwNp4pWsZ 	 
    	  
    		 uchar mrzLZmHoLhUzGPqSVSWYQ 	 moU7cswum1tEpCFhdz0nH 	 
    	  
    		   
     0 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
,&_11093821971029 mqQcJdeZf24ZMXk357zUD 	 0 mvIsUEd5EevSMn8AkFEr2 	 
    	,_11093821971029.size mwoYGv1lf3LjTo33qtGck 	 
    	  
  mZ2_2tvDK9KkrBAARTPot 	 
    	  
    		   
     
     
 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
     
 
  	
     mXQsUGNgZoukfTO39vkwP 	 
    	  
    		   
     
     
 
  
    
    
    _46082543180066935.scaleFactors.resize mxd1ooOS_bL0_Uh1bOsx8 	 
    	  
    		   
     
_8033463663468506753 mVlIk0ncU1k0KblI7kXJb 	 
    	  
    		   
     
     
 
  getParams mJsXLE8zWibbVSZkgr7sR 	.nOctaveLevels mdjqSIFIGMPy6TB2p5rXa 	 
    	  
     mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		 
     mLKt8Vq7dlkHx_FS_4ubV 	 
    	  
    		   
     
 _175247759750 mEsCokXGzfSkbSNhRJF3n 	 
    	_8033463663468506753 mKWQiGcI03yFaWIZgwzoA 	 getParams mOuSLzvQYyP5qwR44RZl0 	 
    	  
    		   
   .scaleFactor mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     

    _46082543180066935.scaleFactors mfqY1vAKV8IyrQ6Hp8nMV 	 
  0 mHnx0MhdU3oTzuyIpVbjV 	 
    	  
    		   
     
     mEsCokXGzfSkbSNhRJF3n 	 
1 mtmf10UJavtFufEIdYJQE 	 
    	 
     mv842vstwttwpSTC53ckv 	 
    	  
    		   
     
     
 
  	size_t _2654435874 mwggywncoycNg0ZalVKcE 	 
    	  
    		   
     
 1 mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
_2654435874 mfLTBWN12E1Fpsr4SddaU 	 
    	  
  _46082543180066935.scaleFactors.size mdfitEjJDfX5JRuF8bXel 	 
  mKZSfq5CAVRIjVM6z9udz 	 
    _2654435874 mx1eIclnlkObQ3brbbK1m 	 
    	  
    		   
     
     
  mdjqSIFIGMPy6TB2p5rXa 	 
        _46082543180066935.scaleFactors mfqY1vAKV8IyrQ6Hp8nMV 	 
    	  
    		   
     
   _2654435874 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   
     
     
 
  	  mf7uQnLk3PYBBCuntis6S 	 
    	  
    		   
     
     
 
  _46082543180066935.scaleFactors mKFfRfKNAdGHTMXcRWoHV 	 
    _2654435874-1 mqxiqEVJqEpdcx2wQqpBd 	 
    	  
    		   
     
  *_175247759750 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
 
  
    
     mpgp127ninlZ4DNI4hAoo 	 
    	  
    		    mVQ8i37hpNkIkaxXGHhrO 	 
    	 _9811310495564694013.size msRSLa1rEosf7XXiVm4xR 	 
    	  
    		   
     
     
  mO3cy6qBtctqMh9v1n6AM 	 
    	  
    		   
   0 mTZwdzczLVbxzpWnOdfGa 	 
    	  
    		   
     
     
 
   mMdt4NW72l_xAW2a_7MaH 	 
    	  
    		   
     
    
        vector mERJ1GNplDMg0yOy8L0ug 	 
    	  
    		  cv mZauRjZulqrGATSW8brQM 	 
    	  
    		   
     
 Point2f mTXGfaSGfbInmGn4DClnj 	 
    	  
    		   
     
     
 
  _11093822294365 mENURSUbK5YsL6gDFG0XV 	 
    	 _11093822294365.reserve mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
_9811310495564694013.size mNGNAekcyVB3BBHwb2S0y 	 
    	  
    		   
     
      mSKLIkoiXh2O2ZJCB5tMN 	 
    	  
    		   
 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
    
         mID3KGvR2YcYnpT7Ek1mJ 	 
    	  
    		   
     
    auto _2654435881:_9811310495564694013 mdjqSIFIGMPy6TB2p5rXa 	 
    	  
    		   
     
     _11093822294365.push_back mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
 _2654435881.pt mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   mtmf10UJavtFufEIdYJQE 	 
    	  
    
        _12800511165451773841 mqO12iB0NH1UmNi6cKZzy 	 
    	  
    		   0 mUmS87lLj_Mf5SpVtoLqH 	 
._5505640830793117477.undistortPoints mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
     
 _11093822294365  mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		   
     
     
 
  	  mH0vDS5erwxVyVoypRmvt 	 
    	  
    		   
     
     
        _46082543180066935.und_kpts mCPc29blbqZ5t89uMSE7C 	 
    	  
    		 _9811310495564694013 mtmf10UJavtFufEIdYJQE 	 
    	  
   
        _46082543180066935.kpts.resize mh_Tf1Nm9McqJoH10VHzf 	 
    	  
 _9811310495564694013.size maJhscyEMQhI3OBH5VVm0 	 
    	  
    		   
 mmMWx9cmhAJYDHc6CoPK8 	 
    	  
    		 mcOUtWan0a817s2f7PxBD 	 
    	  
    		   

         mXpWjd6Da2oJAVgzWlzrH 	 
    	  
    		   
  mVQ8i37hpNkIkaxXGHhrO 	 
    	  
    		   
     
     
 
 size_t _2654435874 mB1LJDQLgEtQsHykyiRGq 	0 mKZSfq5CAVRIjVM6z9udz 	 
    	  
    		   
    _2654435874 mfLTBWN12E1Fpsr4SddaU 	 
    	  
    		   
    _9811310495564694013.size mXewpxihHVBVnZgRIIOmS 	 
    	  
    		   
     
     
 
  	 mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
     
     
 
  	 _2654435874 mvSflanzpnxaAKW5vBjsh 	 
    	  
    		   
   mZ2_2tvDK9KkrBAARTPot 	 
    	   mq5iB_SvYcQOHaxqVDKSC 	 
    	  
    		   
     
     
 
  
            _46082543180066935.kpts mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   
     
     
 
  	_2654435874 mmq78_qMEiQRxdANIEaE2 	 
    	  
  mB1LJDQLgEtQsHykyiRGq 	 
    	  
 _9811310495564694013 mQSqjUxpUR2zqVJEYbq5M 	 
    	  
    	_2654435874 mmLapJobqTMcWmayld_1p 	 
    	  
    		   
     
     
.pt mQfS9tiOvGsXX9DD53UoY 	 
    	  
    		   
     
     
 
  
            _46082543180066935.und_kpts mOUmw6CStUwR2yTJByg62 	_2654435874 mzOhosYdDPvjBVX2OQJqA 	 
    	  
    		   
     
     
 .pt mC4xav6jq4r9qdxV8aAjL 	 
    	  
    		   
     
    _11093822294365 mfqY1vAKV8IyrQ6Hp8nMV 	 
    _2654435874 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		   mtmf10UJavtFufEIdYJQE 	 
    	 
         mAOUv187BUnBTJj3ytNVq 	 
    	  
    		   
     
     
 
 
     mikfDmb676vn6P2J8crTL 	 
  
    
     mYQfKuc6wcfjjileYeZRV 	 
    	  
    		   
    auto &_2654435878:_46082543180066935.markers mmMWx9cmhAJYDHc6CoPK8 	 
     mkMa5jPmx9O_418wkkYBV 	 
    	
        _2654435878.und_corners mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
    _2654435878.corners mtmf10UJavtFufEIdYJQE 	 

        _12800511165451773841 mV0JUzxDJGEJD_uX2w3nt 	 
    	  
    		   
   0 mUmS87lLj_Mf5SpVtoLqH 	 
    	  
    		   
    ._5505640830793117477.undistortPoints mCNsgngMBa8r2GTvALgCE 	 
    	  
    		   
     
  _2654435878.und_corners mrnqzGFHPDarlUVk4S3a3 	 
    	 mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     
     mr6i8va6vrjg52LdPBFvK 	 
    	  
    		   
     
    _46082543180066935.flags.resize mTnJ0tRkJ9I2h7pCRqgNu 	 
    	  
    		   
    _46082543180066935.und_kpts.size mOuSLzvQYyP5qwR44RZl0 	 
  mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
 mtmf10UJavtFufEIdYJQE 	 
   
     mZwbWPcaOgtoYVI3Vym0_ 	 
    	  
    		   
     
 auto &_2654435887:_46082543180066935.flags mSSFbqZ6suzyqBFuTOK8y 	 
    	  
    		   
     
   _2654435887.reset mOuSLzvQYyP5qwR44RZl0 	 
    	  
    		   
      mxzEpv4Eh_7CpNKRgx4W9 	 
    	  
    		   
    _46082543180066935.ids.resize mhjZhzZncRXafCfJ3w69z 	 
    	  
    _46082543180066935.und_kpts.size mji_CmPnVQxlriixIouJp 	 
    	  
    		   
     
      m_f1eqo38yEJGZc_NJV2M 	 
    	  
  mcOUtWan0a817s2f7PxBD 	 
    	  
    		   
     
     
 
  
    
     mCzeyQHwBRD3aSZNFADGR 	 
    	  
    		_706246332827243 mCPc29blbqZ5t89uMSE7C 	 
    	  
    		   
     
 std mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		   
     
     
 
  	numeric_limits mkD1iGSJt2oyfwNp4pWsZ 	 
    	  
    		   
     
     
 
  	 uint32_t mo1oH2VnnRGtookyNl14U 	 
    	  
    		   
     
     
 mx_sf_UfonWBXCdYKLjrt 	 
    	  
    		   
    max mOuSLzvQYyP5qwR44RZl0 	 
    	  
    		   
     
     
 
   mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		   
     
     

     mID3KGvR2YcYnpT7Ek1mJ 	 
    	  
    		   
     
 auto &_11093822405034:_46082543180066935.ids mrnqzGFHPDarlUVk4S3a3 	 
    	  
    		   
     
  _11093822405034 mleKEARqJ89xIJCH7fsA_ 	 
  _706246332827243 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
    
    
    _46082543180066935.idx mgd705K7LHsqyMTqQZd5V 	 
    std mNmJ6yxo1KXO341_e1pHs 	 
    	  
    		   
     
  numeric_limits muZhC9Yd2qzjouoywqDiO 	 
    	  
    		   
     
     uint32_t mfiSRAwugVLmNlTHqDQLn 	 
    	  
     mukxzF5Z4VwhAj2gq2GBE 	 
    	  
    		   
     
     
 
  max mwoYGv1lf3LjTo33qtGck 	 
    	  
    	 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   

    _46082543180066935.fseq_idx mEsCokXGzfSkbSNhRJF3n 	 
    	_9933887380370137445 mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
     
 
  
    _46082543180066935.imageParams mCPc29blbqZ5t89uMSE7C 	 
    	  
    		 _12800511165451773841 mOUmw6CStUwR2yTJByg62 	 
 0 mVePGhmhzcrNpULYlc9MA 	 
    	  
    		._5505640830793117477 mFzprcwGX5TYPR8TCSOsI 	 
    	  
    		   
 
    _46082543180066935.create_kdtree maJhscyEMQhI3OBH5VVm0 	  mqzaY24A6rPasAO75wW1L 	 
    
    
    _46082543180066935.minXY mB1LJDQLgEtQsHykyiRGq 	 
    	 cv mNmJ6yxo1KXO341_e1pHs 	 
    	 Point2f mKzDQjDDFy9nfjTcIGEjn 	 
    	  
    		   
     
     
 
  	0,0 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
 mtmf10UJavtFufEIdYJQE 	 
    	  
    	
    _46082543180066935.maxXY mgd705K7LHsqyMTqQZd5V 	 
    	  
    		   
     
   cv mukxzF5Z4VwhAj2gq2GBE 	 
    	  
Point2f mrWpmXK1Uy57IaLxMoHw4 	_46082543180066935.imageParams.CamSize.width,_46082543180066935.imageParams.CamSize.height m_f1eqo38yEJGZc_NJV2M 	 
     mQfS9tiOvGsXX9DD53UoY 	 
    	  
   
     mjoRivKB9IaaWetnXIp3t 	 
   _46082543180066935.imageParams.Distorsion.total msRSLa1rEosf7XXiVm4xR 	 
 mNH8XnJPeJT8g0S6ijYW3 	 
0 m_f1eqo38yEJGZc_NJV2M 	 
    	  
    		   
    mdkwtZ2sLtp1DPR2xAXUz 	 
  
        
        vector mNaDEYA4ktSo1tAee0UBy 	 
    	  
    		   
     
     
 
cv mreuaw_ExVZgNEVAfU1Ix 	 
    	  
    		   
     
    Point2f mrzLZmHoLhUzGPqSVSWYQ 	 
    	  
    _6806985041881495917 mEsCokXGzfSkbSNhRJF3n 	 
    	  
    		   
     
     
 msDA0oA68jPtXx6r60msz 	 
   _46082543180066935.minXY,_46082543180066935.maxXY mhypoZE0o7J5ZKuuPEU18 	 
     mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
     
  
        _46082543180066935.imageParams.undistortPoints mxd1ooOS_bL0_Uh1bOsx8 	 
    	_6806985041881495917 mij3FyLvnkUgNUS9rv9RG 	 
    	  
    		   
  mjA8LcpHiJRrjo1GrHbPY 	 
    	  
    		 
        _46082543180066935.minXY mleKEARqJ89xIJCH7fsA_ 	 
    	  
    		   
     
     
 _6806985041881495917 mOUmw6CStUwR2yTJByg62 	 
    	  
    		   
     
     
0 mmLapJobqTMcWmayld_1p 	 
   mtmf10UJavtFufEIdYJQE 	 
    	  
    		   
    
        _46082543180066935.maxXY m_2dgqWMgdoegqwSE6Fwl 	 
    	  
    		   
     
     
 
  	_6806985041881495917 mV0JUzxDJGEJD_uX2w3nt 	1 m_f10xp9iBg5cy1Elj1NV 	 
    	  
    		   
     
     
 mqzaY24A6rPasAO75wW1L 	 
    	  
    		   
     
 
     mTQguI68n8F10NlqLVj95 	 
    	  
    		   
     
   
 mr6i8va6vrjg52LdPBFvK 	 
    	  
    		   
     
     
 

 mhypoZE0o7J5ZKuuPEU18 	 
    	  
    	

#ifdef _7892363061217164298
#undef  mgd705K7LHsqyMTqQZd5V 
#undef  mXzAExIMJoAgjDjzu8D1b 
#undef  mGAzYe14_xZXM3aWmeC2_ 
#undef maW_9jRkFbV0Jg_BX_n9RqWYw8z30Oh
#undef  mhVs1NlNCBiUHE03vFfXd 
#undef  mYw505xnZ0NhcrmjtCwiu 
#undef  mILiOfExbTWUMb6E0qWi4 
#undef  mLp03EKCwVPqCbUNhSBQL 
#undef  mr6i8va6vrjg52LdPBFvK 
#undef  mrraePCBsgi4fdXgdYTXy 
#undef  mENURSUbK5YsL6gDFG0XV 
#undef  mHnx0MhdU3oTzuyIpVbjV 
#undef  mNYuSvvMY7cC1egOFMDe9 
#undef  mpU2hymgTA0othpUeyhm3 
#undef  mdjqSIFIGMPy6TB2p5rXa 
#undef mPQ2s9U9DVqoNDeDMKI_4lRoeu4cBpX
#undef  myRZXV6h8AlBu1Yz0nX3_ 
#undef  mI1jUgaQGpSxGrrSMlXZ9 
#undef mi6S0Dk6cOldlqvgpWurl_2UzEcIjw0
#undef  mXh7D7IQ30bfYGV5sGPaO 
#undef  mWid0hlsAUY1zHRmxgF31 
#undef  mQW1qj2eB_butTzRBCZ8q 
#undef  ma_Rd5JsTsGYdr6SVfx3F 
#undef myGrOcKKBd5l_RKbm4zmSkIPIFCK42S
#undef  mDnMnCZpCb7zrOg4iSz03 
#undef  mkWP4IBnPeZGSrAaqSsJL 
#undef  mIbE_YisNma0t9GoO67Ew 
#undef  mmxdHUWdGjLkjtQEWsVG9 
#undef  mKFfRfKNAdGHTMXcRWoHV 
#undef mEb6LwiMJhIDtpOIyPvlqTMIfoIsp4V
#undef  mXDnhW__VmwWwJIvnFsLN 
#undef  me_IA79PbLZ_OARrc_uPr 
#undef mEi6ONBuxDhgXg8pU4BsXIswPV3UoMt
#undef  ml1GqNiJhthkEfvXSnaJx 
#undef  mORFe0xZzjkJ3dJXtMmDi 
#undef  mAkV6SPh7Ic1Oj0eyVdd5 
#undef  mpgp127ninlZ4DNI4hAoo 
#undef mCSLSZni8xAELU4sY5oy6Wqv8LDL7gG
#undef  mt4oZdscHQLkgCboylyXW 
#undef  mOeA74WLHuw0KgD2jDJJf 
#undef  mjoRivKB9IaaWetnXIp3t 
#undef mYcHAJcazQi1vgNOos_uHnDBo1TFnau
#undef  mCmejEEcw6mbsuLwBjzN1 
#undef  msRSLa1rEosf7XXiVm4xR 
#undef  mV0JUzxDJGEJD_uX2w3nt 
#undef  mKfZ0o2zaHXCOqL0xHIoR 
#undef  mQfS9tiOvGsXX9DD53UoY 
#undef  mhQdIzwRajAQGMZcowBBZ 
#undef  meQeweLLs8FbyuQ95xVw3 
#undef  mAOUv187BUnBTJj3ytNVq 
#undef mBy3V9HaolgsR8_dIRmyYtaNSayXxuV
#undef  mi_5tOeSpqmrWNCq5u_Mc 
#undef  mZbWBXlS0QTgGZOPD1_1K 
#undef  miJo4F2j3dkD8omqvp1B8 
#undef  moVkrUcknm2Dur4mU4lg2 
#undef  mNmJ6yxo1KXO341_e1pHs 
#undef  mYwwnUnHpluL2mXrXBOkk 
#undef  mvzaTnzatCWBA4BilWhwD 
#undef  mZivMr7Y_Eq0cjOTHMvDo 
#undef  mJKL5zHEjFsF92wjWTqwm 
#undef  mTXGfaSGfbInmGn4DClnj 
#undef  mYaVihULebEzCYJ3QR3P4 
#undef  mSJgbU9S58H8rmM3lUWSy 
#undef  miip_wX0isukeJQaecixC 
#undef  mzxCH9MuKC3C3HS9i9BR_ 
#undef  mVhaW8ANfJd8P2Fkqq3Gf 
#undef  mQSqjUxpUR2zqVJEYbq5M 
#undef  mRTmu8HBf2yujtxWglcvW 
#undef  mMVTm3d0OBJabraTFRmX2 
#undef  moo5YYsEcPIkPaYzVhmBE 
#undef  mw9xQerE7sCZgNzC9eCs9 
#undef  mYJrqHsaOGvQlJWJiyAxD 
#undef  mVQ8i37hpNkIkaxXGHhrO 
#undef mynCbyVeitARXJAlwvVxP95YjdQJXvz
#undef  mI9Gb4vLcvVPjVAV5QsO6 
#undef  mTED_KGydiVM1RbH5g_XM 
#undef  mCzWS7NpH3xKXsR3Wc5lD 
#undef  mx1eIclnlkObQ3brbbK1m 
#undef mgnwYCVPCfRl9DbDqpe1CjlghTXkTCo
#undef  mHUrFSdfvZIhAcBO0ueDF 
#undef  modRHhcgbpXJO9eZ61IgM 
#undef  ms8yr8htzxRLyf0ZH9bpl 
#undef mI55gjYwQ86o1pCfFFaUIhBlctjjKp1
#undef  mfgi4q0nqp6yuroWJMV6p 
#undef  mXewpxihHVBVnZgRIIOmS 
#undef  mJsXLE8zWibbVSZkgr7sR 
#undef  mc3NT7HjlpMaaOUyh2pm_ 
#undef  mCXLyXNIpE5CVT6b8WA0b 
#undef m_C4TUxtzAmfaiJjZjNYNwKylMzfPZ5
#undef  mhjZhzZncRXafCfJ3w69z 
#undef  mfLJ4DZuYxLKPaFykYuXO 
#undef  mFRlz5c4y22ZxTwvNMTJm 
#undef  mgOIVVAQzQ5eI5iuXfaeP 
#undef  mTAPKBWLHU6DHok0dQIf1 
#undef  mikfDmb676vn6P2J8crTL 
#undef  mXcU3lNsBzDQpRygbVL5u 
#undef  mAju5khZZ6HcJ6Tqu5FWg 
#undef  makL5jqnU___VhQ5sewiV 
#undef mp6LqvBCOZKVkg1Yn_i3XZo_4OSNeNs
#undef mccWkSB6n44SUQqThNPpMc5NiiqdFV5
#undef  mcvP1cjCNq7Fzz2UmFmHg 
#undef  ml5Wx70S9zMYFt94cgzMR 
#undef  mq9SRhyfxCdaIMKqQ8BFs 
#undef  mX7MFtiWgnK3l1N59mEfg 
#undef  mfjRK0U1BIpjv9bVGRmJR 
#undef  mYblBKQGKHQnpj1SGKzoX 
#undef mIMC5ACzyXDNjGacLR0WmyO73mj75RX
#undef  moT8JdFNunYaVSf3p5opt 
#undef  mR9rABeG_U9Or5MzHMizJ 
#undef  mDbODQ_pk1aeu1UegcTMF 
#undef msxa7s1nqtUSmJvIfw7Gf6ZmiJ_4BZ4
#undef mNYnc7TYXKquITVxiSR4g8pgVUQLdrV
#undef mZzyZDgrhCBCT7jSpEXMiPtXTHwZVy6
#undef  mF88QLuLswVNp4gkyxKC1 
#undef  mkUO1UYNfOlkn_FNhJPQy 
#undef  mrWpmXK1Uy57IaLxMoHw4 
#undef  mhubijm6_xUy0rnroZaYW 
#undef  maJhscyEMQhI3OBH5VVm0 
#undef  m_8b38IHjAGZ8i526e9W0 
#undef  mJAKI0m004Q4BblDgaUGq 
#undef  mIFo77Yc6l3egb4ZDAQzx 
#undef mn1ATwd5MF3y7oVEMuR2nFQx7FhC7_3
#undef  mkimz3XID1NS4WRLqHBkD 
#undef  meCjWleY3Dl205itM_ZqV 
#undef  mum5iYeCnDDy8nbEyxtM5 
#undef mBzWGES6q2ZTveRLPsvRzYmI3DWwUSf
#undef  mSRDpeweDkx3HzrNYbSBJ 
#undef  mZH6reMDs0LTknFpwssir 
#undef  mFYZfETqwJXuYwfa9aeTj 
#undef  mv1v0hP8yXyyMYanjndPT 
#undef mwJHgh2nPrAubqclTHvY_j8D9ues1kV
#undef  mtmf10UJavtFufEIdYJQE 
#undef  mfNuMYy8kbdstBSuCqIG7 
#undef  mqeU87itToVNjCC99YJYx 
#undef  mwm5CS2rChH_2oro391Od 
#undef  mkO3WBr3MgfF_cb_A2cVB 
#undef  mFzprcwGX5TYPR8TCSOsI 
#undef  mwh0H0CH8hbXoUMeW0njl 
#undef mYRvgsSCMOzIUDbBHoaQfc__oIJeXAv
#undef  mBpnD3k3uyepls7AtoRbM 
#undef  mNGfm4lrdni9kzsRyZZW_ 
#undef  mVYNPgVz_4wyryxpQ_Cqc 
#undef  mkojsQXmbjMflhcS04kG_ 
#undef  mKwpdOP7LA2TzewlweD6_ 
#undef  mZauRjZulqrGATSW8brQM 
#undef  meFhvE1PIaCGu5Q7hczZr 
#undef  mDouI5WYikZi15f2APwEr 
#undef  mEhyBKfYsOyCTEQ6lc4nM 
#undef  moU7cswum1tEpCFhdz0nH 
#undef  mTKlFdU1Dx6noAvGeHeF2 
#undef  mASg1ouD1BdIe8Qogbe7Q 
#undef  mcTc2RK6E445uTnlg41tB 
#undef mX7DbFtGTJdciQesTTZ1MJomozB7aMC
#undef  mv842vstwttwpSTC53ckv 
#undef  mjqjUbOB41j95fsQjGe4M 
#undef maX46T6x11cBV6m8ypHleZsR6A5kMyC
#undef  mVIhWNnO4WpjYSi5wjrsU 
#undef  mApcU3QorHU6UKsSNFCMa 
#undef  mmBFyV0pRbQZSqu8LEMmC 
#undef  m_YxQUUHIq8VTevXXDHD_ 
#undef  mc3yVlZ04etoTxiGGV1i7 
#undef  mElZB4rj5AdFj2VznXgor 
#undef  mFlLiaiBJqFiDudfeE9Kh 
#undef  mFspWuDxyb1nJ0DTdynBI 
#undef mqiAY2aFpTrUFfsDXauSpQVgGfKcA0J
#undef  mFcva5z1yM6KI3I8OsNe3 
#undef  mI_KmnkodyswOabezai5H 
#undef  mHpDQ4y94Jd3ehaxDmYhl 
#undef  mOqsQj_fw6vptW6nmXApd 
#undef  md3FWp5NtL2QVKzyeqdJT 
#undef  mbyb6jxRDAnCSInDFtmYO 
#undef  m_jmoUAF0yDvJJKifkYyS 
#undef mFw7yPHYLZb96zAhdyIo2N32SrQSZ06
#undef  mnXvDx6OWDFktIilhhUfY 
#undef  mJEoxwTEoOigzLpkgSjAq 
#undef mPS1gZLKwNHuKn6BbcvUZYPv7ksBxMQ
#undef  mjyI2a10ukdPXJp8Ii1qL 
#undef  mKWQiGcI03yFaWIZgwzoA 
#undef  mkXyQVTOuF_cxlC9sEKnc 
#undef  mSA0WoegzYv9KyxhsFr77 
#undef  mxFD8bOY_ESsY4AGC8rWX 
#undef  mMuLdBzfcFQFMUvDhPDpv 
#undef  mqo95vqnRMrnj9mz9qSlg 
#undef mGbR5wA6da0LvHoDkTQa38z3p6sipFT
#undef  mGGEvKPIuZbqGUag8vVEx 
#undef  mSKLIkoiXh2O2ZJCB5tMN 
#undef  muBvTpt3XzKH8F1gy8kye 
#undef  mO_YwuXCjdhOG2Y2RHoEf 
#undef  mLwp2qB0N1WcDxl0paSjv 
#undef m_jqlvmAsWE89NrrRCyUYNTqmgiGhCZ
#undef  mff7s0AdyqnKbv6sRk_IQ 
#undef  mRbFMmhFvakp82qdIXPnM 
#undef  mWHhGudmXGrCcT4vdVrfQ 
#undef  mjBfnx5bD9skndAUaBfxW 
#undef  mx6J1P9EqT1QzNSE3H2NV 
#undef  mkLGqN1ItSGOZdLr0KP4m 
#undef  mLmakzxCa2ww4gj87i1PJ 
#undef  mUfFaLTFPWV6pp9Q_PY13 
#undef  m_Zu3MijpZY76TObRxPlb 
#undef  mE82ddK08RqkE2ioB87Cg 
#undef mf5IhQWKMPIusuooSNsjdaPbq7gZhXM
#undef  mYm7Xp6xbkbmGjmKCRQsw 
#undef  mE_H_97PS57UMCX8elp7T 
#undef  mgDtQlzQ2fooHKv65shkR 
#undef  mGU8klm5s_iOx9SFoXjGl 
#undef  mUyy1dPtr6ye8RqArO8i7 
#undef mW7Ndg_zhFC4VTgvo24Rrz378fhRc7f
#undef  mk2zzOMnLY9Nk7_WGN3l0 
#undef mUHHXGrQf3D3O1ljVVkYniLgmooz8EE
#undef  mzD8x50gAKMFXpn1Kk7ZP 
#undef  meC4JX1WY2BpVgtvgjZ1B 
#undef  mkD1iGSJt2oyfwNp4pWsZ 
#undef  mHfAi5AlzlEQoZlyEISh2 
#undef  mSjI2mgsWD_A1XpN46AAd 
#undef  mQ5GzfRYJtHhuOIzF92Dv 
#undef  mI8XBdmCbSo9vPRkqOrN3 
#undef mDGFk8oxa_uglRJNTKL7MBe4GDFSwsh
#undef  mIbgaGLt_veu03B1y6TVE 
#undef  muga6f3tA6APHJGLYYvGb 
#undef  mZa6h8IcDq4tg9PE0pSxx 
#undef  mML8dd81_AIwqW_UaGYEP 
#undef  myaj6myEVlqrMXPEioaqI 
#undef mCYdHAHwZRkvlrSnwi4MIBYIRtIVw4L
#undef  mqwIU0rvE0oWeKVgWJYln 
#undef mOPggcLq0K1drLys3NNldvD1cfshoi0
#undef mWtMOi5dJUzI1sUSDiSmmMO0SzZ4QOe
#undef  miD2zs6QeugvIbR40Pmdg 
#undef mJwbq3_DM7OLOrqEDx3M4bJVqRD2mmi
#undef  mcKjT6p8Z3rrj4sDDXe7U 
#undef mBoekdygw1lUGjGJRkOF1NMbgoM55aM
#undef  mjA8LcpHiJRrjo1GrHbPY 
#undef  mNhtjQeMr73vCGfu1s8md 
#undef mrFVyPVqjRt0u5iz3jBisgMsaIUn_7h
#undef  mx8JhB_9A0B0jSDMRVJBM 
#undef  maeFWI39DY1ojcPc3Ae2E 
#undef  mErWLHXQ2oP3Ykz5Pkqx2 
#undef  mpENQJnA7iB3YHq2v_3bF 
#undef  mzqu2YiPQ5SkEBiZtUMyd 
#undef  mBu0w6fXbwNaO59skWzER 
#undef  mHuLzC0JrWfrRwHnE0M67 
#undef  mWVByyFDpS3mkTWcDjfIY 
#undef mt2JfTpShBapVOKoPasfOQgKo4MFEBO
#undef mMpozgBCbsUzZ43Zc6x9755g_y2lxqG
#undef  mzfVZbevdXZt1lQmnk8So 
#undef  mfpmGpvIPPUbTfCWWDD_m 
#undef mSWHH9BVZWheJzKMNLGJrBLT3nZAjk8
#undef mTBlvmqXr4jGON3l549ndle24HHPwpF
#undef  mSnypOnN3BGb0Sf99pqCF 
#undef  mI6bj8J7uRxv8ATP_edoX 
#undef meUjfcS_IVwLKwr9EPPTtItbqiCasAc
#undef  ms7nFQYTD6lTbbh4ccN6q 
#undef mV_jiVURl1LzTxy5RVjVceRA4Q8et0x
#undef  mij3FyLvnkUgNUS9rv9RG 
#undef  mHeJOL9dAIju_MrKHZ2Lu 
#undef  mooVnUfuO20MEn8yVsp5k 
#undef  myasISNHobbz0Mqqp1Is0 
#undef mUNWZK3pUsXO7rBGNXsMmo1wkUjiEHs
#undef  mRkdh3n8Tu8GfnzIjmaU6 
#undef mugz73lirPchWOPFMyfaLfxZh6ufriY
#undef  motqoD9yEu959l6_lYdsf 
#undef  mSOn7mlmPZKMhqvAT_mFa 
#undef mY3VxQRFAShydoaG6QflNJNM3deRb9_
#undef mAiVWFBE1kqFNTbht9YtwCtczAnUzvN
#undef mC1lMyfNQlgcuZbgBoZtBncuCtQIUqO
#undef  mggO2nUknv36WZKdfBBvE 
#undef mV8nPDDoxmTr8LHSyXj6qtsRyHgxaCs
#undef  mCdGviKX2tW09rD2iUjwV 
#undef  mOuSLzvQYyP5qwR44RZl0 
#undef  mrjtqAyOVGjxWdEwyGq7W 
#undef mur5T4VI0ecCwaRH8bESf8y8PCxQha1
#undef  mVlIk0ncU1k0KblI7kXJb 
#undef  mSJWWH2zQd9HgNGguChqj 
#undef mlptpDNtP3RNo01mcJnjc6y_jqu5XqN
#undef  miOzAHag9F4gT7ROHlfh_ 
#undef  mZXjD1gfBasiwgcnH9Kev 
#undef  mxQNFG7xz5qgcDdgJPCcx 
#undef  mpLgi59MRGuq_HOb9BA50 
#undef mKrfVYf3XKtqFPB3X_SvkZzipNl26GS
#undef  mzzUpgIrxS5Tz5DDcg3Pl 
#undef  mio6mwotLIwPAa3aBx5XS 
#undef  mERJ1GNplDMg0yOy8L0ug 
#undef  mR9iReIdHFAyvcWV25z2H 
#undef  mmLapJobqTMcWmayld_1p 
#undef  mjhVlWHK5qr678DTwN14o 
#undef  msqlmpDaitk8qmIoMstNu 
#undef  mB9k03xsLByQZPal5hSGk 
#undef  mr5CEz_VrMrPXc2hRtDi3 
#undef  mgYKecCGYvmUKxWyLU5Yp 
#undef  mTZwdzczLVbxzpWnOdfGa 
#undef  mUizUxqcwWNjUnwqaUzyb 
#undef  mdVjQpRPzNbHBaXyUnX2J 
#undef  mfiSRAwugVLmNlTHqDQLn 
#undef mfRtMj6hC0fS09Sckgb8Kxt53boniSi
#undef  mVMubSaER93sod6AcNaVY 
#undef  mZwbWPcaOgtoYVI3Vym0_ 
#undef mOtYDqy6_xvUSt9m1G0WjwgtfImYmLM
#undef  mg9RcyoUpBMcPhDg3RmnG 
#undef  mM1Wr0HMg3CNI1MP7nzg_ 
#undef  mFeSCRmKjA7JKdFLARQ6_ 
#undef mKmVBZvJe9FD6MR7wtSzEwrORKj7htZ
#undef  mj09THBEPXwun9vVG_ygd 
#undef  mvIsUEd5EevSMn8AkFEr2 
#undef  mccoC28wai6IWgunSqG4I 
#undef  mlzZOCnvpuNQTxiXdZwpC 
#undef  mQqHkx_GRkHAGh_8sGVB9 
#undef  mrSc6lwF_HlwXZHjcxq5S 
#undef  muWP0dquHWmMLiaB1LirH 
#undef  mC4xav6jq4r9qdxV8aAjL 
#undef  mmMWx9cmhAJYDHc6CoPK8 
#undef  mBngNnv8mHetcnWrAQGCN 
#undef  mKzDQjDDFy9nfjTcIGEjn 
#undef mO_RBOm1f9DByFtOZ5c2MR5nEZLX3sO
#undef  mhXo791poINq2hJ_2l6NK 
#undef  mFkAsEqesj8AsMNiEOB3E 
#undef  mfLTBWN12E1Fpsr4SddaU 
#undef  mZN1Zo1zK333HnGG3n9Kg 
#undef  mAZ9rTzpHdADaRd2Bb691 
#undef  mvh7NIcjiDW_pyQpLib4Z 
#undef mYERBAOurDuP6YswdxGcdyE0imf_Ieq
#undef  mfzoot_XsNZNrbqgA8tP5 
#undef mqhkTwm5rvsahp0ywp_07K302PXE4JU
#undef  mj1FYqv8U0ZeHnxgDf42G 
#undef  mIbK2m28YepmBKoCm61cv 
#undef  mrnqzGFHPDarlUVk4S3a3 
#undef  mICsDsAd4nYXICgXgbl0o 
#undef  mT5G1Cwl_1DwbIe1jQm7p 
#undef  me2apC9EkxS0a1MLNpxCO 
#undef moaT30J8Uonwi5NUCrHozf2mgxEXqP6
#undef  mrNEGap3YpE0gqVau1IbE 
#undef  mf12hfREnmnF8LvpyEYPf 
#undef mzDB_uNI8qcc3XRfgTgzYnQgmCox3Xs
#undef  mYo0CBFKDf0xO1zGnJmri 
#undef  mf7uQnLk3PYBBCuntis6S 
#undef  mbdsMa7jH60UElJsX0KDJ 
#undef  mj_L7II2AK4YJr3re7pNc 
#undef  mGTsC5WBbh9pRfQwp_ngK 
#undef  mr6IOB3zGpLy55k7h8RW2 
#undef  myIuhEZVrrx7ujdA0U2P0 
#undef mJlRsBHgvq8m1GTzuNvDUayTBwsRCxx
#undef  mvSflanzpnxaAKW5vBjsh 
#undef  mukxzF5Z4VwhAj2gq2GBE 
#undef  mzAPaDrr26WBLJaPLAhAB 
#undef  mdgFJ1P04BAlyREu4lwcG 
#undef  mi2IhvsgG2eoPm4A5mDcU 
#undef  mwTGMSqlSHVE3ODL9j2lB 
#undef  mDe9OcuvFkYD49F9EbnDT 
#undef  m_wCH5IAQfazKbdBPiKLv 
#undef  mx3yy2lZVVSJ9u9EbkIfD 
#undef  mdkwtZ2sLtp1DPR2xAXUz 
#undef mHh6VteM99DWUdt2pX6MSuonEaJypdb
#undef  m_f1eqo38yEJGZc_NJV2M 
#undef  mBGo9jkhIFfV0fH5zfz5t 
#undef  mZudLh3LuvzwNtIg9heHw 
#undef  mVePGhmhzcrNpULYlc9MA 
#undef  mrzLZmHoLhUzGPqSVSWYQ 
#undef  mqO12iB0NH1UmNi6cKZzy 
#undef  mqxiqEVJqEpdcx2wQqpBd 
#undef  mh7sA0nxoEP7JvljkaMNk 
#undef  mcAkplAtU_jHRvwupIxCl 
#undef  mkMa5jPmx9O_418wkkYBV 
#undef  mGDXomYjoJafW8z9K6m1P 
#undef  mG3STzxtp1FcibwrKUS35 
#undef  myxALO6iQAUoZa8_hEbsz 
#undef  mSRiiEeTCByC6NcUUUK2v 
#undef  ml2d8CUNXHy8FuGN2mxhB 
#undef  mdYur1q_by6ODuryMT9iU 
#undef  mb5TIbHhZNAo9gYpJ9HQm 
#undef  mhraMGrKiOjd9qXHjUnVP 
#undef  mCZX10THppqeTgSrqysBt 
#undef  mab7VCXQdI1JL3ySN7TyO 
#undef  mBbBvAQ3ZPzFIeA1627_k 
#undef  mSSFbqZ6suzyqBFuTOK8y 
#undef  myudRlGPdeEDMlo9qPIzk 
#undef  mJSdvdtWO4_pkCgiN7xj5 
#undef  mc3Mj6HvensKjoIAWkC9N 
#undef  mx_sf_UfonWBXCdYKLjrt 
#undef  mmEOOzMMIXMPpVimMMWFN 
#undef  mUmS87lLj_Mf5SpVtoLqH 
#undef  mMdt4NW72l_xAW2a_7MaH 
#undef  mf93KHE75PL0YvPjf3hDs 
#undef  mrDrclEdY4Vi4Qr6Qveis 
#undef  mD6fu9M7Sgmg4hL5uxTFf 
#undef  mUakI50g7mfxzM8UuqV2_ 
#undef  mNxD4dUFQydr8zzcqQfyQ 
#undef  mo1oH2VnnRGtookyNl14U 
#undef mXFQPMSVxI8GX9qW_j_JHVuI3IQAxgI
#undef  mboqNWaztwhcPEj_PpA3Q 
#undef  mBNhDkZd8sWapx0LxE02f 
#undef  mmq9HNT6SMSK3FI_mJgeB 
#undef  mCzeyQHwBRD3aSZNFADGR 
#undef  moob3Tjk5_eucM91XQB9l 
#undef  mY4CnwjxIThh4pzZvvLJZ 
#undef  mTQguI68n8F10NlqLVj95 
#undef  mu4uPqbqXkhC5h4uvMqPK 
#undef  mNGNAekcyVB3BBHwb2S0y 
#undef  mlQmkoT3Es1uOLX_lRSyd 
#undef  mUa_inzJAyguyAfuTj3RF 
#undef  myIh9SsGuPxW0iXGx0IoV 
#undef  mmq78_qMEiQRxdANIEaE2 
#undef  mgerPKsaoNAi9ZakHXnUj 
#undef  mEsCokXGzfSkbSNhRJF3n 
#undef mtvNQYx7BKsaXne8wADldML3ttrYnma
#undef mh7Okqgkf8DBlLcg87QkJsUpTRjq7Ha
#undef mKXzDkYpG_8FJihbeOoed9KoQFsZuhM
#undef  mxa9WM3AYB8EYc97iYWl0 
#undef  mNaDEYA4ktSo1tAee0UBy 
#undef  mq3NMzP75aSEvua42dKaz 
#undef  mLjPrJR_wSFaBK8AA8TWf 
#undef  mVOyFcmnL0XXFMsyUotgB 
#undef  mDdotmnOqAPyG1TX2QMMA 
#undef  mg5mxVukUYjE8eGmYVFZ2 
#undef  mJVqrARa5craS_rpTwFoN 
#undef  m_2dgqWMgdoegqwSE6Fwl 
#undef  miJUo5jnQovuz3yGyssDa 
#undef  mxVL059Rj7sAVxu8ZQrac 
#undef  mJUrPJCvnG0u5vWtQbhFe 
#undef  mISNiaRe6Ng6lkxBzcP0M 
#undef  mMH6jfL3NjDSEwhJQ7ht1 
#undef  meDeg2depRbhujkehIjYn 
#undef  mPrjYDQhnqU23TeGfL8Fd 
#undef  mreuaw_ExVZgNEVAfU1Ix 
#undef mEn0mJ2sMejgCFxEzf4SsBy9P13fJPd
#undef  mo5_YCe_3p3oahUqMp1lZ 
#undef mEhqSQ41U24TCPBJk_VktDfVhEADbuz
#undef  mP7TCqDhX9vf4SP2pVoPh 
#undef mEB3sTrQHkkDiOa6y6whys_TFlMP5vG
#undef mpUz75ow3QnH2t1ubVURknUiFUzmwtz
#undef  mmz0I7NAU_CfdcrmO0sD3 
#undef msCqMiO9_C_EWpyXGBNK2fglqKf2eDT
#undef  mQFoEm1zkTdY7rZvWRq4M 
#undef  mdneNFvgDhDtbPudgXuJt 
#undef mSCj1Srg5ZHmzgHtORPBajnlXfskCKj
#undef  mWzFn_mBBwvC8tgkog5X5 
#undef mneRPtvmoTTJ4VdPrrfc8N7rL7d2bUT
#undef  mSxJs4gE7p50F9jNGMjN9 
#undef  mtzxS2_2ntP3K4OCfU6H7 
#undef mEmeIn0QDX_JSdoYRVx2UcyM0gQCdUd
#undef  mZhNGjTuryvAzEnsGpJPT 
#undef  mv3svsTZs7sWccnQ9ExiA 
#undef  mwggywncoycNg0ZalVKcE 
#undef  mgOPlNL2g9SqVQgUhuti7 
#undef  mPobsR0SUkL4k40J6meFr 
#undef  mKZSfq5CAVRIjVM6z9udz 
#undef  mWDkBJgv8ZNU9tBu7DC8H 
#undef mFhbh6WxadnFAtnPxvCHCXUYuwiM0dz
#undef  mMjTIYp_7sZv7lARatjuB 
#undef mxmpYhT4_mhzhQB7OCiLHNAjO3Qsxgd
#undef  mVeA7EFwXxAtT0b0bO_Tl 
#undef  mO3cy6qBtctqMh9v1n6AM 
#undef  mwDPBjt_JAxtL9j0stKBK 
#undef  mTtrxwRZKYNcw1C_FQRXS 
#undef  mp3ALuatVgPMBt3pB7xx9 
#undef  mM_JyoqKJWOyd7xyV_Pob 
#undef  mKtAv9uNvOnILS3tul6Jg 
#undef  mDtPKfWKFQ_J_7DQBz07x 
#undef  mI_gFoTifEc3UvYetwZE2 
#undef  mRYaFSeiBVdNfVQpbg_gH 
#undef  mVKRBrgbb_3KvAoVy7Y76 
#undef  mID3KGvR2YcYnpT7Ek1mJ 
#undef  mhu8Gj47ZnxkaJ8t6f6uP 
#undef  mji_CmPnVQxlriixIouJp 
#undef  mUoB4yoSYI1E3ym2ZBf33 
#undef  mSnxauW1l4cH7MlfpdxUs 
#undef mHC647XXBFhXBpeDYefHxZQoV2ZylKp
#undef  mkE7Ykm6gZJFSVGppStlu 
#undef  mask7yuvCXZPiuY5e504A 
#undef  mdzT6psG_86ZMDW_2KJtn 
#undef  mo9quB8eIg1npyuTaUzlx 
#undef  mIMdzMIkd7_29ucta_LAy 
#undef  mpqlIZFmNBWfOnIJL92lD 
#undef  mpsvSLcMEIUE3R_u2Tltx 
#undef  mtvkUlfY1BF4KdNJaDnYe 
#undef  mTnJ0tRkJ9I2h7pCRqgNu 
#undef  mLnED0Dm5GLvaIjhfJtxV 
#undef  msagS6fnc4mr0wm8CBA6I 
#undef mVdtJBPg6QfC2IKKktd2z1Un6lwNwjF
#undef meaMoGxabLAzPaXAgGYpeoKJpBHV28L
#undef  mXiLtzRcWjDEAek2NbwiV 
#undef mh0XkrbSEwNvgR0tf39a4IrF9kkH3aY
#undef  mqA5_gscttdBLbEY9ELaK 
#undef  mPlnPE2m8HZNTQZ8ON28r 
#undef mP9ny6SUph9JsFZtWtJDe4XapV79vVD
#undef  mx766FnERFEEgCdVnQy7V 
#undef mjPQ2Jgne3Bp0t69GkXfo6zhqfJ3lz6
#undef  mzePdU27OKbFlZZ42yIRY 
#undef  mPgD9jtFbDFY_jW4aQF9v 
#undef  mjMWFdoUKZIiXkNZ57jQi 
#undef  mCPc29blbqZ5t89uMSE7C 
#undef motJRKDqaX6WZYh9kKCsod_zqEEyiAQ
#undef mldrlDDSPs94RCKeDfbTnWmrJ1sq8vG
#undef  mnGIUEiJQkiACsREjxo5v 
#undef  mTCT82oy2YWyKoo1flGrH 
#undef  mS5lsgt2rx3WpTtLlLBW3 
#undef  mt8XKCQLE1Z66tL_YR8aC 
#undef  mXpWjd6Da2oJAVgzWlzrH 
#undef  m_z4ro8joUKfW4Z5eRrC2 
#undef  mBR84S7CubRM60VKwus13 
#undef  mZ2_2tvDK9KkrBAARTPot 
#undef  mYQfKuc6wcfjjileYeZRV 
#undef  m_f10xp9iBg5cy1Elj1NV 
#undef  mvyz9F7m7Z76dYn99FG0x 
#undef  mxIO7h6PJBkfricKCIyow 
#undef  mzFNF5gRbWN6Bcr_tZEAn 
#undef  mfbeaAXVUs5OhGlqWOVi2 
#undef mO5qzO0BBrdHwVqDBWA3ggGX_GMQ09_
#undef  mq5iB_SvYcQOHaxqVDKSC 
#undef mq3PxsQfbD6BHdJB2SoEFRU_m19Rnbs
#undef  muyP0iGDcz803udJeLeVN 
#undef  mCNsgngMBa8r2GTvALgCE 
#undef  mwbLOnMiTBzVKRgIlFnYH 
#undef  mrxbHnccAfM8suFrhn2qm 
#undef  muZhC9Yd2qzjouoywqDiO 
#undef  mdfitEjJDfX5JRuF8bXel 
#undef  mXvM0qorf_SpbgDdpYfPu 
#undef  mvXBiLTqg7lhkm1Nh8bPn 
#undef  mUahIcsBNmcUVblDAWprS 
#undef  mdgYtvUqosu6IEbgGCpUO 
#undef  mG4bG2yiAeCqG0TyyiMha 
#undef  mZpLMilyO0bRxIH38sCpH 
#undef  mpKCY0sGapkQKY7apcx6h 
#undef  mZmuVLfzgHBU2m2CtqWr0 
#undef  mgFKnycKm41sL2YZQ5lpq 
#undef  mU1JHGYmHGuiMF3xXenzY 
#undef  mYynacvMOsJPdrHyZrqLt 
#undef  m_lj0iFkh41gBWc09MCsP 
#undef  ms0Z3f33Rmn8fuYvGgoa1 
#undef  mX7P4s1_g7S0Vw9Y54fYj 
#undef  mAdrqP9V1LvJsjA0Aj6ob 
#undef  mLaFxTKaaD_L3PvkVeN6Y 
#undef  mY_7FyNgPv7fIjYivahtw 
#undef  mcJ0CCSuGu9_XMo985IYC 
#undef mwfuzPp4XIekv0tplysWJFuHvsAMz64
#undef  mu_wkmRnV5ozD1X_97OUp 
#undef  msuASqXEqahJgXQfOZ6MX 
#undef  mm3GuW09JJo8axO1ElzOX 
#undef  migBcIS9gGVE_48sIDVsw 
#undef  mZdkluhnBTGGYxak9Y3oe 
#undef mOVovPRlifM1c0D9SX2Qcg0jmuiCFfo
#undef  msDA0oA68jPtXx6r60msz 
#undef  mgKt7VS6FM2VcrsRGyRYQ 
#undef  mncvuqxjMdoDbpwtPdNlP 
#undef  mNH8XnJPeJT8g0S6ijYW3 
#undef  mcOZGi_ONtONTDthxekeR 
#undef  mOb7vP5ncyxcZGeJJnrwa 
#undef  mBOh4kOXxptFzI9UxRAvD 
#undef  mWeo5kjFRZxMUAQe8mmxw 
#undef  mhypoZE0o7J5ZKuuPEU18 
#undef mewJwObQM86ftmqqrYpSEP7mX8pwKnH
#undef  muGaMnbn9wzwHB1Hf2QGl 
#undef  mLwr3ORKxhiYVBNWCtrz3 
#undef  mfbt5J7SV9iaRBOq55Y3r 
#undef  mUpgkoar0dTXSWHoohUqp 
#undef  mnZIMdxSe0bwhxrHI7GNX 
#undef  mLKt8Vq7dlkHx_FS_4ubV 
#undef  mJxDYSQ8F13ugn_H_QO0F 
#undef  mb8eJr_RoiYK4GdUCapbw 
#undef  mqZiPh2UU3unhdzRfwVIt 
#undef  mW0sbE7SzhqeHzZGq7zim 
#undef  mwoYGv1lf3LjTo33qtGck 
#undef  me3btRfyaqb5ClBYDlyTe 
#undef  mB1LJDQLgEtQsHykyiRGq 
#undef  mW1UcfAC8WTfmNpBZRYoD 
#undef  mfepBt_IkXqdRWgLD_6UJ 
#undef  mleKEARqJ89xIJCH7fsA_ 
#undef  mDklz_JaoOvTrPYxdjKI0 
#undef  mgy_nFaz09bjPPER5FtBs 
#undef  mumNyH1FYX3I32tTDmN7d 
#undef  mmn2L475jNx9pG5OG93Og 
#undef  mi9GC6IycKxFSz9g_CVXZ 
#undef mX2YS5qNCxx4WA13fiIIj5F3LubdbFX
#undef  mcOUtWan0a817s2f7PxBD 
#undef  mFq44HKU4qY91OiqfWwHV 
#undef mhxwmdMskHstODI9NNDFcbcZGps2Qon
#undef  mSAMq84M1ajU6bvqkjk_t 
#undef  mDDfvt3NS0apcBGCE8job 
#undef  mBpxihTLI7mWa33lKHm6o 
#undef mHvosmTNFCgL9JiIRMwlzfQ_MEVdVVY
#undef  mj89OvJV2XAv03Crzylny 
#undef  mY6EkjnEEkk2kf3P2CNbA 
#undef mYY2BrKuCryBMS5VURtbHpI38K7OJHE
#undef  mGwSL4y5H5STcCbNqgDkd 
#undef  mPeYD5uLZqkM7zRnOSXoy 
#undef  mrPVVe4FvkR2o3oMuRPIt 
#undef  mfqY1vAKV8IyrQ6Hp8nMV 
#undef  m_fcd96qN6cq3IbR5ZMKK 
#undef mVq_kBYVjiUJOUSdfV76hnBB8bpRNLB
#undef  mu5KEdiJoFgtXEFiJAn7w 
#undef  mvFQC57y8wipLpQCZzjww 
#undef  mlOlwCYyTQx2KagePc2is 
#undef  mxmPGlIq_UbZhEhb3qIjD 
#undef  mGeeGZBwM9SX5v5V4YLxW 
#undef  mC4Qaa69L5y7kPGElwY39 
#undef  mRSJv1dHjU8olKcUlzVQM 
#undef  mCdgKNbzbgzTX98EOSa5l 
#undef  m_pbLH3oc7kd5B7elyJic 
#undef  mmQCCPqcjTesys7AQtAvW 
#undef  mOUmw6CStUwR2yTJByg62 
#undef  mQBJhEhGAPkNmt9WeIGXo 
#undef  mRotAewZ3cQMzML0UW3zu 
#undef  mRoYOREwJZnktaRTasVAF 
#undef  mzvTYdb5gPiPthWAVra0r 
#undef  mH0vDS5erwxVyVoypRmvt 
#undef  meyMUZB6UHmY3FVD_Y62b 
#undef mkeaE_Pc59moZvlXsd1fqcb2IqPvlwl
#undef  mQ96GcbuYf8YhhgtcIf1L 
#undef  mQt65nuPk48VsbP_j3pGY 
#undef  mqQcJdeZf24ZMXk357zUD 
#undef  mAWVNR_NR1uiYYwpsz1Ic 
#undef  mK8lzEMdDh4u16CXN_nsu 
#undef  mWxGnqr3TtQ6Oq94MrkX6 
#undef  meYIfaPXmFnMDvFmDcmRf 
#undef  mh_Tf1Nm9McqJoH10VHzf 
#undef  manHv0zqp3l89SmbTLRdn 
#undef  mjvzxrxfDdEk7DKEcp_1A 
#undef  mqzaY24A6rPasAO75wW1L 
#undef  mxzEpv4Eh_7CpNKRgx4W9 
#undef  moaaUUdDOGUc7iNfPSazC 
#undef mFm_5bBrwRYXrp0hwdzcf_qOop0FJWe
#undef  mnAho8yy99tQf5U9rmT5b 
#undef  mXQsUGNgZoukfTO39vkwP 
#undef  mzOhosYdDPvjBVX2OQJqA 
#undef  mmvgClZ1llA3_d9E17fsk 
#undef mLJcRkJlSxwz7oxmibPC0NMF3QWrcYt
#undef  mxd1ooOS_bL0_Uh1bOsx8 
#undef  mnAsK1UQpq2hqXnZP69XL 
#undef  mEYXB1C9jxDk6pOX0KXQ0 
#undef  ml68IJIq89IrD6MxU__3X 
#undef  mQ9it27VPrvf0QS2h6sJ_ 
#undef  mHGsFagXmPlHW2uq4QRgr 
#undef  mgdTJRVTRu7KCcddhNYkR 
#endif
