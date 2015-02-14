
#include <map.h>

#define ROWS 63
#define COLS 65




void VisualMap::extract_feature_point(){
Vision v;
int i,j;
MatrixXd M(ROWS,COLS);
M=v.brisc_detector(0);

for(i=0;i<ROWS;i++)
	for(j=0;j<COLS;j++){
		featureX=M(i,0);
		featureY=M(i,1);
	}
}


void VisualMap::Prj2Dto3D(VectorXd X){
        Eigen::MatrixXd RW(3,3);

Eigen::VectorXd hRL(3),h(3),fx(3);
int fku=195;
int fkv=195;
int u0=162;
int v0=125;
float u,v;
        RW(0,0)=0.1;
        RW(0,1)=0.1;
        RW(0,2)=0.1;
        RW(1,0)=0.1;
        RW(1,1)=0.1;
        RW(1,2)=0.1;
        RW(2,0)=0.1;
        RW(2,1)=0.1;
        RW(2,2)=0.1;
        hRL= RW*(fx-X);
        u=u0-fku*hRL(0)/hRL(2);
        v=v0-fkv*hRL(1)/hRL(2);
        h(0)=u;
        h(1)=v;
        return h;
    }






VisualMap::convert2landmarks(){





}


void VisualMap::addtoMap(VectorXd fx){


}

