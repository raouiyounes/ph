#include <iostream>
#include <vector.h>
#include <cv.h>
#include <fstream>
#include <highgui.h>
#include <iostream>
#include "Vision.h"
#include <string>
#include <eigen3/Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>


using namespace std;
using namespace Eigen;
typedef vector<string> Vec;
typedef vector<Vec> Mat;

#define ROWS 63
#define COLS 65
#define NUMBEROFFEATURES 1000

MatrixXd carte(ROWS,COLS); 

MatrixXd Vision::brisc_detector(int i){
const int BUFFSIZE=400;
map<int, vector<int> > carte;
char buff[BUFFSIZE];
char buff_appearance[3];
MatrixXd array(ROWS,2);
ifstream ifs("Brisc_db_detector.txt");
stringstream ss;
int aide;
string s;
int k;
int c;
for(int row=0;row<ROWS-1;row++) {
ifs.getline(buff,BUFFSIZE);
ss<<buff;
for(int col=0;col<2;col++){
getline( ss,s,',');
array(row,col)=atoi(s.c_str());
//cout<<array(row,col);
//cin>>c;
ss<<"";
}
}
return array;
}


vector<float> Vision::addToMap(vector<float> coordinate_feature, vector<float> &X ){
bool test;
int i;
for(i=2;i<X.size();i++){


if(coordinate_feature.at(0)==X.at(i))
    if(coordinate_feature.at(1)==X.at(i+1))
        if(coordinate_feature.at(2)==X.at(i+2))
test=false;
else
test=true;


if(test==true)

X.at(X.size())=coordinate_feature.at(0);
X.at(X.size()+1)=coordinate_feature.at(0);
X.at(X.size()+2)=coordinate_feature.at(0);

return X;

}

}


void Vision::matching(MatrixXd carte_frame1,MatrixXd carte_frame2, int i){

int N=6;
float x,y;
    VectorXd dist(N);
    VectorXd V1(N), V2(N);
int j,k;
for(j=0;j<N;j++){
    V1(j)=carte_frame1(i,j);
    V2(j)=carte_frame2(i,j);
}

for(k=0;k<N;k++){
x=V1(k);
y=V2(k);
dist(k)=hamdistance(x,y);
}

}

unsigned  Vision::hamdistance(unsigned x,unsigned y){
unsigned val;
unsigned dist=0;
val=x^y;

while(val){
++ dist;
val &=val-1;

}
return dist;
}
/*
void Vision::map_contruction(VectorXd X){

X(0)=10;
X(1)=10;
X(2)=10;

VectorXd h(3);
Eigen::Matrix3f covariance_matrix;
Eigen::Vector4f xyz_centroid;
vector<int>  carte;
MatrixXd  features(ROWS,2);
features=brisc_detector(0);


for(i=0;i<ROWS;i++){

if(belongtomap(features(i,0),feature(i,1))==false){

X(0)=features(i,0);
X(1)=features(i,1);
h=Prj2Dto3D(X);
addToMap(h);
}
}

}

*/

MatrixXd rotation_camera_robot(float thetak){
MatrixXd Rgr(3,3);
Rgr(0,0)=cos(thetak);
Rgr(1,0)=sin(thetak);
Rgr(0,1)=-sin(thetak);
Rgr(1,1)=cos(thetak);
return Rgr;
}

MatrixXd rotation_g_robot(float thetak){
MatrixXd Rgr(3,3);
Rgr(0,0)=cos(thetak);
Rgr(1,0)=sin(thetak);
Rgr(0,1)=-sin(thetak);
Rgr(1,1)=cos(thetak);
return Rgr;
}


MatrixXd Rcr(float theta){

MatrixXd R(3,3);
R(0,0)=cos(theta);
R(0,1)=sin(theta);
R(0,0)=0;
R(1,0)=-sin(theta);
R(0,0)=cos(theta);
R(0,0)=0;
R(0,0)=0;
R(0,0)=0;
R(0,0)=1;
return R;
}

Eigen::VectorXd Prj2Dto3D(VectorXd X){
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

int Vision::BriscCompute(IplImage* imageStr){
        char* str_image;
	char *str_detector;
        str_image="Brisc_db.txt";
	str_detector="Brisc_detect.txt";
        int i;
        CvMat *image1=0;
           ofstream outputFile(str_image);
	   ofstream   outputDetector(str_detector);
         static CvScalar red_color[] ={0,0,255};
            image1 = cvCreateMat(imageStr->height, imageStr->width, CV_8UC1);
            cvCvtColor(imageStr, image1, CV_BGR2GRAY);
           CvSURFParams params;
          std::vector<cv::KeyPoint> keypointsA;
            cv::Mat descriptorsA;
            int Threshl=40;
            int Octaves=2;
            float PatternScales=1.0f;
            cv::BRISK  BRISKD(Threshl,Octaves,PatternScales);//initialize algoritm
            BRISKD.create("Feature2D.BRISK");
            BRISKD.detect(image1, keypointsA);
            BRISKD.compute(image1, keypointsA,descriptorsA);
            cv::Ptr<cv::FeatureDetector> detector = cv::Algorithm::create<cv::FeatureDetector>("Feature2D.BRISK");
            detector->detect(image1, keypointsA);
            cv::Ptr<cv::DescriptorExtractor> descriptorExtractor =cv::Algorithm::create<cv::DescriptorExtractor>("Feature2D.BRISK");
            descriptorExtractor->compute(image1, keypointsA, descriptorsA);
            for(int i=0;i<keypointsA.size();i++){
          outputDetector<<keypointsA[i].pt.x<<" "<<keypointsA[i].pt.y<<endl;
           outputFile<<descriptorsA.row(i)<<endl;
        }
outputFile <<endl;
return 0;       
}



MatrixXd Vision::brisc(int i){
const int BUFFSIZE=400;
map<int, vector<int> > carte;
char buff[BUFFSIZE];
char buff_appearance[3];
MatrixXd array(ROWS,COLS);
ifstream ifs("Brisc_db.txt");
stringstream ss;
int aide;
string s;
int k;
int c;
for(int row=0;row<ROWS-1;row++) {
ifs.getline(buff,BUFFSIZE);
ss<<buff;
for(int col=0;col<COLS-2;col++){
getline( ss,s,',');
array(row,col)=atoi(s.c_str());
//cout<<array(row,col);
//cin>>c;
ss<<"";
}
}
return array;
}
MatrixXd Vision::Clustering(){
    const int BUFFSIZE=400;
    map<int, vector<int> > carte;
    char buff[BUFFSIZE];
    char buff_appearance[3];
    MatrixXd array(ROWS,COLS);
    ifstream ifs("Brisc_db.txt");
    stringstream ss;
    int aide;
    string s;
    int k;
    int c;
    for(int row=0;row<ROWS-1;++row) {
        ifs.getline(buff_appearance,BUFFSIZE);
        ss<<buff;
        for(int col=0;col<COLS-1;++col){
            getline( ss,s,',');
            array(row,col)=atoi(s.c_str());
            ss<<"";
        }
        
        cin>>k;
    }
    return array;
}


IplImage* Vision::map_simulated(){
CvCapture* capture=cvCaptureFromCAM(CV_CAP_ANY);
if(!capture){
fprintf(stderr,"Erreur de capture");
getchar();
return NULL;
}
}

void Vision::featureDetector(char* str){
 string temp;
Vec vect;
 numberOfFeatures=0;
 char separateur=' ';
    ifstream ifs(str);
while(getline(ifs,temp)){

string::size_type stTemp = temp.find(separateur);
vect.push_back(temp.substr(0, stTemp-1));
vect.push_back(temp.substr(0, stTemp-1));
numberOfFeatures++;
}
}


int* Vision::kmeanCluster(){
int c,i,j,k;
float distance;

int center[4][COLS];
int s[ROWS];
float s_biased[ROWS];
float d[4]={0.0,0.0,0.0,0.0};
i=0;
ofstream ff("test.txt");

ofstream ofs("testt.txt");

for(int i=0;i<ROWS-1;i++)
s[i]=0;
indice[0]=0;

for(i=0;i<COLS-1;i++){
    center[0][i]=carte(0,i);
    center[1][i]=carte(2,i);
    center[2][i]=carte(6,i);
    center[3][i]=carte(9,i);
}
//      bug
   for(i=0;i<COLS-1;i++)
        {

        d[0]=d[1]=d[2]=d[3]=0.0;
        for(j=0;j<ROWS-1;j++){
            d[0]=d[0]+sqrt(pow((center[0][j]-carte(j,i)),2));
            d[1]=d[1]+sqrt(pow((center[1][j]-carte(j,i)),2));
            d[2]=d[2]+sqrt(pow((center[2][j]-carte(j,i)),2));
            d[3]=d[3]+sqrt(pow((center[3][j]-carte(j,i)),2));

 
}

indice[i]=min(d);

}
return indice;
}

int Vision::min(float d[4]){
int i,min_i;
float min;
i=1;
min=d[0];
min_i=0;
while(i!=4)
{
if(min>d[i]){
    min=d[i];
    min_i=i;
  }
    i++;
}


return min_i;
}



int Vision::capture(){

    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
    if ( !capture ) {
        fprintf( stderr, "ERROR: capture is NULL \n" );
        getchar();
        return -1;
                 }
    // Create a window in which the captured images will be presented
    cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
    // Show the image captured from the camera in the window and repeat
    while ( 1 ) {
        // Get one frame
        IplImage* frame = cvQueryFrame( capture );
        if ( !frame ) {
            fprintf( stderr, "ERROR: frame is null...\n" );
            getchar();
            break;
        }
        cvShowImage( "mywindow", frame );
        // Do not release the frame!

        if ( (cvWaitKey(10) & 255) == 's' ) {
            CvSize size = cvGetSize(frame);
            IplImage* img= cvCreateImage(size, IPL_DEPTH_16S, 1);
            img = frame;
             cvSaveImage("matteo.jpg",&img);
                                            }
     if ( (cvWaitKey(10) & 255) == 27 ) break;
    }
    // Release the capture device housekeeping
    cvReleaseCapture( &capture );
    cvDestroyWindow( "mywindow" );
    return 0;
}


float *Vision::inverse_depth_parametrization(float *xyz, float theta,float phi,float rho){

float *Xi;

vector<float> m;
VectorXd y(6);
y(0)=*xyz;
y(1)=*(xyz+1);
y(2)=*(xyz+2);
y(3)=theta;
y(4)=phi;
y(5)=rho;
m.at(0)=(1/rho)*(cos(theta)*sin(theta));
m.at(1)=-(1/rho)*sin(phi);
m.at(2)=(1/rho)*(cos(phi)*cos(theta));

*Xi=y(0)+m.at(0);
*(Xi+1)=y(1)+m.at(1);
*(Xi+2)=y(2)+m.at(2);

return Xi;
}