

class RFID{

    VectorXd Xtag;
    VectorXd Ytag;
    VectorXd Xr;
  
public:
    RFID(){
        Xtag(0)=10;Xtag(1)=12;Xtag(2)=14;Xtag(3)=16;
        Ytag(0)=5;Ytag(1)=6;Ytag(2)=10;Ytag(3)=11;
    }
    ~RFID(){}
	int  RFID_cluster_data_with_pcl();
friend bool isInSector(float x_center,float y_center,float angle_offset,float angle, float x,float y);
vector<int> Rfid_map(VectorXd X);
void finger_print();
};