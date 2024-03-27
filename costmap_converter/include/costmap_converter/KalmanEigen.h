#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/msg/polygon.hpp>
using namespace std;
// This class represents the internel state of individual tracked objects observed as bounding box.

#define stateType pair<double,double> 

class KalmanEigen
{
public:
	KalmanEigen()
	{
		
		init_kf(pair<double,double> ());
		
	}
	KalmanEigen(pair<double,double> stateMat,int count, int obsid, geometry_msgs::msg::Polygon points)
	{
		init_kf(stateMat);
		id = count;
		obstacleId = obsid;
		pointList = points;
	}

	~KalmanEigen()
	{
		
	}
	pair<double,double>  predict();
	void update(pair<double,double> stateMat);
	int unmatchedHistory;
	int matchedHistory;
	pair<double,double>  getState(int stateMode);
	int id;
	int count;
	int obstacleId;
	geometry_msgs::msg::Polygon pointList;
private:
	void init_kf(pair<double,double> stateMat);

	Eigen::MatrixXd x;//state Matrix
	Eigen::MatrixXd A; // Transition Matrix 
	Eigen::MatrixXd w; // Process Noise 
	Eigen::MatrixXd Q; // Covariance Matrix of Process Noise
	Eigen::MatrixXd m; // Measurement Matrix
	Eigen::MatrixXd m_pre;
	Eigen::MatrixXd H; //Output Matrix
	Eigen::MatrixXd v; // Measurement Noise
	Eigen::MatrixXd R;
	Eigen::MatrixXd P;
	Eigen::MatrixXd K;

	Eigen::MatrixXd xPred;
	Eigen::MatrixXd PPred;
	int predState;
	
	float operateTime;
	int filterMode;
	
    //std::vector<stateType> predList;
};


