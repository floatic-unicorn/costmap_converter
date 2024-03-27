#include "costmap_converter/KalmanEigen.h"
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;
// initialize Kalman filter
void KalmanEigen::init_kf(pair<double,double> stateMat) // mode 설정
{
	int stateNum = 4;//x y x' y'
	int measureNum = 2;
	predState = 0;
	operateTime=0.0;
	unmatchedHistory = 0;
	matchedHistory = 0;
	x = Eigen::MatrixXd::Identity(stateNum, 1); //state Matrix
	A = Eigen::MatrixXd::Identity(stateNum, stateNum); // Transition Matrix 
	w = Eigen::MatrixXd::Identity(stateNum, 1); // Process Noise 
	Q = Eigen::MatrixXd::Identity(stateNum, stateNum);// Covariance Matrix of Process Noise
	m = Eigen::MatrixXd::Identity(measureNum, 1); // Measurement Matrix
	m_pre = Eigen::MatrixXd::Identity(measureNum, 1);
	H = Eigen::MatrixXd::Identity(measureNum, stateNum); //Output Matrix
	v = Eigen::MatrixXd::Identity(measureNum, 1); // Measurement Noise
	R = Eigen::MatrixXd::Identity(measureNum, measureNum);
	P = Eigen::MatrixXd::Identity(stateNum, stateNum);
	K = Eigen::MatrixXd::Identity(stateNum, measureNum);
	xPred = Eigen::MatrixXd::Identity(stateNum, 1); 
	PPred = Eigen::MatrixXd::Identity(stateNum, stateNum);
	
	//measurement = Mat::zeros(measureNum, 1, CV_32F);
	x(0,0) = stateMat.first;
	x(1,0) = stateMat.second;
	A <<
	 	1, 0, 1, 0, 
	 	0, 1, 0, 1, 
	 	0, 0, 1, 0, 
	 	0, 0, 0, 1;
	 	

	 	 // Process Noise Covariance
		Q(0,0) = 1e-3;
		Q(1,1) = 1e-3;
		Q(2,2) = 1e-3;
		Q(3,3) = 1e-3;
		//Q = Q * 5e-2;
		
		// Measurement Noise Covariance
		R(0,0) = 1e-2;
		R(1,1) = 1e-2;
		//P = P * 1e+0; // Error Covariance
		P(0,0) = 1e-2;
		P(1,1) = 1e-2;
		P(2,2) = 1e-2;
		P(3,3) = 1e-2;
		//cout<<"Init"<<P <<endl;

	
}


// Predict the estimated bounding box.
pair<double,double> KalmanEigen::predict()
{
	A <<
	1, 0, 0.1, 0, 
	0, 1, 0, 0.1, 
	0, 0, 1, 0, 
	0, 0, 0, 1;
	//cout << x(0,0) <<", "<< x(1,0) <<", " << x(2,0) <<", " << x(3,0) << endl;
	//operateTime += 0.05;
	//Q(0,0) = 1e-2 * measureTime*measureTime*(1.0+operateTime);
	//Q(1,1) = 1e-2 * measureTime*measureTime*(1.0+operateTime);

	// if(predState == 0)
	// {
	// 	xPred = x;
	// 	PPred = P;
	// }
	//Q(0,2) = 1e-3 * dt*dt;
	//Q(1,3) = 1e-3 * dt*dt; 
	 // Error Covariance
	//cout<<"Pred"<<PPred <<endl;
	//A(0,2) = 0.05;
	//A(1,3) = 0.05;
	xPred = A*x; // statePre
	PPred = A*P*A.transpose() + Q;
	//PPred = A*P*A.transpose() + Q;
	//printf("%.8lf, %.8lf, %.8lf, %.8lf\n",xPred(0,0),xPred(1,0),xPred(2,0),xPred(3,0));
	pair<double, double> prediction = make_pair(xPred(0), xPred(1));
	//cout << "state: " << endl<< xPred << endl;
	//cout << "Error Covariance: " << endl<<PPred << endl;
	// if(predState == 0)
	// {
	// 	x= xPred;
	// 	P= PPred;
	// }
	x= xPred;
	P= PPred;
	predState = predState + 1;
	return prediction;
}


// Update the state vector with observed bounding box.
void KalmanEigen::update(pair<double,double> stateMat)
{
	
	
	m(0,0) = stateMat.first;
	m(1,0) = stateMat.second;
	if(filterMode == 0)
	{
		//Q(0,0) = 3*predState+20;//200;//3*predState;
		//Q(1,1) = 3*predState+20;//200;//3*predState;
		//Q(2,2) = (predState)+20;//(predState+20)/(1.0+abs(m(0,0) - xPred(0,0)));//50/(1.0+abs(m(0,0) - xPred(0,0)));
		//Q(3,3) = (predState)+20;//(predState+20)/(1.0+abs(m(1,0) - xPred(1,0)));//50/(1.0+abs(m(1,0) - xPred(1,0)));
		//Q(2,2) = (predState)/(1.0+abs(m(0,0) - xPred(0,0)));//50/(1.0+abs(m(0,0) - xPred(0,0)));
		//Q(3,3) = (predState)/(1.0+abs(m(1,0) - xPred(1,0)));//50/(1.0+abs(m(1,0) - xPred(1,0)));
		
		//Q(0,0) = 200;//3*predState+20;
		//Q(1,1) = 200;//3*predState+20;
		//Q(2,2) = 10;//predState;
		//Q(3,3) = 10;//predState;
		// cout << "processNoiseCov: " <<endl;
		//<< Q(0,0) << endl;
		// cout << Q(1,1) << endl;
		// cout << Q(2,2) << endl;
		// cout << Q(3,3) << endl;
		// cout << "m-xpred: " <<endl;
		// cout << abs(m(0,0) - xPred(0,0)) << endl;
		// cout << abs(m(1,0) - xPred(1,0)) << endl;
		
	}
	//Q(0,0) = //predState*abs(m(0,0) - xPred(0,0));
	//Q(1,1) = //predState*abs(m(1,0) - xPred(1,0));
	// Q(2,2) = predState*abs(m(0,0) - xPred(0,0));
	// Q(3,3) = predState*abs(m(1,0) - xPred(1,0));
	//Q(0,0) = 2*operateTime*operateTime*abs(m(0,0) - xPred(0,0));
	//Q(1,1) = 2*operateTime*operateTime*abs(m(1,0) - xPred(1,0));
	//Q(2,2) = operateTime*operateTime*abs(m(0,0) - xPred(0,0));
	//Q(3,3) = operateTime*operateTime*abs(m(1,0) - xPred(1,0));
	//if(sqrt((m(0.0)-m_pre(0,0))*(m(0.0)-m_pre(0,0))+(m(1.0)-m_pre(1,0))*(m(1.0)-m_pre(1,0)))>1.5)
	//{
		
	//PPred = A*P*A.transpose() + Q;
	
	K = PPred * H.transpose() * (H * PPred * H.transpose() + R).inverse();
	x = xPred + K * (m - H * xPred);
	P = PPred - K * H * PPred;
	//cout<< P(0,0)<<","<<P(1,1)<< ","<<P(2,2)<< ","<<P(3,3)<< ","<<K(0,0)<< "," << K(1,1)<< "," <<K(2,0)<< ","<<K(3,1)<< endl;
	// sprintf(str,"%f,%f,%f,%f,%f,%f,%f,%f",P(0,0),P(1,1),P(2,2),P(3,3),K(0,0),K(1,1),K(2,0),K(3,1));
	// filefunc.FileLog("/home/mkyun/log/GPSKalman.txt",str);
	// memset(str,0,360*sizeof(char));
	//P(0,0),P(1,1),P(2,2),P(3,3),K(0,0),K(1,1),K(2,0),K(3,1)); 
	//}
	//x(2,0) = 0;
	//x(3,0) = 0;
	//K = PPred * H.transpose() * (H * PPred * H.transpose() + R).inverse();
	//x = x + K * (m - H * x);
	//P = PPred - K * H * PPred;

	m_pre = m;
	 //cout << "state: " << x << endl;
	 //cout << "errorCovPre: " <<endl<< PPred << endl;
	 //cout << "errorCovPost: " <<endl<< P << endl;
	 //cout << "MeasurementNoise: " << R << endl;
	 //cout << "Measurement: " << m << endl;
	 //cout << "Kalman Gain: " <<endl<< K << endl;
	 //cout << "predState: " <<predState << endl;
	 //cout << "operateTime: " <<operateTime << endl;
	predState = 0;
	operateTime = 0.0;
	unmatchedHistory = 0;
	matchedHistory++;
	
}


// Return the current state vector
pair<double,double> KalmanEigen::getState(int stateMode)// 0:position 1:velocity
{

	pair<double, double> correction;
	if(stateMode == 0)
		correction = make_pair(x(0), x(1));
	else if(stateMode ==1)
		correction = make_pair(x(2), x(3));
	return correction;
}
