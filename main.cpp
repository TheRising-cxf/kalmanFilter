#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include "pKalmanFilter.h"
#include "time.h"
/*
cv::Mat resultMatrix;//Ŀ��ֵ����
cv::Mat *A_Matrix = NULL;//״̬ת�ƾ���
cv::Mat *B_Matrix = NULL;//���ƾ���
cv::Mat Z_Matrix;//�۲����������й۲�õ�������
cv::Mat *H_Matrix = NULL;//��������������ڽ��۲�����ת��Ϊ��������ĸ�ʽ
cv::Mat *Q_Matrix = NULL;//������������
cv::Mat *R_Matrix;//������������
cv::Mat P_Matrix;//Э�������
cv::Mat Kg_Matrix;//�����������
cv::Mat U_Matrix;//���ƾ���
*/

using namespace cv; // all the new API is put into "cv" namespace. Export its content
using namespace std;
vector<Point>orginPoints;//��ʵ��
vector<Point>kalmanPoints;//Ԥ���
int64 lastTime = 0;//��һ֡���ȡʱ��
float lastVx = 0;
float lastVy = 0;
Mat x_vector;//[X,Y,X_V,Y_V]
Mat z_vector;//
Mat u_vector;//
Mat A_Matrix;//
Mat B_Matrix;//
Mat H_Matrix;// 
cv::Mat Q_Matrix;//������������
cv::Mat R_Matrix;//������������
vector<float> err_x;
vector<float> err_y;
pKalmanFilter kalmanFiter(4, 1, 2,0.8);
int pointIndex = 0;
int downFlag = 0;
void on_mouse(int event, int x, int y, int flags, void *ustc)
{
	static cv::Point pre_pt = cv::Point(-1, -1);//��ʼ����
	static cv::Point cur_pt = cv::Point(-1, -1);//ʵʱ����
	char temp[16];
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		orginPoints.clear();
		kalmanPoints.clear();
		Point tmp(x, y);
		orginPoints.push_back(tmp);
		kalmanPoints.push_back(tmp);
		x_vector = Mat(4, 1, CV_32FC1, Scalar::all(1));
		z_vector = Mat(4, 1, CV_32FC1);
		u_vector = Mat(1, 1, CV_32FC1);
		A_Matrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		B_Matrix = (Mat_<float>(4, 1) << 0, 0.5, 0, -1);
		H_Matrix = (Mat_<float>(2, 4) << 1, 0, 0, 0, 0, 1, 0, 0);
		kalmanFiter.setBasicMatrix(A_Matrix, B_Matrix, H_Matrix, Q_Matrix, R_Matrix);
		pointIndex = 0;
		lastTime = clock();
		downFlag = 1;
		lastVx = 0;
		lastVy = 0;
	}
	else if(downFlag==1&&(event == cv::EVENT_MOUSEMOVE || event == EVENT_LBUTTONUP))
	{
		Point tmp(x, y);
		int64 curTime = clock();
		int64 dTime = curTime - lastTime;
		if (dTime <= 3)return;
		float vx = float(x - orginPoints[orginPoints.size() - 1].x) / (float)dTime;
		float vy = float(y - orginPoints[orginPoints.size() - 1].y) / (float) dTime;
		float ax = (vx - lastVx) / (float)dTime;
		float ay = (vy - lastVy) / (float)dTime;
		lastVx = vx;
		lastVy = vy;

		orginPoints.push_back(tmp);
		lastTime = curTime;
		z_vector.at<float>(0, 0) = x;
		z_vector.at<float>(1, 0) = y;
		z_vector.at<float>(2, 0) = vx;
		z_vector.at<float>(3, 0) = vy;
		u_vector.at<float>(0, 0) = 1;
		
		A_Matrix = (Mat_<float>(4, 4) << 1, 0, dTime, 0, 0, 1, 0, dTime, 0, 0, 1, 0, 0, 0, 0, 1);
		B_Matrix = (Mat_<float>(4, 1) << 0.5*dTime*dTime*ax, 0.5*dTime*dTime*ay, dTime*ax, dTime*ay);
		kalmanFiter.update(x_vector, x_vector, u_vector, A_Matrix, B_Matrix);
		Point temp_point = Point(x_vector.at<float>(0, 0), x_vector.at<float>(1, 0));
		kalmanPoints.push_back(temp_point);
		kalmanFiter.update(x_vector, x_vector, z_vector, u_vector);
		printf("%d %d %d %d\n", x, y, temp_point.x, temp_point.y);
		if (event == EVENT_LBUTTONUP) {
			downFlag = 0;
		}
	}
}
void a(Mat m)
{
	if (m.empty())
	{
		cout << "TRUE";
	}
	else
	{
		cout << "FALSE";
	}
}
int main()
{
	cv::namedWindow("img",0);
	Mat mat = Mat(1920, 1080, CV_8UC3, Scalar(0, 0, 0));
	cv::setMouseCallback("img", on_mouse, 0);
	while (true)
	{
		for (int i = pointIndex; i < orginPoints.size(); i++) {
			circle(mat, orginPoints[i], 3, Scalar(255, 0, 0), -1);
			circle(mat, kalmanPoints[i], 3, Scalar(0, 0, 255), -1);
		}
		pointIndex = orginPoints.size();
		imshow("img", mat);
		int key = waitKey(10);
		if (key == 32) {
			mat = Mat(1920, 1080, CV_8UC3, Scalar(0, 0, 0));
		}
	}
	waitKey(0);
}