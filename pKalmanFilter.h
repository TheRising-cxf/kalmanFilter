#pragma once

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/utility.hpp>

using namespace std;

class pKalmanFilter
{
private:
	cv::Mat result_Matrix;//目标值向量
	cv::Mat *A_Matrix = NULL;//状态转移矩阵
	cv::Mat *B_Matrix = NULL;//控制矩阵
	cv::Mat Z_Matrix;//观测向量，存有观测得到的数据
	cv::Mat *H_Matrix = NULL;//控制输入矩阵，用于将观测数据转化为计算所需的格式
	cv::Mat *Q_Matrix = NULL;//过程噪音向量
	cv::Mat *R_Matrix = NULL;//测量噪音向量
	cv::Mat P_Matrix;//协方差矩阵
	cv::Mat Kg_Matrix;//增益矩阵差矩阵
	cv::Mat U_Matrix;//控制矩阵
	int num_status;
	int num_control;
	int num_observe;
	float weight;

public:
	pKalmanFilter(int num_status, int num_control, int num_observe,float weight);
	bool setBasicMatrix(cv::Mat& A, cv::Mat& B, cv::Mat& H, cv::Mat& Q, cv::Mat& R);
	bool update(cv::Mat &input, cv::Mat &output, cv::Mat &control, cv::Mat& A, cv::Mat& B);
	bool update(cv::Mat &input, cv::Mat &output, cv::Mat &measurement, cv::Mat &control);
	~pKalmanFilter();
};

