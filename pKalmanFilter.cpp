#include "pKalmanFilter.h"
#include <iostream>

using namespace std;
using namespace cv;

//pKalmanFilter::pKalmanFilter()
//{
//}

//cv::Mat resultMatrix;//目标值向量
//cv::Mat *A_Matrix = NULL;//状态转移矩阵
//cv::Mat *B_Matrix = NULL;//控制矩阵
//cv::Mat Z_Matrix;//观测向量，存有观测得到的数据
//cv::Mat *H_Matrix = NULL;//控制输入矩阵，用于将观测数据转化为计算所需的格式
//cv::Mat *Q_Matrix = NULL;//过程噪音向量
//cv::Mat *R_Matrix;//测量噪音向量
//cv::Mat P_Matrix;//预测的协方差矩阵
//cv::Mat Kg_Matrix;//增益矩阵差矩阵
//cv::Mat U_Matrix;//控制矩阵

//初始化各个参数
pKalmanFilter::pKalmanFilter(int num_status, int num_control,int num_observe,float weight)
{
	this->num_status = num_status;
	this->num_control = num_control;
	this->num_observe = num_observe;
	this->weight = 1.0f / weight;
	result_Matrix = Mat(num_status, 1,CV_32FC1);
	randn(result_Matrix, Scalar::all(0), Scalar::all(0.1));
	P_Matrix = Mat::zeros(num_status, num_status, CV_32FC1);//协方差矩
	setIdentity(P_Matrix, Scalar::all(1e-5));

	//Z_Matrix = Mat::zeros(num_observe, 1, CV_32FC1);
	//Kg_Matrix=Mat::zeros(num_control, 0, CV_32FC1);//增益矩阵差矩阵

}

//对各个参数进行赋值，如果输入为空，则使用默认值进行赋值
//return 保留，用于扩展
bool pKalmanFilter::setBasicMatrix(cv::Mat& A, cv::Mat& B,cv::Mat& H, cv::Mat& Q, cv::Mat& R)
{
	if (A.empty())
	{
		cout << "Matrix A must be define";
		return false;
	}

	A_Matrix = &A;

	if (B.empty())
	{
		B_Matrix = new Mat(num_status, num_control, CV_32FC1, Scalar::all(0));
		//U_Matrix =Mat(num_control, 1, CV_32FC1, Scalar::all(0));
	}
	else
	{
		B_Matrix = &B;
		//U_Matrix = U;
	}

	if (H.empty())
	{
		H_Matrix = new Mat(num_observe,num_status , CV_32FC1, Scalar::all(1));
	}
	else
	{
		H_Matrix = &H;
	}
	//Q与R的比值对结果的影响较大，代表应该更信任哪个结果，
	//同时，如果已经知道实际情况（即不是真实的自然环境),初始值越大，最后得到的误差也越大
	if (Q.empty())
	{
		Q_Matrix = new Mat(num_status, num_status, CV_32FC1, Scalar::all(0));
		setIdentity(*Q_Matrix, Scalar::all(5.));//这个代表预测过程的可信度，数字越大，可信度越低
	}
	else
	{
		Q_Matrix = &Q;
	}

	if (R.empty())
	{
		R_Matrix = new Mat(num_observe, num_observe, CV_32FC1, Scalar::all(0));
		setIdentity(*R_Matrix, Scalar::all(5.));//这个代表测量过程的可信度，数字越大，可信度越低
	}
	else
	{
		R_Matrix = &R;
	}

}
bool pKalmanFilter::update(cv::Mat &input, cv::Mat &output,  Mat &control, cv::Mat& A, cv::Mat& B)
{
	if (A.empty())
	{
		cout << "Matrix A must be define";
		return false;
	}

	A_Matrix = &A;

	if (B.empty())
	{
		B_Matrix = new Mat(num_status, num_control, CV_32FC1, Scalar::all(0));
	}
	else
	{
		B_Matrix = &B;
	}

	//根据Q、R矩阵模拟过程与测量噪音
	Mat temp_processNoise(num_status, 1, CV_32FC1);
	for (int i = 0; i < num_status; i++)
	{
		randn(temp_processNoise, Scalar::all(0), Scalar::all((*Q_Matrix).at<float>(i, i)));
	}
	result_Matrix = input;
	if (!U_Matrix.empty())
		U_Matrix = control;
	else
		U_Matrix = Mat(num_control, 1, CV_32FC1, Scalar::all(0));

	//坐标速度的预测更新
	result_Matrix = (*A_Matrix)*input + (*B_Matrix)*U_Matrix/* + temp_processNoise*/;//1 模拟添加噪音

	output = result_Matrix;

	return true;
}
//对预测进行更新
//注意该函数会改变输入向量
//input 输入向量  output 输出向量  measurement 测量向量（测量值为未经处理的原始值）  control 控制向量
//return 保留，用于扩展
bool pKalmanFilter::update(cv::Mat &input, cv::Mat &output,cv::Mat &measurement,Mat &control)
{
	//根据Q、R矩阵模拟过程与测量噪音
	Mat temp_measurementNoise(num_observe, 1, CV_32FC1);
	for (int i = 0; i < num_observe; i++)
	{
		randn(temp_measurementNoise, Scalar::all(0), Scalar::all((*R_Matrix).at<float>(i, i)));
	}

	result_Matrix = input;

	//协方差的预测更新
	P_Matrix = (*A_Matrix)*P_Matrix*((*A_Matrix).t())+(*Q_Matrix);//2


	Kg_Matrix = P_Matrix*((*H_Matrix).t())*((((*H_Matrix)*P_Matrix*((*H_Matrix).t()) + *R_Matrix)).inv());//3

	Z_Matrix = (*H_Matrix)*measurement /*+temp_measurementNoise*/;//对测量值添加噪音（测量值为未经处理的原始值）

	//坐标速度的修正
	result_Matrix = result_Matrix + Kg_Matrix*(Z_Matrix - (*H_Matrix)*result_Matrix);//4

	//协方差的修正
	P_Matrix = weight*(P_Matrix - Kg_Matrix*(*H_Matrix)*P_Matrix);//5
	output=result_Matrix ;

	return true;
}

pKalmanFilter::~pKalmanFilter()
{
	delete A_Matrix;
	delete B_Matrix;
	delete H_Matrix;
	delete Q_Matrix;
	delete R_Matrix;
}
