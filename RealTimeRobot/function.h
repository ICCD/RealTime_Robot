#pragma once
//此文件存储公用函数，常数，宏定义
#include <key_point.h>
#include <pcl\registration\icp.h>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#define PI 3.1415926    //定义π
#define RANSAC_TIMES  100 //?

//关键点对应后生成的点对
struct PairPoint {
	KeyPoint point_i;
	KeyPoint point_j;
};

///<summary>
///匹配关系筛选函数
///</summary>
void Ransac(vector<PairPoint> pairpoint,int ransac_times) {
	int *count = new int[RANSAC_TIMES];

	for (int i = 0; i < RANSAC_TIMES; i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i; // 创建点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j;
		 //填充点云数据
		(*cloud_i).width = 5; //设置点云宽度
		(*cloud_i).height = 1; //设置点云高度
		(*cloud_i).is_dense = false; //非密集型
		(*cloud_i).points.resize((*cloud_i).width * (*cloud_i).height); //变形，无序

		(*cloud_j).width = 5; //设置点云宽度
		(*cloud_j).height = 1; //设置点云高度
		(*cloud_j).is_dense = false; //非密集型
		(*cloud_j).points.resize((*cloud_j).width * (*cloud_j).height); //变形，无序
		 //设置这些点的坐标
		int pointnum = 0;
		for each (PairPoint pair in pairpoint)
		{
			(*cloud_i).points[pointnum] = pair.point_i.Key_coordinate;
			(*cloud_j).points[pointnum] = pair.point_j.Key_coordinate;
		}
		//*********************************
		// ICP配准
		//*********************************
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
		icp.setInputCloud(cloud_i); //设置输入点云
		icp.setInputTarget(cloud_j); //设置目标点云（输入点云进行仿射变换，得到目标点云）
		pcl::PointCloud<pcl::PointXYZ> Final; //存储结果
											  //进行配准，结果存储在Final中
		icp.align(Final);
		//输出最终的变换矩阵（4x4）
		std::cout << icp.getFinalTransformation() << std::endl;
		icp.getFitnessScore();
		
	}
}

void  keyPointICP(pcl::PointCloud<pcl::PointXYZ>::Ptr SpointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud) {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //创建ICP对象，用于ICP配准
	icp.setInputCloud(mPointCloud); //设置输入点云
	icp.setInputTarget(SpointCloud); //设置目标点云（输入点云进行仿射变换，得到目标点云）
	pcl::PointCloud<pcl::PointXYZ> Final; //存储结果
										  //进行配准，结果存储在Final中
	icp.align(Final);
	//输出最终的变换矩阵（4x4）
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud(*mcloud,*transformed_cloud, icp.getFinalTransformation());

	//保存到PCD文件
	pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud); //将点云保存到PCD文件中

	//显示点云窗口
	system("pcl_viewer.exe 11351.pcd 70761_c.pcd");
	system("pcl_viewer.exe 11351.pcd transformed_cloud.pcd");
	//pcl::PointCloud<pcl::PointXYZ> cheng;
	//cheng.width = (*mcloud).width;
	//cheng.height = (*mcloud).height;
	//cheng.is_dense = false;
	//cheng.points.resize(cheng.width*cheng.height);
	//int tnum = 0;
	//for each (pcl::PointXYZ v in *mcloud)
	//{
	//	cheng.points[tnum].x = v.x + 1;
	//	cheng.points[tnum].y= v.y + 0.5;
	//	cheng.points[tnum++].z = v.z;
	//}
	//pcl::io::savePCDFileASCII("70761_c.pcd", cheng); //将点云保存到PCD文件中
	int o = 0;
	while (true) {
		o--;
	}
}

bool match_by_height(pcl::PointXYZ &key1, pcl::PointXYZ &key2)		//利用高度筛选
{
	float temp = key1.z / key2.z;
	if (temp >= float(2 / 3) || temp <= 1.5)
		return true;
	return false;
}

bool match_by_area(double *v1, double *v2)					//利用面积筛选
{
	if ((v1[0] / v2[0]) > 3 || (v1[0] / v2[0]) < float(1 / 3) || (v1[1] / v2[1]) > 3 || (v1[1] / v2[1]) < float(1 / 3) || (v1[2] / v2[2]) > 3 || (v1[2] / v2[2]) < float(1 / 3))
		return false;
	return true;
}

bool match_by_occupied(OccupiedGrid &o1, OccupiedGrid &o2)
{
	float temp = o1.Number / o2.Number;
	if (temp > 2 || temp < 0.5)
		return false;
	return true;
}

