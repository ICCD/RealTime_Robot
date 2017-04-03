// RealTimeRobot.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <cstdlib>
#include <pcl/keypoints/harris_3D.h>
#include <model_point.h>
#include <scan_point.h>
#include <matching.h>

using namespace pcl;
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("6.pcd", *cloud);
	ScanPoint scanpoint =  ScanPoint(cloud);
	pcl::PointCloud<pcl::PointXYZ> keypoint = scanpoint.getKeypoint();





	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("6.pcd", *cloud);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(cloud);
	
	pcl::PointCloud<pcl::PointXYZ> * i;*/
	std::cout << "No Bug" << std::endl;
	int o = 0;
	while (true){
		o--;
	}
    return 0;
}

void Ransac(int *x_cord, int *y_cord, int length, double &c0, double &c1, double &c2, char *file_name_prefix2, int nums, int &min) {

}
/*关键点提取函数 输入指向点云的指针，输出关键点坐标集合*/
//pcl::PointCloud<pcl::PointXYZ>::Ptr getKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
//	viewer->addPointCloud(cloud, "all_cloud");
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
//	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
//	harris.setInputCloud(cloud);
//	harris.setNonMaxSupression(true);
//	harris.setRadius(0.04f);
//	harris.setThreshold(0.0012f);
//	cloud_out->height = 1;
//	cloud_out->width = 100;
//	cloud_out->resize(cloud_out->height*cloud->width);
//	cloud_out->clear();
//	harris.compute(*cloud_out);
//	int size = cloud_out->size();
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_harris(new pcl::PointCloud<pcl::PointXYZ>);
//	cloud_harris->height = 1;
//	cloud_harris->width = 100;
//	cloud_harris->resize(cloud_out->height*cloud->width);
//	cloud_harris->clear();
//
//	pcl::PointXYZ point;
//	for (int i = 0; i<size; i++)
//	{
//		point.x = cloud_out->at(i).x;
//		point.y = cloud_out->at(i).y;
//		point.z = cloud_out->at(i).z;
//		cloud_harris->push_back(point);
//	}
//	return cloud_harris;
//}
