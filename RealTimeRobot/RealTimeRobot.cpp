// RealTimeRobot.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h> 

using namespace pcl;
int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr clou(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("6.pcd", *clou);
	pcl::visualization::CloudViewer viewer("Cloud Viewer");
	viewer.showCloud(clou);
	int o = 0;
	pcl::PointCloud<pcl::PointXYZ> * i;

	while (true){
		o--;
	}
    return 0;
}

void Ransac(int *x_cord, int *y_cord, int length, double &c0, double &c1, double &c2, char *file_name_prefix2, int nums, int &min) {

}