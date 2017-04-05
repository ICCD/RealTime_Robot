#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/PCLHeader.h>
#include <pcl/exceptions.h>
#include <pcl/point_traits.h>
#include<vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <cstdlib>
#include <pcl/keypoints/harris_3D.h>
#include <pcl/octree/octree.h>
#include <key_point.h>

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace pcl;



class ScanPoint                                                          //模型点云类
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint;						     //点云指针
	vector<Surface>	surface;											//存放分割好的面的信息
	pcl::PointCloud<pcl::PointXYZ> key_coordinates;						//所有关键点坐标
	vector<KeyPoint> keyPoint;											//关键点集合
	void get_Area(pcl::PointCloud<pcl::PointXYZ>::Ptr scanPoint);	   //三维向量表示面积大小

	ScanPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint_got);
	void getKeypoint(); //提取关键点函数 参数列表：指向模型点云的指针 返回值 ：关键点坐标数组


};

/*关键点提取函数 输入指向点云的指针，输出关键点坐标集合*/
void ScanPoint::getKeypoint()
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	//viewer->addPointCloud(Spoint, "all_cloud");
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	harris.setInputCloud(Spoint);
	harris.setNonMaxSupression(true);
	harris.setRadius(0.04f);
	harris.setThreshold(0.0012f);
	cloud_out->height = 1;
	cloud_out->width = 100;
	cloud_out->resize(cloud_out->height*Spoint->width);
	cloud_out->clear();
	harris.compute(*cloud_out);
	int size = cloud_out->size();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_harris(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_harris->height = 1;
	cloud_harris->width = 100;
	cloud_harris->resize(cloud_out->height*Spoint->width);
	cloud_harris->clear();

	pcl::PointXYZ point;
	for (int i = 0; i<size; i++)
	{
		point.x = cloud_out->at(i).x;
		point.y = cloud_out->at(i).y;
		point.z = cloud_out->at(i).z;
		cloud_harris->push_back(point);
	}
	key_coordinates = *cloud_harris;
}


//
void ScanPoint::get_Area(pcl::PointCloud<pcl::PointXYZ>::Ptr scanPoint)								//分割面
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);	//原始点云
	pcl::copyPointCloud(*scanPoint, *cloud_filtered);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);		//每次分割出的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);		//分割后剩下的点云

	pcl::ModelCoefficients::Ptr Coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// 创建分割对象
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// 可选
	seg.setOptimizeCoefficients(true);
	// 必选
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.005);

	// 创建滤波器对象
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	int  nr_points = (int)cloud_filtered->points.size();
	// 当还有30%原始点云数据时
	while (cloud_filtered->points.size() > 0.15 * nr_points)
	{
		// 从余下的点云中分割最大平面组成部分
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *Coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		// 分离内层
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);		//分割出的平面存在cloud_p中
										//std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


		pcl::ConvexHull<pcl::PointXYZ> chull;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
		chull.setComputeAreaVolume(true);

		chull.setInputCloud(cloud_p);
		// chull.setAlpha (0.1);
		chull.reconstruct(*cloud_hull);
		Surface s_temp;
		s_temp.Area = chull.getTotalArea();		//得到面积
		s_temp.Coefficients = *Coefficients;
	//	if ()
			surface.push_back(s_temp);


		// 创建滤波器对象
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
	}
}
//define by xueyu 构造函数
ScanPoint::ScanPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint_got) {
	Spoint = Spoint_got;
}
