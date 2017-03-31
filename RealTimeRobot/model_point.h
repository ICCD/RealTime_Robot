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
using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace pcl;

struct Surface {														//面
	double Area;			//面积
	pcl::ModelCoefficients Coefficients;			//面的参数
	bool IsVertical;		//垂直面值为1，水平面为0
};

struct OccupiedGrid {
	pcl::PointCloud<pcl::PointXYZ> modelpoint;//点云   是否与ModelPoint *Modelpoint 等价？
	float Border[6];                           //边界
	int Number;                               //网格中点的个数
};


class KeyPoint {
public:
	OccupiedGrid Occupiedgrid;
	void getOccupiedGrid(ModelPoint *Mpoint, pcl::PointCloud<pcl::PointXYZ>::Ptr point, OccupiedGrid &);//得到占据网格函数 参数列表：指向模型点云的指针，关键点坐标，占据网格引用。
	pcl::PointCloud<pcl::PointXYZ>::Ptr getKeypoint(ModelPoint *Mpoint); //提取关键点函数 参数列表：指向模型点云的指针 返回值 ：关键点坐标数组

	float*** TSDF(ModelPoint *Mpoint, float Border[6]);                  //TSDF距离场 参数列表：指向模型点云的指针，边界
	vector<vector<vector<double>>> vector3D(Surface S);  //三维向量
														 //关键点面片大小，6D vector？
};



class ModelPoint                                                          //模型点云类
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint;						     //点云指针
	vector<Surface>	surface;											//存放分割好的面的信息



	void getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint, vector<Surface> &surface);	   //三维向量表示面积大小




	bool byHeight();                                           //待定
	bool byArea();                                              //待定
	bool byOccupied();                                          //待定

};

void ModelPoint::getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr modelPoint, vector<Surface> &surface)								//分割面
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);	//原始点云
	pcl::copyPointCloud（*modelPoint, *cloud_filtered);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);		//每次分割出的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);		//分割后剩下的点云

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
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
		seg.segment(*inliers, *coefficients);
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
		surface.push_back(s_temp);


		// 创建滤波器对象
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
	}
}
