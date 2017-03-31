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
	pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint;//点云   是否与ScanPoint *Scanpoint 等价？
	float Border[6];                           //边界
	int Number;                               //网格中点的个数
};


class KeyPoint {
public:
	OccupiedGrid Occupiedgrid;
	void getOccupiedGrid(ScanPoint *Spoint, pcl::PointCloud<pcl::PointXYZ>::Ptr point, OccupiedGrid &);//得到占据网格函数 参数列表：指向模型点云的指针，关键点坐标，占据网格引用。
	pcl::PointCloud<pcl::PointXYZ>::Ptr getKeypoint(ScanPoint *Spoint); //提取关键点函数 参数列表：指向模型点云的指针 返回值 ：关键点坐标数组
	vector<vector<vector<double>>> vector3D(Surface S);  //三维向量
														 //关键点面片大小，6D vector？
};



class ScanPoint                                                          //扫描点云类
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint;						     //点云指针
	vector<Surface>	surface;											//存放分割好的面的信息
	void getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint, vector<Surface> &surface);	   //三维向量表示面积大小
	bool byHeight();                                           //待定
	bool byArea();                                              //待定
	bool byOccupied();                                          //待定

};
/*关键点提取函数 输入指向点云的指针，输出关键点坐标集合*/
pcl::PointCloud<pcl::PointXYZ>::Ptr getKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->addPointCloud(Spoint, "all_cloud");
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
	cloud_harris->resize(cloud_out->height*Mpoint->width);
	cloud_harris->clear();

	pcl::PointXYZ point;
	for (int i = 0; i<size; i++)
	{
		point.x = cloud_out->at(i).x;
		point.y = cloud_out->at(i).y;
		point.z = cloud_out->at(i).z;
		cloud_harris->push_back(point);
	}
	return cloud_harris;
}


