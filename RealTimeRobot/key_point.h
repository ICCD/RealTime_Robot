#pragma once
#include <vector>
#include <cstdlib>
#include <Eigen/Geometry>
#include <pcl/PCLHeader.h>
#include <Eigen/StdVector>
#include <pcl/exceptions.h>
#include <pcl/point_types.h>
#include <pcl/point_traits.h>
#include <pcl/octree/octree.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/keypoints/harris_3D.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>


using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace pcl;





double						//用于求空间中点到平面距离的函数
getDistance(float v1, float v2, float v3, float v4, pcl::PointXYZ point)
{
	double d = sqrt(v1*v1 + v2*v2 + v3*v3);
	return abs(v1*point.x + v2*point.y + v3*point.z + v4) / d;
}



struct Surface {														//分割面类
	double Area;			//面积
	pcl::ModelCoefficients Coefficients;			//面的参数		
	bool IsVertical;		//垂直面值为1，水平面为0		暂时不用于匹配
};

struct OccupiedGrid {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;//点云   是否与ModelPoint *Modelpoint 等价？
	float Border[6];                           //边界
	int Number;                               //有点的体素的个数		占据比由此筛选
};

class KeyPoint {														//关键点信息类
public:
	OccupiedGrid Occupiedgrid;			//占据网格
	pcl::PointXYZ Key_coordinate;		//占据网格中关键点坐标
	vector<double> vector3D;  //

	float grid_value[12][12][12];		//向量场
	float Border[6];		//tsdf的边界 ，并非占据网格边界

	KeyPoint(pcl::PointXYZ point);  //构造函数
	KeyPoint();						//默认构造函数
	void getOccupiedGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution, float f_adjust);//得到占据网格函数 参数列表：指向模型点云的指针，关键点坐标，占据网格引用。resolution 和f_adjust分别代表分辨率和占据网格边长一半

	void get_TSDF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution, float f_adjust);                  //TSDF距离场 参数列表：指向模型点云的指针，边界
	void get_Vector3D(vector<Surface> &surface);					//获取三维向量，即三个面的大小	


};

KeyPoint::KeyPoint(pcl::PointXYZ point) {
	Key_coordinate = point;
	Occupiedgrid = OccupiedGrid();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
	Occupiedgrid.cloud = pc;
}
KeyPoint::KeyPoint() {};
void KeyPoint::get_Vector3D(vector<Surface> &surface)					//获取三维向量			目前简单实现
{
	int vertical = 0, horizontal = 0;
	for (int i = 0; i < surface.size(); i++)
	{
		double distance = getDistance(surface[i].Coefficients.values[0], surface[i].Coefficients.values[1], surface[i].Coefficients.values[2], surface[i].Coefficients.values[3], Key_coordinate);//点到平面距离
		if (surface[i].IsVertical == 0 && horizontal == 0 && distance <= 0.02)		//水平面 只有一个
		{
			vector3D[0] = surface[i].Area;
			horizontal++;
		}
		else if (surface[i].IsVertical == 0 && horizontal <= 1 && distance <= 0.02)			//垂直面最多两个
		{

			vector3D[1 + vertical] = surface[i].Area;
			vertical++;
		}
	}
	if (vector3D[1] < vector3D[2])
	{
		double temp = vector3D[2];
		vector3D[2] = vector3D[1];
		vector3D[1] = temp;
	}
}
void KeyPoint::getOccupiedGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution = 0.02f, float f_adjust = 0.08f)//得到占据网格函数 参数列表：指向模型点云的指针，关键点坐标，占据网格引用。
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>octree(resolution);	//八叉树
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	Vector3f v_min;
	Vector3f v_max;
	Vector3f v_center;
	v_center << Key_coordinate.x, Key_coordinate.y, Key_coordinate.z;			//输入关键点的坐标，确定占据网格的中心

	Vector3f v_adjust;
	v_adjust << f_adjust, f_adjust, f_adjust;
	v_min << v_center - v_adjust;		//占据网格的左下角
	v_max << v_center + v_adjust;		//占据网格的右上角

	std::vector<int> k_indices;			//占据网格中点在点云中的索引
	int num;	//占据网格中点的个数

	double min_X = v_center(0) - f_adjust;
	double min_Y = v_center(1) - f_adjust;
	double min_Z = v_center(2) - f_adjust;

	double max_X = v_center(0) + f_adjust;
	double max_Y = v_center(1) + f_adjust;
	double max_Z = v_center(2) + f_adjust;

	//得到占据网格中的点，并将点的坐标存放在k_indices
	num = octree.boxSearch(v_min, v_max, k_indices);
	//std::cout << num << std::endl;

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>octree_temp(resolution);					//重新开辟一个八叉树，只存占据网格中的点
 	
	Occupiedgrid.cloud->width = k_indices.size();
	Occupiedgrid.cloud->height = 1;
	Occupiedgrid.cloud->resize(Occupiedgrid.cloud->width * Occupiedgrid.cloud->height);
	for (int i = 0; i < num; i++)
	{
		Occupiedgrid.cloud->points[i].x = cloud->points[k_indices[i]].x;
		Occupiedgrid.cloud->points[i].y = cloud->points[k_indices[i]].y;
		Occupiedgrid.cloud->points[i].z = cloud->points[k_indices[i]].z;
	}

	octree_temp.defineBoundingBox(min_X, min_Y, min_Z, max_X, max_Y, max_Z);		//定义八叉树的范围

	octree_temp.setInputCloud(Occupiedgrid.cloud);
	octree_temp.addPointsFromInputCloud();
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pointGrid;			//存放有点的体素的中心坐标
	Occupiedgrid.Number = octree_temp.getOccupiedVoxelCenters(pointGrid);			//得到有点的体素的中心坐标，存放在pointGrid中
}
void KeyPoint::get_TSDF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution = 0.02f, float f_adjust = 0.12f)                //TSDF距离场 参数列表：指向模型点云的指针，边界
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	Vector3f v_min;
	Vector3f v_max;
	Vector3f v_center;
	v_center << Key_coordinate.x, Key_coordinate.y, Key_coordinate.z;			//输入关键点的坐标，确定占据网格的中心

	Vector3f v_adjust;
	v_adjust << f_adjust, f_adjust, f_adjust;
	v_min << v_center - v_adjust;		//占据网格的左下角
	v_max << v_center + v_adjust;		//占据网格的右上角

	std::vector<int> k_indices;			//占据网格中点在点云中的索引
	int num;	//占据网格中点的个数

	double min_X = v_center(0) - f_adjust;
	double min_Y = v_center(1) - f_adjust;
	double min_Z = v_center(2) - f_adjust;

	double max_X = v_center(0) + f_adjust;
	double max_Y = v_center(1) + f_adjust;
	double max_Z = v_center(2) + f_adjust;

	Border[0] = min_X;
	Border[1] = min_Y;
	Border[2] = min_Z;
	Border[3] = max_X;
	Border[4] = max_Y;
	Border[5] = max_Z;


	//得到占据网格中的点，并将点的坐标存放在k_indices
	num = octree.boxSearch(v_min, v_max, k_indices);
	//std::cout << num << std::endl;

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>octree_temp(resolution);					//重新开辟一个八叉树，只存占据网格中的点
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_temp->width = k_indices.size();
	cloud_temp->height = 1;
	cloud_temp->points.resize(cloud_temp->width * cloud_temp->height);

	for (int i = 0; i < num; i++)
	{
		cloud_temp->points[i].x = cloud->points[k_indices[i]].x;
		cloud_temp->points[i].y = cloud->points[k_indices[i]].y;
		cloud_temp->points[i].z = cloud->points[k_indices[i]].z;
	}
	octree_temp.defineBoundingBox(min_X, min_Y, min_Z, max_X, max_Y, max_Z);	//定义八叉树的范围
	octree_temp.setInputCloud(cloud_temp);
	octree_temp.addPointsFromInputCloud();
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pointGrid;
	int num_center = octree_temp.getOccupiedVoxelCenters(pointGrid);

	for (int i = 0; i<12; i++)
		for (int j = 0; j<12; j++)
			for (int k = 0; k < 12; k++)
			{
				float _distance = 900;
				for (int p = 1; p < pointGrid.size(); p++)
				{
					int x = (pointGrid[p].x - min_X) / 0.02;	//间距
					int y = (pointGrid[p].y - min_Y) / 0.02;
					int z = (pointGrid[p].z - min_Z) / 0.02;
					float temp = sqrt(x*x + y*y + z*z);
					if (temp < _distance)
						_distance = temp;
				}
				grid_value[i][j][k] = _distance;
			}

}
