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

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace std;
using namespace pcl;

#define PI 3.1415926    //�����

double						//������ռ��е㵽ƽ�����ĺ���
getDistance(float v1, float v2, float v3, float v4, pcl::PointXYZ point)
{
	double d = sqrt(v1*v1 + v2*v2 + v3*v3);
	return abs(v1*point.x + v2*point.y + v3*point.z + v4) / d;
}






struct Surface {														//�ָ�����
	double Area;			//���
	pcl::ModelCoefficients Coefficients;			//��Ĳ���		
	bool IsVertical;		//��ֱ��ֵΪ1��ˮƽ��Ϊ0		��ʱ������ƥ��
};

struct OccupiedGrid {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;//����   �Ƿ���ModelPoint *Modelpoint �ȼۣ�
	float Border[6];                           //�߽�
	int Number;                               //�е�����صĸ���		ռ�ݱ��ɴ�ɸѡ
};


class KeyPoint {														//�ؼ�����Ϣ��
public:
	OccupiedGrid Occupiedgrid;			//ռ������
	pcl::PointXYZ Key_coordinate;		//ռ�������йؼ�������
	vector<double> vector3D;  //��ά����		�����ʼ��Ϊ0.04
	

	void getOccupiedGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, OccupiedGrid &Occupiedgrid, float resolution = 0.02f, float f_adjust = 0.08f);//�õ�ռ�������� �����б�ָ��ģ�͵��Ƶ�ָ�룬�ؼ������꣬ռ���������á�resolution ��f_adjust�ֱ����ֱ��ʺ�ռ������߳�һ��

	
	void get_Vector3D(vector<Surface> &surface, vector<double> &vector3D);					//��ȡ��ά����


};

void KeyPoint::get_Vector3D(vector<Surface> &surface, vector<double> &vector3D)					//��ȡ��ά����			Ŀǰ��ʵ��
{
	int vertical = 0, horizontal = 0;
	for (int i = 0; i < surface.size(); i++)
	{
		double distance = getDistance(surface[i].Coefficients.values[0], surface[i].Coefficients.values[1], surface[i].Coefficients.values[2], surface[i].Coefficients.values[3], Key_coordinate);//�㵽ƽ�����
		if (surface[i].IsVertical == 0 && horizontal == 0 && distance <= 0.02)		//ˮƽ�� ֻ��һ��
		{
			vector3D[0] = surface[i].Area;
			horizontal++;
		}
		else if (surface[i].IsVertical == 0 && horizontal <= 1 && distance <= 0.02)			//��ֱ���������
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
void KeyPoint::getOccupiedGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, OccupiedGrid &Occupiedgrid, float resolution = 0.02f, float f_adjust = 0.08f)//�õ�ռ�������� �����б�ָ��ģ�͵��Ƶ�ָ�룬�ؼ������꣬ռ���������á�
{
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>octree(resolution);	//�˲���
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	Vector3f v_min;
	Vector3f v_max;
	Vector3f v_center;
	v_center << Key_coordinate.x, Key_coordinate.y, Key_coordinate.z;			//����ؼ�������꣬ȷ��ռ�����������

	Vector3f v_adjust;
	v_adjust << f_adjust, f_adjust, f_adjust;
	v_min << v_center - v_adjust;		//ռ����������½�
	v_max << v_center + v_adjust;		//ռ����������Ͻ�

	std::vector<int> k_indices;			//ռ�������е��ڵ����е�����
	int num;	//ռ�������е�ĸ���

	double min_X = v_center(0) - f_adjust;
	double min_Y = v_center(1) - f_adjust;
	double min_Z = v_center(2) - f_adjust;

	double max_X = v_center(0) + f_adjust;
	double max_Y = v_center(1) + f_adjust;
	double max_Z = v_center(2) + f_adjust;

	//�õ�ռ�������еĵ㣬���������������k_indices
	num = octree.boxSearch(v_min, v_max, k_indices);
	//std::cout << num << std::endl;

	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ>octree_temp(resolution);					//���¿���һ���˲�����ֻ��ռ�������еĵ�
	Occupiedgrid.cloud->width = k_indices.size();
	Occupiedgrid.cloud->height = 1;
	Occupiedgrid.cloud->resize(Occupiedgrid.cloud->width * Occupiedgrid.cloud->height);
	for (int i = 0; i < num; i++)
	{
		Occupiedgrid.cloud->points[i].x = cloud->points[k_indices[i]].x;
		Occupiedgrid.cloud->points[i].y = cloud->points[k_indices[i]].y;
		Occupiedgrid.cloud->points[i].z = cloud->points[k_indices[i]].z;
	}

	octree_temp.defineBoundingBox(min_X, min_Y, min_Z, max_X, max_Y, max_Z);		//����˲����ķ�Χ

	octree_temp.setInputCloud(Occupiedgrid.cloud);
	octree_temp.addPointsFromInputCloud();
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> pointGrid;			//����е�����ص���������
	Occupiedgrid.Number = octree_temp.getOccupiedVoxelCenters(pointGrid);			//�õ��е�����ص��������꣬�����pointGrid��
}









class ScanPoint                                                          //ģ�͵�����
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint;						     //����ָ��
	vector<Surface>	surface;											//��ŷָ�õ������Ϣ
	pcl::PointCloud<pcl::PointXYZ> key_coordinates;						//���йؼ�������
	vector<KeyPoint> keyPoint;											//�ؼ��㼯��

	void getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint, vector<Surface> &surface);	   //��ά������ʾ�����С
	pcl::PointCloud<pcl::PointXYZ> getKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint); //��ȡ�ؼ��㺯�� �����б�ָ��ģ�͵��Ƶ�ָ�� ����ֵ ���ؼ�����������


};

/*�ؼ�����ȡ���� ����ָ����Ƶ�ָ�룬����ؼ������꼯��*/
pcl::PointCloud<pcl::PointXYZ> ScanPoint::getKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint)
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
	return *cloud_harris;
}


//
void ScanPoint::getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr scanPoint, vector<Surface> &surface)								//�ָ���
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);	//ԭʼ����
	pcl::copyPointCloud(*scanPoint, *cloud_filtered);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);		//ÿ�ηָ���ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);		//�ָ��ʣ�µĵ���

	pcl::ModelCoefficients::Ptr Coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	// �����ָ����
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// ��ѡ
	seg.setOptimizeCoefficients(true);
	// ��ѡ
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.005);

	// �����˲�������
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	int  nr_points = (int)cloud_filtered->points.size();
	// ������30%ԭʼ��������ʱ
	while (cloud_filtered->points.size() > 0.15 * nr_points)
	{
		// �����µĵ����зָ����ƽ����ɲ���
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *Coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}
		// �����ڲ�
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);		//�ָ����ƽ�����cloud_p��
										//std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;


		pcl::ConvexHull<pcl::PointXYZ> chull;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
		chull.setComputeAreaVolume(true);

		chull.setInputCloud(cloud_p);
		// chull.setAlpha (0.1);
		chull.reconstruct(*cloud_hull);
		Surface s_temp;
		s_temp.Area = chull.getTotalArea();		//�õ����
		s_temp.Coefficients = *Coefficients;
	//	if ()
			surface.push_back(s_temp);


		// �����˲�������
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
	}
}
