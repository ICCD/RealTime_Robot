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

struct Surface {														//��
	double Area;			//���
	pcl::ModelCoefficients Coefficients;			//��Ĳ���
	bool IsVertical;		//��ֱ��ֵΪ1��ˮƽ��Ϊ0
};

struct OccupiedGrid {
	pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint;//����   �Ƿ���ScanPoint *Scanpoint �ȼۣ�
	float Border[6];                           //�߽�
	int Number;                               //�����е�ĸ���
};


class KeyPoint {
public:
	OccupiedGrid Occupiedgrid;
	void getOccupiedGrid(ScanPoint *Spoint, pcl::PointCloud<pcl::PointXYZ>::Ptr point, OccupiedGrid &);//�õ�ռ�������� �����б�ָ��ģ�͵��Ƶ�ָ�룬�ؼ������꣬ռ���������á�
	pcl::PointCloud<pcl::PointXYZ>::Ptr getKeypoint(ScanPoint *Spoint); //��ȡ�ؼ��㺯�� �����б�ָ��ģ�͵��Ƶ�ָ�� ����ֵ ���ؼ�����������
	vector<vector<vector<double>>> vector3D(Surface S);  //��ά����
														 //�ؼ�����Ƭ��С��6D vector��
};



class ScanPoint                                                          //ɨ�������
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint;						     //����ָ��
	vector<Surface>	surface;											//��ŷָ�õ������Ϣ
	void getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr Spoint, vector<Surface> &surface);	   //��ά������ʾ�����С
	bool byHeight();                                           //����
	bool byArea();                                              //����
	bool byOccupied();                                          //����

};
/*�ؼ�����ȡ���� ����ָ����Ƶ�ָ�룬����ؼ������꼯��*/
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


