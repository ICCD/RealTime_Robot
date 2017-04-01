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

struct Surface {														//�ָ�����
	double Area;			//���
	pcl::ModelCoefficients Coefficients;			//��Ĳ���		
	bool IsVertical;		//��ֱ��ֵΪ1��ˮƽ��Ϊ0		��ʱ������ƥ��
};

struct OccupiedGrid {
	pcl::PointCloud<pcl::PointXYZ> Mpoint;//����   �Ƿ���ModelPoint *Modelpoint �ȼۣ�
	float Border[6];                           //�߽�
	int Number;                               //�����е�ĸ���
};


class KeyPoint {														//�ؼ�����Ϣ��
public:
	OccupiedGrid Occupiedgrid;			//ռ������
	pcl::PointXYZ Key_coordinate;		//ռ�������йؼ�������
	vector<double> vector3D;  //��ά����
	float ***tsdf;


	void getOccupiedGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint, OccupiedGrid &Occupiedgrid);//�õ�ռ�������� �����б�ָ��ģ�͵��Ƶ�ָ�룬�ؼ������꣬ռ���������á�

	void get_TSDF(pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint, float Border[6]);                  //TSDF���볡 �����б�ָ��ģ�͵��Ƶ�ָ�룬�߽�
	void get_Vector3D(vector<Surface> &surface);					//��ȡ��ά����


};




class ModelPoint                                                          //ģ�͵�����
{
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint;						     //����ָ��
	vector<Surface>	surface;											//��ŷָ�õ������Ϣ
	pcl::PointCloud<pcl::PointXYZ> key_coordinates;						//���йؼ�������
	vector<KeyPoint> keyPoint;											//�ؼ��㼯��

	void getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint, vector<Surface> &surface);	   //��ά������ʾ�����С
	pcl::PointCloud<pcl::PointXYZ> getKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint); //��ȡ�ؼ��㺯�� �����б�ָ��ģ�͵��Ƶ�ָ�� ����ֵ ���ؼ�����������


																							//bool byHeight();                                           //����
																							//bool byArea();                                              //����
																							//bool byOccupied();                                          //����

};

/*�ؼ�����ȡ���� ����ָ����Ƶ�ָ�룬����ؼ������꼯��*/
pcl::PointCloud<pcl::PointXYZ> ModelPoint::getKeypoint(pcl::PointCloud<pcl::PointXYZ>::Ptr Mpoint)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	viewer->addPointCloud(Mpoint, "all_cloud");
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	harris.setInputCloud(Mpoint);
	harris.setNonMaxSupression(true);
	harris.setRadius(0.04f);
	harris.setThreshold(0.0012f);
	cloud_out->height = 1;
	cloud_out->width = 100;
	cloud_out->resize(cloud_out->height*Mpoint->width);
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
	return *cloud_harris;
}


//����
void ModelPoint::getArea(pcl::PointCloud<pcl::PointXYZ>::Ptr modelPoint, vector<Surface> &surface)								//�ָ���
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);	//ԭʼ����
	pcl::copyPointCloud��*modelPoint, *cloud_filtered);
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
		surface.push_back(s_temp);


		// �����˲�������
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
	}
}
