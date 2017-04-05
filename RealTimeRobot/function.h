#pragma once
//���ļ��洢���ú������������궨��
#include <key_point.h>
#include <pcl\registration\icp.h>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#define PI 3.1415926    //�����
#define RANSAC_TIMES  100 //?

//�ؼ����Ӧ�����ɵĵ��
struct PairPoint {
	KeyPoint point_i;
	KeyPoint point_j;
};

///<summary>
///ƥ���ϵɸѡ����
///</summary>
void Ransac(vector<PairPoint> pairpoint,int ransac_times) {
	int *count = new int[RANSAC_TIMES];

	for (int i = 0; i < RANSAC_TIMES; i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i; // ��������
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j;
		 //����������
		(*cloud_i).width = 5; //���õ��ƿ��
		(*cloud_i).height = 1; //���õ��Ƹ߶�
		(*cloud_i).is_dense = false; //���ܼ���
		(*cloud_i).points.resize((*cloud_i).width * (*cloud_i).height); //���Σ�����

		(*cloud_j).width = 5; //���õ��ƿ��
		(*cloud_j).height = 1; //���õ��Ƹ߶�
		(*cloud_j).is_dense = false; //���ܼ���
		(*cloud_j).points.resize((*cloud_j).width * (*cloud_j).height); //���Σ�����
		 //������Щ�������
		int pointnum = 0;
		for each (PairPoint pair in pairpoint)
		{
			(*cloud_i).points[pointnum] = pair.point_i.Key_coordinate;
			(*cloud_j).points[pointnum] = pair.point_j.Key_coordinate;
		}
		//*********************************
		// ICP��׼
		//*********************************
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //����ICP��������ICP��׼
		icp.setInputCloud(cloud_i); //�����������
		icp.setInputTarget(cloud_j); //����Ŀ����ƣ�������ƽ��з���任���õ�Ŀ����ƣ�
		pcl::PointCloud<pcl::PointXYZ> Final; //�洢���
											  //������׼������洢��Final��
		icp.align(Final);
		//������յı任����4x4��
		std::cout << icp.getFinalTransformation() << std::endl;
		icp.getFitnessScore();
		
	}
}

void  keyPointICP(pcl::PointCloud<pcl::PointXYZ>::Ptr SpointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud) {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //����ICP��������ICP��׼
	icp.setInputCloud(mPointCloud); //�����������
	icp.setInputTarget(SpointCloud); //����Ŀ����ƣ�������ƽ��з���任���õ�Ŀ����ƣ�
	pcl::PointCloud<pcl::PointXYZ> Final; //�洢���
										  //������׼������洢��Final��
	icp.align(Final);
	//������յı任����4x4��
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud(*mcloud,*transformed_cloud, icp.getFinalTransformation());

	//���浽PCD�ļ�
	pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud); //�����Ʊ��浽PCD�ļ���

	//��ʾ���ƴ���
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
	//pcl::io::savePCDFileASCII("70761_c.pcd", cheng); //�����Ʊ��浽PCD�ļ���
	int o = 0;
	while (true) {
		o--;
	}
}

bool match_by_height(pcl::PointXYZ &key1, pcl::PointXYZ &key2)		//���ø߶�ɸѡ
{
	float temp = key1.z / key2.z;
	if (temp >= float(2 / 3) || temp <= 1.5)
		return true;
	return false;
}

bool match_by_area(double *v1, double *v2)					//�������ɸѡ
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

