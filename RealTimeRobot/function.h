#pragma once
//���ļ��洢���ú������������궨��
#include <key_point.h>
#include <pcl\registration\icp.h>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <time.h>
#include <iostream>
#include"stdlib.h"

#define PI 3.1415926    //�����
#define RANSAC_TIMES  100 //?

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
#define Random(x) (rand() % x) 

//�ؼ����Ӧ�����ɵĵ��
struct PairPoint {
	KeyPoint point_i;
	KeyPoint point_j;
};
double pointdistance(pcl::PointXYZ p1, pcl::PointXYZ p2) {
	
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) + (p1.z - p2.z)*(p1.z - p2.z));
}

///<summary>
///ƥ���ϵɸѡ����
///</summary>
Eigen::Matrix4f Ransac(vector<PairPoint> pairpoint,int ransac_times, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud) {
	Eigen::Matrix4f matrix;
	vector<Eigen::Matrix4f> em;
	double minidistance = 10000000.0;
	Eigen::Matrix4f bestMatrix;
	int p=0;
	int best_innum = 0;
	for (int i = 0; i < pairpoint.size(); i++)
	{
		//���ȡһ�Թؼ���
		//int int_rand = rand() % pairpoint.size();
		PairPoint pair = pairpoint[i];
		// ���ݸöԹؼ��������ת����
		get_Distance(matrix, pair.point_i,pair.point_j);
		//��תɨ����ƵĹؼ��㼯��
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>); // ��������,ģ�͵���
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j(new pcl::PointCloud<pcl::PointXYZ>);
		
			(*cloud_i).height = 1; //���õ��Ƹ߶�
			(*cloud_i).width = pairpoint.size(); //���õ��ƿ��
			(*cloud_i).points.resize((*cloud_i).width * (*cloud_i).height); //���Σ�����

			(*cloud_j).width = pairpoint.size(); //���õ��ƿ��
			(*cloud_j).height = 1; //���õ��Ƹ߶�
			(*cloud_j).points.resize((*cloud_j).width * (*cloud_j).height); //���Σ�����

			int pointnum = 0;
			for each (PairPoint pp in pairpoint)
			{
				(*cloud_i).points[pointnum] = pp.point_i.Key_coordinate;
				(*cloud_j).points[pointnum++] = pp.point_j.Key_coordinate;
			}
			pcl::transformPointCloud(*cloud_j, *cloud_j, matrix);		//�任
		//����˴�ƥ��ľ��ڵ���
			Eigen::Matrix4f noUse;
			int	inline_num = 0;
		
			for (int j = 0; j < pairpoint.size(); j++)
			{
				double dis = pointdistance((*cloud_i).points[j], (*cloud_j).points[j]);
				//cout << dis << "ffffffffff" << endl;
				float  des = get_Distance(noUse, pairpoint[j].point_i, pairpoint[j].point_j);
	
				if (dis<0.15&&des<100)
				{
					inline_num++;
				}
			}
			if (inline_num > best_innum) {
				bestMatrix = matrix;
				best_innum = inline_num;
				p = 0;
				em.clear();
			}else if(inline_num=best_innum) {
				cout << p++ << "+++++++++++++++++++++++++++++" << endl;
				em.push_back(matrix);
			//pcl::transformPointCloud(*cloud, *cloud, matrix);
			//pcl::io::savePCDFileASCII("mcloud1.pcd", *mcloud); //�����Ʊ��浽PCD�ļ���
			//pcl::io::savePCDFileASCII("transformed_cloud1.pcd", *cloud);
			//system("pcl_viewer.exe  mcloud1.pcd transformed_cloud1.pcd");
			}
			cout << inline_num << "================================" << endl;
		
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr gg(new pcl::PointCloud<pcl::PointXYZ>);
	//for each (Eigen::Matrix4f var in em)
	//{
	//	pcl::transformPointCloud(*cloud, *gg, var);
	//	pcl::io::savePCDFileASCII("mcloud1.pcd", *mcloud); //�����Ʊ��浽PCD�ļ���
	//	pcl::io::savePCDFileASCII("transformed_cloud1.pcd", *gg);
	//	system("pcl_viewer.exe  mcloud1.pcd transformed_cloud1.pcd");
	//}
		return bestMatrix;
}

void  keyPointICP(pcl::PointCloud<pcl::PointXYZ>::Ptr SpointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mPointCloud , pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr mcloud) {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp; //����ICP��������ICP��׼
	icp.setInputCloud(mcloud); //�����������
	icp.setInputTarget(cloud); //����Ŀ����ƣ�������ƽ��з���任���õ�Ŀ����ƣ�
	pcl::PointCloud<pcl::PointXYZ> Final; //�洢���
										  //������׼������洢��Final��
	icp.align(Final);
	//������յı任����4x4��
	std::cout << icp.getFinalTransformation() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud(*mcloud,*transformed_cloud, icp.getFinalTransformation());

	//���浽PCD�ļ�
	pcl::io::savePCDFileASCII("mcloud.pcd", *mcloud); //�����Ʊ��浽PCD�ļ���
	pcl::io::savePCDFileASCII("transformed_cloud.pcd", *transformed_cloud);
	//��ʾ���ƴ���
	system("pcl_viewer.exe 11351.pcd mcloud.pcd");
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

/**
*���룺�ؼ���ԣ�PairPoint
*�����4*4���� ��Matrix4
*��;�����ݹؼ���Լ�����Ա任����
**/
//Eigen::Matrix4f getMatrixfromPairPoint(PairPoint p) {
//	
//}
bool match_by_height(pcl::PointXYZ &key1, pcl::PointXYZ &key2)		
{
	float temp = key1.z / key2.z;
	if (temp >= float(2 / 3) || temp <= 1.5)
		return true;
	return false;
}
bool match_by_area(vector<double> v1, vector<double> v2)
{
	if ((v1[0] / v2[0]) > 3 || (v1[0] / v2[0]) < float(1 / 3) || (v1[1] / v2[1]) > 3 || (v1[1] / v2[1]) < float(1 / 3) || (v1[2] / v2[2]) > 3 || (v1[2] / v2[2]) < float(1 / 3))
		return false;
	return true;
}
bool match_by_occupied(OccupiedGrid &o1, OccupiedGrid &o2)
{
	if (o2.Number == 0) return false;	//defined by xueyu ��Q�������㆖�}
	float temp = o1.Number / o2.Number;
	if (temp > 2 || temp < 0.5)
		return false;
	return true;
}

