#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/PCLHeader.h>
#include <pcl/exceptions.h>
#include <pcl/point_traits.h>
#include<vector>
template <typename PointT>
class PCL_EXPORTS ScanPoint
{
public:
	pcl::PointCloud<pcl::PointXYZ> *scanpoint;
	struct Surface {
		double Area;
		vector<float>Normal;
		²ÎÊý£¿

	};
	void getArea(*ScanPoint, &Surface)

		class KeyPoint {
		public:
			struct OccupiedGrid {
				pcl::PointCloud<pcl::PointXYZ> *scanpoint;
				int Number;
				float Border[6];
			};
			void getOccupiedGrid(*ScanPoint, pcl::PointXYZ, &OccupiedGrid);
			pcl::PointXYZ getKeypoint(*ScanPoint);
			vector<vector<vector<double>>> vector3D(Surface S);
			class ScanFeature {
			public:
				void getOccupiedGrid(*ModelPoint, pcl::PointXYZ, &OccupiedGrid);
				int surfaceArea keypointArea(pcl::PointXYZ);
				vector<vector<vector<vector<vector<vector<double>>>>>> get6Dvector(pcl::PointXYZ);
			};
	};
	bool byHeight();
	bool byArea();
	bool byOccupied();

};
e

