#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/PCLHeader.h>
#include <pcl/exceptions.h>
#include <pcl/point_traits.h>
#include<vector>
template <typename PointT>
class PCL_EXPORTS ModelPoint
{
public:
	pcl::PointCloud<pcl::PointXYZ> *modelpoint;
	struct Surface {
		double Area;
		vector<float>Normal;
		²ÎÊý£¿

	};
	void getArea(*ModelPoint, &Surface)

		class KeyPoint {
		public:
			struct OccupiedGrid {
				pcl::PointCloud<pcl::PointXYZ> *modelpoint;
				int Number;
				float Border[6];
			};
			void getOccupiedGrid(*ModelPoint, pcl::PointXYZ, &OccupiedGrid);
			pcl::PointXYZ getKeypoint(*ModelPoint);
			float[][][] TSDF(*ModelPoint, Border[6]);
			vector<vector<vector<double>>> vector3D(Surface S);
			class ModelFeature {
			public:
				void getOccupiedGrid(*ModelPoint, pcl::PointXYZ, &OccupiedGrid);
				float[][][] TSDF(*ModelPoint, Border[6]);
				int surfaceArea keypointArea(pcl::PointXYZ);
				vector<vector<vector<vector<vector<vector<double>>>>>> get6Dvector(pcl::PointXYZ);

			};
	};
	bool byHeight();
	bool byArea();
	bool byOccupied();

};

