#ifndef CVC_CLUSTER_H
#define CVC_CLUSTER_H

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

//C++
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <algorithm>
#include <limits>

template<typename T> 
std::string toString(const T& t) {
	std::ostringstream oss;
	oss << t;
	return oss.str();
}


struct PointAPR{
   float azimuth;
   float polar_angle;
   float range;
};

struct Voxel{
   bool haspoint = false;
   int cluster = -1;
   std::vector<int> index;
};



class CVC{
	public:
	CVC();
	CVC(std::vector<float>& param){
		if(param.size() != 3){
			printf("Param number is not correct!");
			std::abort();		
		}
		for(int i=0; i<param.size(); ++i){
			deltaA_ = param[0];
			deltaR_ = param[1];
			deltaP_ = param[2];
		}
	}

	~CVC(){}
	void calculateAPR(const pcl::PointCloud<pcl::PointXYZ>& cloud_IN, std::vector<PointAPR>& vapr);
	void build_hash_table(const std::vector<PointAPR>& vapr, std::unordered_map<int, Voxel> &map_out);
	void find_neighbors(int polar, int range, int azimuth, std::vector<int>& neighborindex);
	bool most_frequent_value(std::vector<int> values, std::vector<int> &cluster_index);
	void mergeClusters(std::vector<int>& cluster_indices, int idx1, int idx2);
	std::vector<int>  cluster(std::unordered_map<int, Voxel> &map_in,const std::vector<PointAPR>& vapr);
	void process();

private:
	float deltaA_ = 2;
	float deltaR_ = 0.35;
	float deltaP_ = 1.2;
	float min_range_ = std::numeric_limits<float>::max();
	float max_range_ = std::numeric_limits<float>::min();
	float min_azimuth_ = -24.8 * M_PI/180;
	float max_azimuth_ = 2 * M_PI/180;
	int length_ = 0;
	int width_  = 0;
	int height_ = 0;
};

#endif


