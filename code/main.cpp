#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h> 
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <math.h>
//PCL
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
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h> 
#include "patchwork.hpp"
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>//自定义点云类型时要加
#include <pcl/filters/conditional_removal.h>
//Eigen
#include <Eigen/Dense>
#include <queue> 

#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "CVC_cluster.h"

using namespace std;

struct PointXYZILID
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t label;                     ///< point label
  uint16_t id;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint16_t, id, id))


struct PointXYZIR{
	PCL_ADD_POINT4D;
	float intensity;
	uint16_t ring;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
		(float,x,x)
		(float,y,y)
		(float,z,z)
		(float,intensity,intensity)
		(uint16_t,ring,ring)
)


void PointXYZILID2XYZI(pcl::PointCloud<PointXYZILID>& src,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr dst){
  dst->points.clear();
  for (const auto &pt: src.points){
    pcl::PointXYZI pt_xyzi;
    pt_xyzi.x = pt.x;
    pt_xyzi.y = pt.y;
    pt_xyzi.z = pt.z;
    pt_xyzi.intensity = pt.intensity;
    dst->points.push_back(pt_xyzi);
  }
}


using PointType = PointXYZILID;
boost::shared_ptr<PatchWork<PointType> > PatchworkGroundSeg;


int main(int argc, char**argv) {

       pcl::PointCloud<PointXYZIR>::Ptr cloud(new pcl::PointCloud<PointXYZIR>);//初始化点云 
       pcl::io::loadPCDFile<PointXYZIR>(argv[1], *cloud);//加载pcd点云并放入cloud中
    

       PatchworkGroundSeg.reset(new PatchWork<PointXYZILID>());
       pcl::PointCloud<PointType>::Ptr pc_curr(new pcl::PointCloud<PointType>);
       pcl::PointCloud<PointType>::Ptr pc_ground(new pcl::PointCloud<PointType>);
       pcl::PointCloud<PointType>::Ptr pc_non_ground(new pcl::PointCloud<PointType>);
       double time_taken;

	for(int i=0; i<cloud->points.size(); ++i){
		PointType p;
		p.x = cloud->points[i].x;
		p.y = cloud->points[i].y;
		p.z = cloud->points[i].z;
		p.intensity = cloud->points[i].intensity;
		pc_curr->points.push_back(p);
	}

        PatchworkGroundSeg->estimate_ground(*pc_curr, *pc_ground, *pc_non_ground, time_taken);

	cout<<time_taken<<endl;

	vector<float> param(3,0);
	param[0] = 2;
	param[1] = 0.4;
	param[2] = 1.5;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_point(new pcl::PointCloud<pcl::PointXYZ>);

	for(int i=0; i<pc_non_ground->points.size(); ++i){
		pcl::PointXYZ p;
		p.x = pc_non_ground->points[i].x;
		p.y = pc_non_ground->points[i].y;
		p.z = pc_non_ground->points[i].z;
		cluster_point->points.push_back(p);
	}


	cout<<"cluster_point "<<cluster_point->points.size()<<endl;
	CVC Cluster(param);
	std::vector<PointAPR> capr;
	Cluster.calculateAPR(*cluster_point, capr);

	cout<<"capr "<<capr.size()<<endl;
	std::unordered_map<int, Voxel> hash_table;
	Cluster.build_hash_table(capr, hash_table);
	vector<int> cluster_indices;	
	cluster_indices = Cluster.cluster(hash_table, capr);
	vector<int> cluster_id;
	Cluster.most_frequent_value(cluster_indices, cluster_id);

	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
			new pcl::visualization::PCLVisualizer("pcd")); //PCLVisualizer 可视化类
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1);

	cv::RNG rng(12345);

        int r = rng.uniform(0, 200);
        int g = rng.uniform(0, 200);
        int b = rng.uniform(0, 200);
	pcl::visualization::PointCloudColorHandlerCustom <PointType> colorob(pc_ground, (r), (g),(b));
	string csob = "cloudfinalob";
	viewer->addPointCloud(pc_ground,colorob, csob);


	//pcl::visualization::PointCloudColorHandlerCustom <PointType> colorob2(pc_non_ground, (0), (255),(0));
	//string csob2 = "cloudfinalob1";
	//viewer->addPointCloud(pc_non_ground,colorob2, csob2);


	for(int j = 0; j < cluster_id.size(); ++j){
                int r = rng.uniform(20, 255);
                int g = rng.uniform(20, 255);
                int b = rng.uniform(20, 255);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudcluster(new pcl::PointCloud<pcl::PointXYZ>);//初始化
		for(int i =0; i<cluster_indices.size(); ++i){
			if(cluster_indices[i] == cluster_id[j]){
				 cloudcluster->points.push_back(cluster_point->points[i]);
			}
		}

		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> color(cloudcluster, (r), (g),(b));
		string text = "cloud" + toString(j);
		viewer->addPointCloud(cloudcluster,color, text);
	}


	while (!viewer->wasStopped()) {
		viewer->spin();
	}

    return 0;
}



