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
#include <iostream>
#include <unordered_map>
#include <algorithm>
#include<pcl/filters/voxel_grid.h> 

using namespace std;
const float PI = 3.1415926;
template<typename T> string toString(const T& t) {
	ostringstream oss;
	oss << t;
	return oss.str();
}
static int64_t gtm() {
	struct timeval tm;
	gettimeofday(&tm, 0);
	int64_t re = (((int64_t) tm.tv_sec) * 1000 * 1000 + tm.tv_usec);
	return re;
}

float Polar_angle_cal(float x, float y){
  float temp_tangle = 0;
  if(x== 0 && y ==0){
     temp_tangle = 0;
  }else if(y>=0){
     temp_tangle = (float)atan2(y,x);
  }else if(y<=0){
     temp_tangle = (float)atan2(y,x)+2*PI;
  }
 return temp_tangle;
}




struct PointAPR{
   float azimuth;
   float polar_angle;
   float range;
};

struct Voxel{
   bool haspoint = false;
   int cluster = -1;
   vector<int> index;
};


float minrange = 3;
float maxrange = 3;
float minazimuth = 0;
float maxazimuth = 0;
float deltaA = 2;
float deltaR = 0.35;
float deltaP = 1.2;
int length = 0;
int width = 0;
int height = 0;

bool compare_cluster(pair<int,int> a,pair<int,int> b){
    return a.second>b.second;
}//升序


template <typename PointT>
void calculateAPR(const pcl::PointCloud<PointT>& cloud_IN, vector<PointAPR>& vapr){
     for (int i =0; i<cloud_IN.points.size(); ++i){
           PointAPR par;
           par.polar_angle = Polar_angle_cal(cloud_IN.points[i].x, cloud_IN.points[i].y);
           par.range = sqrt(cloud_IN.points[i].x*cloud_IN.points[i].x+cloud_IN.points[i].y*cloud_IN.points[i].y);
           par.azimuth =(float) atan2(cloud_IN.points[i].z,par.range);
           if(par.azimuth < minazimuth){
                 minazimuth = par.azimuth;
           }
           if(par.azimuth > maxazimuth){
                 maxazimuth = par.azimuth;
           }
           if(par.range < minrange){
                 minrange = par.range;
           }
           if(par.range > maxrange){
                 maxrange = par.range;
           }
           vapr.push_back(par);
    }

  length = round((maxrange - minrange)/deltaR);
  width = 301;
  height = round((maxazimuth - minazimuth)/deltaA);
  
}


void build_hash_table(const vector<PointAPR>& vapr, unordered_map<int, Voxel> &map_out){
     vector<int> ri;
     vector<int> pi;
     vector<int> ai;
     for(int i =0; i< vapr.size(); ++i){
           int azimuth_index = round(((vapr[i].azimuth-minazimuth)*180/PI)/deltaA);
           int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
           int range_index = round((vapr[i].range-minrange)/deltaR);
           int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);
           ri.push_back(range_index);
           pi.push_back(polar_index);           
           ai.push_back(azimuth_index);
           unordered_map<int, Voxel>::iterator it_find;
           it_find = map_out.find(voxel_index);
           if (it_find != map_out.end()){
                it_find->second.index.push_back(i);

           }else{
                Voxel vox;
                vox.haspoint =true;
                vox.index.push_back(i); 
                vox.index.swap(vox.index);
                map_out.insert(make_pair(voxel_index,vox));
           }

    }
    auto maxPosition = max_element(ai.begin(), ai.end());
    auto maxPosition1 = max_element(ri.begin(), ri.end());
    auto maxPosition2 = max_element(pi.begin(), pi.end());
    cout<<*maxPosition<<" "<<*maxPosition1<<" "<<*maxPosition2<<endl;

}
     

void find_neighbors(int polar, int range, int azimuth, vector<int>& neighborindex){
	for (int z = azimuth - 1; z <= azimuth + 1; z++){
		if (z < 0 || z >round((maxazimuth-minazimuth)*180/PI/deltaA)){
			continue;
		}

		for (int y = range - 1; y <= range + 1; y++){
			if (y < 0 || y >round((50-minrange)/deltaR)){
				continue;
			}

			for (int x = polar - 1; x <= polar + 1; x++){
                                int px = x;
				if (x < 0 ){
					px=300;
				}
                                if(x>300){
                                        px=0;

                                }

				neighborindex.push_back((px*(length+1)+y)+z*(length+1)*(width+1));
                        }
               }
        }
}


bool most_frequent_value(vector<int> values, vector<int> &cluster_index) {
	unordered_map<int, int> histcounts;
	for (int i = 0; i < values.size(); i++) {
		if (histcounts.find(values[i]) == histcounts.end()) {
			histcounts[values[i]] = 1;
		}
		else {
			histcounts[values[i]] += 1;
		}
	}

	int max = 0, maxi;
	vector<pair<int, int>> tr(histcounts.begin(), histcounts.end());
        sort(tr.begin(),tr.end(),compare_cluster);
        for(int i = 0 ; i< tr.size(); ++i){
             if(tr[i].second>10){
             cluster_index.push_back(tr[i].first);
             }
        }

	return true;
}


void mergeClusters(vector<int>& cluster_indices, int idx1, int idx2) {
	for (int i = 0; i < cluster_indices.size(); i++) {
		if (cluster_indices[i] == idx1) {
			cluster_indices[i] = idx2;
		}
	}
}

vector<int>  CVC(unordered_map<int, Voxel> &map_in,const vector<PointAPR>& vapr){
     int current_cluster = 0;
     cout<<"CVC"<<endl;
     vector<int> cluster_indices = vector<int>(vapr.size(), -1);

     for(int i = 0; i< vapr.size(); ++i){

           if (cluster_indices[i] != -1)
			continue;
           int azimuth_index = round((vapr[i].azimuth+fabs(minazimuth))*180/PI/deltaA);
           int polar_index = round(vapr[i].polar_angle*180/PI/deltaP);
           int range_index = round((vapr[i].range-minrange)/deltaR);
           int voxel_index = (polar_index*(length+1)+range_index)+azimuth_index*(length+1)*(width+1);
           
           unordered_map<int, Voxel>::iterator it_find;
           unordered_map<int, Voxel>::iterator it_find2;

           it_find = map_in.find(voxel_index);
           vector<int> neightbors;

           if (it_find != map_in.end()){

               vector<int> neighborid;
               find_neighbors(polar_index, range_index, azimuth_index, neighborid);
               for (int k =0; k<neighborid.size(); ++k){
                     
                  it_find2 = map_in.find(neighborid[k]);

                  if (it_find2 != map_in.end()){

                     for(int j =0 ; j<it_find2->second.index.size(); ++j){
                        neightbors.push_back(it_find2->second.index[j]);
                      }
                   }
                }
            }
       
            neightbors.swap(neightbors);

            if(neightbors.size()>0){
                   for(int j =0 ; j<neightbors.size(); ++j){
                      int oc = cluster_indices[i] ;
                      int nc = cluster_indices[neightbors[j]];
		      if (oc != -1 && nc != -1) {
				if (oc != nc)
					mergeClusters(cluster_indices, oc, nc);
				}
		      else {
				if (nc != -1) {
					cluster_indices[i] = nc;
				}
				else {
					if (oc != -1) {
						cluster_indices[neightbors[j]] = oc;
					}
				}
			}

                   }
                }
                          
 		if (cluster_indices[i] == -1) {
			current_cluster++;
			cluster_indices[i] = current_cluster;
                   for(int s =0 ; s<neightbors.size(); ++s){             
                        cluster_indices[neightbors[s]] = current_cluster;
		   }
               }


          }
	return cluster_indices;
}


vector<float> hsv2rgb(vector<float>& hsv){
vector<float> rgb(3);
float R,G,B,H,S,V;
H = hsv[0];
S = hsv[1];
V = hsv[2];
if(S == 0){
rgb[0]=rgb[1]=rgb[2]=V;
}else{

int i = int(H*6);
float f = (H*6) - i;
float a = V * ( 1 - S );
float b = V * ( 1 - S * f );
float c = V * ( 1 - S * (1 - f ) );
i = i%6;
switch(i){
   case 0: {rgb[0] = V; rgb[1]= c; rgb[2] = a; break;}
   case 1: {rgb[0] = b; rgb[1] = V; rgb[2] = a;break;}
   case 2: {rgb[0] = a; rgb[1] = V; rgb[2] = c;break;}
   case 3: {rgb[0] = a; rgb[1] = b; rgb[2] = V;break;}
   case 4: {rgb[0] = c; rgb[1] = a; rgb[2] = V;break;}
   case 5: {rgb[0] = V; rgb[1] = a; rgb[2] = b;break;}
  }
}

return rgb;
}
