# Lidar-Segementation
An implementation on "Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance" from IROS 2019

1.https://datalab.snu.ac.kr/~ukang/papers/cvcIROS19.pdf 

2.https://ieeexplore.ieee.org/document/8968026

Part of this code references to the https://github.com/FloatingObjectSegmentation/CppRBNN 

This file is just a function file, you may need to change a little bit to fit your own code

### How to use:

     vector<PointAPR> papr;
     calculateAPR(*cloud_gr,papr);
     unordered_map<int, Voxel> hvoxel;
     build_hash_table(papr,hvoxel);
     vector<int> cluster_index = CVC(hvoxel,papr);
     vector<int> cluster_id;
     most_frequent_value(cluster_index, cluster_id);
     
     
The output is the same as https://github.com/FloatingObjectSegmentation/CppRBNN

### Result:

![Image text](https://github.com/WAN96/Lidar-Segementation/blob/master/Screenshot%20from%202019-12-27%2014-13-13.png)
