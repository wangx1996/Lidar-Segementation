# Lidar-Segementation

[![build passing](https://img.shields.io/badge/build-passing-brightgreen.svg)](https://github.com/wangx1996/Lidar-Segementation) [![velodyne_HDL_64 compliant](https://img.shields.io/badge/velodyne_HDL_64-compliant-red.svg)](https://github.com/wangx1996/Lidar-Segementation)

An implementation on "Curved-Voxel Clustering for Accurate Segmentation of 3D LiDAR Point Clouds with Real-Time Performance" from IROS 2019

Paper Link [[link 1](https://datalab.snu.ac.kr/~ukang/papers/cvcIROS19.pdf)][[link 2](https://ieeexplore.ieee.org/document/8968026)]

### Updat on 20210825
1. Add a demo code
2. Fix some problem

### This file is just a function file, you may need to change a little bit to fit your own code

### How to use:

     vector<PointAPR> papr;
     calculateAPR(*cloud_gr,papr);
     unordered_map<int, Voxel> hvoxel;
     build_hash_table(papr,hvoxel);
     vector<int> cluster_index = CVC(hvoxel,papr);
     vector<int> cluster_id;
     most_frequent_value(cluster_index, cluster_id);
     
     
The output is the same as https://github.com/FloatingObjectSegmentation/CppRBNN


For the demo code:

     mkdir build
     cd build
     cmake ..
     make -j
     

### Reference
1. Part of this code references to the https://github.com/FloatingObjectSegmentation/CppRBNN 
2. Ground Remocve: https://github.com/LimHyungTae/patchwork

### Result:

![Image text](https://github.com/wangx1996/Lidar-Segementation/blob/master/img/result.png)
