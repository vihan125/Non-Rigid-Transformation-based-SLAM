#ifndef INCLUDES_DEFORMATION_H_
#define INCLUDES_DEFORMATION_H_


#include <iostream>
#include <pcl/point_types.h>
#include <pcl/gpu/containers/device_array.h>

 /*check if the compiler is of C++*/
#ifdef __cplusplus

extern "C" bool deformCloud(pcl::gpu::DeviceArray<pcl::PointXYZRGB>& cloud_device, double *rot, double *trans, double *err_points, double *t_uniq, float *radius, int err_p_size);

#endif

#endif/* INCLUDES_DFORMATION_H_ */