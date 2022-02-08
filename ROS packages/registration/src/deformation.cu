#include "deformation.h"
#include <thrust/device_vector.h>
#include <cuda.h>

  
 __global__ void deform_points(pcl::gpu::PtrSz<pcl::PointXYZRGB> cloud_device, double *rot, double *trans, double *err_points, double *t_uniq, float *radius, int *err_p_size)
{

    int errnous_points = *err_p_size;
    int common_rot_applied = 0;

    int point_position = threadIdx.x + (blockIdx.x * blockDim.x); 

    // checking weather any point is a nan point
    if(!(isnan(cloud_device[point_position].x) || isnan(cloud_device[point_position].y) || isnan(cloud_device[point_position].z))){
        
        double point_x = cloud_device[point_position].x;
        double point_y = cloud_device[point_position].y;
        double point_z = cloud_device[point_position].z;

        float r = *radius;
        float max_weight = 0;

        // loop in errrnous points and apply tranformations 
        for(int i = 0; i < errnous_points; i++){

            double err_p_x = err_points[(i*3)+0];
            double err_p_y = err_points[(i*3)+1];
            double err_p_z = err_points[(i*3)+2];

            // 1. calculate distance between two points (squared distance)
            double x_dif = (point_x - err_p_x);
            double y_dif = (point_y - err_p_y);
            double z_dif = (point_z - err_p_z);

            float d = (x_dif*x_dif) + (y_dif*y_dif) + (z_dif*z_dif);

            // 2. check if the distance is less than or equal to the radius provided

            if(d<=r){

                // 3. calculate weight for distribution of translation

                float weight = (1 - (d/r));

                // 4. check if the common rotation is applied earlier
                if(common_rot_applied){

                    // 5.1. if yes apply only the unique translation
                    point_x = point_x + (weight*t_uniq[(3*i)+0]);
                    point_y = point_y + (weight*t_uniq[(3*i)+1]);
                    point_z = point_z + (weight*t_uniq[(3*i)+2]);   

                    if (weight > max_weight){
                        max_weight = weight;
                    }
                }

                else{

                    // 5.2. if not apply both (rotation + unique) and set the flag
                    point_x = (point_x*rot[0] + point_y*rot[1] +point_z*rot[2]) + (weight*t_uniq[(3*i)+0]);
                    point_y = (point_x*rot[3] + point_y*rot[4] +point_z*rot[5]) + (weight*t_uniq[(3*i)+1]);
                    point_z = (point_x*rot[6] + point_y*rot[7] +point_z*rot[8]) + (weight*t_uniq[(3*i)+2]);

                    common_rot_applied = 1;

                    if (weight > max_weight){
                        max_weight = weight;
                    }

                }

            }

        }

        // 6. apply common translation using the heighest weight
        point_x = point_x + (max_weight*trans[0]);
        point_y = point_y + (max_weight*trans[1]);
        point_z = point_z + (max_weight*trans[2]); 

        // 7. asign new values to cloud point
        cloud_device[point_position].x = point_x;
        cloud_device[point_position].y = point_y;
        cloud_device[point_position].z = point_z;
   
    }

}



extern "C" bool deformCloud(pcl::gpu::DeviceArray<pcl::PointXYZRGB>& cloud_device, double *rot, double *trans, double *err_points, double *t_uniq, float *radius, int err_p_size)
{
    double *device_rot;
    double *device_trans;
    double *device_err_points;
    double *device_t_uniq;
    float *device_radius;
    int *device_err_p_size;

    // getting ointers from cuda device to store data
    cudaMalloc((void **) &device_rot, 9*sizeof(double));
    cudaMalloc((void **) &device_trans, 3*sizeof(double));
    cudaMalloc((void **) &device_err_points, 3*err_p_size*sizeof(double));
    cudaMalloc((void **) &device_t_uniq, 3*err_p_size*sizeof(double));
    cudaMalloc((void **) &device_radius, sizeof(float));
    cudaMalloc((void **) &device_err_p_size, sizeof(int));

    // store data in cuda device
    cudaMemcpy(device_rot, rot, 9*sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(device_trans, trans, 3*sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(device_err_points, err_points, 3*err_p_size*sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(device_t_uniq, t_uniq, 3*err_p_size*sizeof(double), cudaMemcpyHostToDevice);
    cudaMemcpy(device_radius, radius, sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(device_err_p_size, &err_p_size, sizeof(int), cudaMemcpyHostToDevice);

    // define grid and thread block sizes
    dim3 dimGrid(600);
    dim3 dimBlock(512);

    deform_points<<<dimGrid,dimBlock>>>(cloud_device, device_rot, device_trans, device_err_points, device_t_uniq, device_radius, device_err_p_size);
    
    // check for errors
    cudaError_t err = cudaGetLastError();
    if (cudaSuccess != err) 
    {
        fprintf(stderr, "CUDA error: %s.\n", cudaGetErrorString( err) );
        exit(EXIT_FAILURE);
    }  

    // wait until GPU kernel is done
    cudaDeviceSynchronize();

    // free device memory
    cudaFree(device_rot);
    cudaFree(device_trans);
    cudaFree(device_err_points);
    cudaFree(device_t_uniq);
    cudaFree(device_radius);
    cudaFree(device_err_p_size);

    return true;
}