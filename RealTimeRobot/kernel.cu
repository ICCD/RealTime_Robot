#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include <stdio.h>

#define CUDA_NUM_THREADS 512
#define CUDA_MAX_NUM_BLOCKS 72

__global__
void ComputeTDF(const int * voxel_grid_occ, float * voxel_grid_TDF, int voxel_grid_dim, int num_occ) {


	int voxel_idx = blockIdx.x * blockDim.x + threadIdx.x;
	if (voxel_idx > (voxel_grid_dim* voxel_grid_dim * voxel_grid_dim))
		return;

	int pt_grid_z = voxel_idx / (voxel_grid_dim * voxel_grid_dim);
	int pt_grid_y = (voxel_idx - (pt_grid_z * voxel_grid_dim * voxel_grid_dim)) / voxel_grid_dim;
	int pt_grid_x = voxel_idx - (pt_grid_z * voxel_grid_dim * voxel_grid_dim) - (pt_grid_y * voxel_grid_dim);

	float _distance = 900;
	for (int i = 0; i < num_occ; i++)
	{
		int x = pt_grid_x - voxel_grid_occ[i * 3 + 0];
		int y = pt_grid_y - voxel_grid_occ[i * 3 + 1];
		int z = pt_grid_z - voxel_grid_occ[i * 3 + 2];
		float temp = (x*x + y*y + z*z);
		if (temp < _distance)
			_distance = temp;
	}
	voxel_grid_TDF[voxel_idx] = _distance;
}

// Helper function for using CUDA to add vectors in parallel.
extern "C"
cudaError_t  ComputeTDFWithCuda(const int * voxel_grid_occ, float * voxel_grid_TDF, int voxel_grid_dim, int num_occ)
{

	int * gpu_voxel_grid_occ = 0;
	float * gpu_voxel_grid_TDF = 0;
	cudaError_t cudaStatus;

	// Choose which GPU to run on, change this on a multi-GPU system.
	cudaStatus = cudaSetDevice(0);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "111 cudaSetDevice failed!  Do you have a CUDA-capable GPU installed? ");
		goto Error;
	}

	// Allocate GPU buffers for three vectors (two input, one output)    .
	cudaStatus = cudaMalloc((void**)&gpu_voxel_grid_occ, num_occ * 3 * sizeof(int));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "222 gpu_voxel_grid_occ cudaMalloc failed!");
		goto Error;
	}

	cudaStatus = cudaMalloc((void**)&gpu_voxel_grid_TDF, 27000 * sizeof(float));
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "333 gpu_voxel_grid_TDF cudaMalloc failed!");
		goto Error;
	}

	// Copy input vectors from host memory to GPU buffers.
	cudaStatus = cudaMemcpy(gpu_voxel_grid_occ, voxel_grid_occ, num_occ * 3 * sizeof(int), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "444 gpu_voxel_grid_occ cudaMemcpy failed!");
		goto Error;
	}

	cudaStatus = cudaMemcpy(gpu_voxel_grid_TDF, voxel_grid_TDF, 27000 * sizeof(float), cudaMemcpyHostToDevice);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "555 gpu_voxel_grid_TDF cudaMemcpy failed!");
		goto Error;
	}

	// Launch a kernel on the GPU with one thread for each element.
	ComputeTDF << <CUDA_MAX_NUM_BLOCKS, CUDA_NUM_THREADS >> >(gpu_voxel_grid_occ, gpu_voxel_grid_TDF, voxel_grid_dim, num_occ);

	// Check for any errors launching the kernel
	cudaStatus = cudaGetLastError();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "666 ComputeTDF launch failed: %s\n", cudaGetErrorString(cudaStatus));
		goto Error;
	}

	// cudaDeviceSynchronize waits for the kernel to finish, and returns
	// any errors encountered during the launch.
	cudaStatus = cudaDeviceSynchronize();
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "777 cudaDeviceSynchronize returned error code %d after launching addKernel!\n", cudaStatus);
		goto Error;
	}

	// Copy output vector from GPU buffer to host memory.
	cudaStatus = cudaMemcpy(voxel_grid_TDF, gpu_voxel_grid_TDF, 27000 * sizeof(float), cudaMemcpyDeviceToHost);
	if (cudaStatus != cudaSuccess) {
		fprintf(stderr, "888 voxel_grid_TDF, gpu_voxel_grid_occ cudaMemcpy failed!");
		goto Error;
	}

Error:
	cudaFree(gpu_voxel_grid_occ);
	cudaFree(gpu_voxel_grid_TDF);


	return cudaStatus;
}
