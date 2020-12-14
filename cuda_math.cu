#include <iostream>
#include <cuda.h>
#include <cuda_device_runtime_api.h>
#include <cuda_runtime_api.h>




void CudaCheckDevice()
{
    cudaDeviceProp deviceProp;
    int *deviceCount = new int (-1);
    cudaGetDeviceCount(deviceCount);
    std::cout << "devices = " << *deviceCount << std::endl;
    for (int i = 0; i < *deviceCount; i++)
    {
      //Получаем информацию об устройстве
      cudaGetDeviceProperties(&deviceProp, i);

      //Выводим иформацию об устройстве
      printf("Device name: %s\n", deviceProp.name);
      printf("Total global memory: %ld\n", deviceProp.totalGlobalMem);
      printf("Shared memory per block: %d\n", deviceProp.sharedMemPerBlock);
      printf("Registers per block: %d\n", deviceProp.regsPerBlock);
      printf("Warp size: %d\n", deviceProp.warpSize);
      printf("Memory pitch: %d\n", deviceProp.memPitch);
      printf("Max threads per block: %d\n", deviceProp.maxThreadsPerBlock);

      printf("Max threads dimensions: x = %d, y = %d, z = %d\n",
        deviceProp.maxThreadsDim[0],
        deviceProp.maxThreadsDim[1],
        deviceProp.maxThreadsDim[2]);

      printf("Max grid size: x = %d, y = %d, z = %d\n",
        deviceProp.maxGridSize[0],
        deviceProp.maxGridSize[1],
        deviceProp.maxGridSize[2]);

      printf("Clock rate: %d\n", deviceProp.clockRate);
      printf("Total constant memory: %d\n", deviceProp.totalConstMem);
      printf("Compute capability: %d.%d\n", deviceProp.major, deviceProp.minor);
      printf("Texture alignment: %d\n", deviceProp.textureAlignment);
      printf("Device overlap: %d\n", deviceProp.deviceOverlap);
      printf("Multiprocessor count: %d\n", deviceProp.multiProcessorCount);

      printf("Kernel execution timeout enabled: %s\n",
        deviceProp.kernelExecTimeoutEnabled ? "true" : "false");
    }
}


//need to create arrays U, J,
void CudaMatrixMultiply4x4(double *A, double *B, double *C)
{

}
