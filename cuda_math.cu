#include <iostream>
#include <cuda.h>
#include <cuda_device_runtime_api.h>
#include <cuda_runtime_api.h>
#include <device_launch_parameters.h>
#include <my_math.h>
//#include <my_math.h>


#define BLOCK_SIZE 16
#define SIZE 4
typedef double scalar;

//static const scalar Q[16]={0, -1, 0, 0, 1, 0, 0, 0,
//                    0,  0, 0, 0, 0, 0, 0, 0};
//static const scalar Q_sq[16] = {1, 0, 0, 0, 0, -1, 0, 0,
//                         0,  0, 0, 0, 0, 0, 0, 0};

__device__ __constant__ scalar hikm_buff[6][6][6];
__device__ __constant__ scalar Q_sq[4 * 4] = {-1,  0, 0, 0,
                                   0, -1, 0, 0,
                                   0,  0, 0, 0,
                                   0,  0, 0, 0};
__device__ __constant__ scalar Q[4 * 4] =     {0, -1, 0, 0,
                                    1,  0, 0, 0,
                                    0,  0, 0, 0,
                                    0,  0, 0, 0};
__device__ __constant__ scalar J_c[6 * 4 * 4];
__device__ __constant__ scalar U_c[6 * 6 * 4 * 4];
__device__ __constant__ scalar T_c[6 * 4 * 4];
__device__ __constant__ scalar speed_c[6];
void CudaMatrixCout(scalar *m, char *ch, int size)
{
    cout << endl;
    cout << ch << endl;
    for (int i = 0 ; i < size; i++)
    {
        cout << setw(12) << *(m + i) ;
        if ((i + 1) % int(sqrt(size)) == 0)
            cout << endl;
    }
    cout << endl;
}
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

//__global__ void CalculateHikm()
//{
//    __shared__ float s_t[6][4][4];
//    __shared__ float s_j[6][4][4];
//    __shared__ float s_s[6];
//    __shared__ float s_r[6];
//    int k, m, j;
//    k = blockIdx.x;
//    m = blockIdx.y;
//    j = blockIdx.z;
//    for (int ctr = 0; ctr < j; ctr ++)
//    {
////        if (ctr == (k - 1) && ctr == (m - 1))
////            CudaMatrixMultiply(s_u_b, Q_sq, s_t);
////        else if (ctr == (k - 1) || ctr == (m - 1))
////            CudaMatrixMultiply(s_u_b, Q, s_t);
////        else
////            CudaMatrixMultiply(s_u_b, s_t, s_t);

//    }
//}

__device__ void CudaMatrixMultiply(scalar *A, scalar *B, scalar *C, scalar *res)
{
// For 3 matrix multiplication
    const int n = SIZE;
    __shared__ double a_s[n*n],
                      b_s[n*n],
                      c_s[n*n],
                      temp_s[n*n],
                      res_s[n*n];
    a_s[threadIdx.y*n + threadIdx.x] = A[threadIdx.y*n + threadIdx.x];
    b_s[threadIdx.y*n + threadIdx.x] = B[threadIdx.y*n + threadIdx.x];
    c_s[threadIdx.y*n + threadIdx.x] = C[threadIdx.y*n + threadIdx.x];

    int row = threadIdx.y;//threadIdx.x / n;
    int col = threadIdx.x;//threadIdx.x % n;
//    C[row*n + col] = A[row*n] * B[col] + A[row*n + 1]*B[col + 1*n] + A[row*n + 2] * B[col + 2*n] + A[row*n + 3] * B[col + 3*n];

    temp_s[row*n + col] = a_s[row*n] * b_s[col] + a_s[row*n + 1] * b_s[col + 1*n] + a_s[row*n + 2] * b_s[col + 2*n] + a_s[row*n + 3] * b_s[col + 3*n];
    res_s[row*n + col] = temp_s[row*n] * c_s[col] + temp_s[row*n + 1] * c_s[col + 1*n] + temp_s[row*n + 2] * c_s[col + 2*n] + temp_s[row*n + 3] * c_s[col + 3*n];
    res[threadIdx.y*n + threadIdx.x] = res_s[threadIdx.y*n + threadIdx.x];
}
__device__ void CudaMatrixMultiply(scalar *A, scalar *B, scalar *res)
{
//For 2 matrix multiplication
    const int n = SIZE;
    __shared__ double a_s[n*n],
                      b_s[n*n],
                      res_s[n*n];
    a_s[threadIdx.y*n + threadIdx.x] = A[threadIdx.y*n + threadIdx.x];
    b_s[threadIdx.y*n + threadIdx.x] = B[threadIdx.y*n + threadIdx.x];

    int row = threadIdx.y;//threadIdx.x / n;
    int col = threadIdx.x;//threadIdx.x % n;
//    C[row*n + col] = A[row*n] * B[col] + A[row*n + 1]*B[col + 1*n] + A[row*n + 2] * B[col + 2*n] + A[row*n + 3] * B[col + 3*n];

    res_s[row*n + col] = a_s[row*n] * b_s[col] + a_s[row*n + 1] * b_s[col + 1*n] + a_s[row*n + 2] * b_s[col + 2*n] + a_s[row*n + 3] * b_s[col + 3*n];
    res[threadIdx.y*n + threadIdx.x] = res_s[threadIdx.y*n + threadIdx.x];
}
__global__ void CalcUBuff(scalar *T, scalar *U, scalar *UBuff)
{
//    scalar Q[16]={0, -1, 0, 0, 1, 0, 0, 0,
//                  0,  0, 0, 0, 0, 0, 0, 0};
//    scalar Q_sq[16] = {1, 0, 0, 0, 0, -1, 0, 0,
//                       0,  0, 0, 0, 0, 0, 0, 0};
//    const int n = SIZE;
//    __shared__ scalar *s_u_b;
//    s_u_b = new scalar [4*4];//[4][4];
//    __shared__ scalar *s_t;
//    s_t = new scalar [6*4*4];
//    //maybe use constant memory
//    __shared__ scalar hikm[6][6];

////    __shared__ scalar s_t[6][n][n];
////    s_t[blockIdx.x][threadIdx.x][threaxIdx.y] = T[blockIdx.x * 16 + threadIdx.y*n + threadIdx.x];
////    __syncthreads();

//    scalar *p = Q_sq;
//    if ((threadIdx.x == blockIdx.x - 1) && (threadIdx.x == blockIdx.y - 1))
//        CudaMatrixMultiply(s_u_b, Q_sq, s_t);
//    else if ((threadIdx.x == blockIdx.x - 1) || (threadIdx.x == blockIdx.y - 1))
//        CudaMatrixMultiply(s_u_b, Q, s_t);
//    else
//        CudaMatrixMultiply(s_u_b, s_t, s_t);
}
//__device__ MatrixMultiplyForHjkm(int *first_index, int *second_index, scalar *T, scalar *result)
//{

//}
__device__ void MakeIdentity(scalar *matrix, int size)
{
    matrix[threadIdx.y * size + threadIdx.x] = 0;
    if (threadIdx.x == threadIdx.y)
        matrix[threadIdx.y * size + threadIdx.x] = 1;
}
__device__ void calc_hikm_buff(scalar *u_buff, scalar *uij, scalar *ji, scalar *speed, scalar *result)
{                           //u_buff 4x4     Ji[6x4x4]  Uji[6x6x4x4]    speed[6]
    const int n = SIZE;
    __shared__ scalar s_u_b[n*n],
                      s_j_i[n*n],
                      s_u_ij[n*n],
                      s_temp[n*n],
                      s_temp1[n*n],
                      s_s[6];

    int block = blockIdx.z;
    int row = threadIdx.y;
    int col = threadIdx.x;
    s_u_b[threadIdx.y*n + threadIdx.x] = u_buff[threadIdx.y*n + threadIdx.x];
    s_j_i[threadIdx.y*n + threadIdx.x] = ji[threadIdx.y*n + threadIdx.x];
    s_u_ij[threadIdx.y*n + threadIdx.x] = uij[threadIdx.y*n + threadIdx.x];
    s_s[blockIdx.z] = speed[blockIdx.z];
//    C[row*n + col] = A[row*n] * B[col] + A[row*n + 1]*B[col + 1*n] + A[row*n + 2] * B[col + 2*n] + A[row*n + 3] * B[col + 3*n];
//    res_s[row*n + col] = a_s[row*n] * b_s[col] + a_s[row*n + 1] * b_s[col + 1*n] + a_s[row*n + 2] * b_s[col + 2*n] + a_s[row*n + 3] * b_s[col + 3*n];
    //Умножение Ji на U транспонированное
    s_temp[row*n + col] = s_j_i[row*n] * s_u_ij[row*n + 0] + s_j_i[row*n + 1] * s_u_ij[row*n + 1] + s_j_i[row*n + 2] * s_u_ij[row*n + 2] + s_j_i[row*n + 3] * s_u_ij[row*n + 3];
    //Умножение  u_buff на результат
    s_temp1[row*n + col] =  s_u_ij[row*n] * s_temp[col] + s_u_ij[row*n + 1] * s_temp[col + 1*n] + s_u_ij[row*n + 2] * s_temp[col + 2*n] +  s_u_ij[row*n + 3] * s_temp[col + 3*n];
    if (threadIdx.x == 0 && threadIdx.y == 0)
    {
        *result = (s_temp1[0] + s_temp1[5] + s_temp1[10] + s_temp1[15]) * s_s[blockIdx.x] * s_s[blockIdx.y];
    }
}
//__global__ void CalculateHikmTestForOneBlock(scalar *T, scalar *Uij, scalar *J, scalar *speed, int i, scalar *hi, scalar *res_matrix)
__global__ void CalcucaleHjkm(scalar *T, scalar *Uij, scalar *J, scalar *speed, int i, scalar *hi, scalar *res_matrix)
{
//    __constant__ scalar c_t[6*4*4];
//    __constant__ scalar c_u[6*6*6];
//    __shared__ scalar u_buff[4][4];
    *hi = 123.123;
    __shared__ scalar *u_buff;
    __shared__ scalar temp;
    u_buff = new scalar[4*4];
    int index_x = blockIdx.x;
    int index_y = blockIdx.y;
    int index_z = blockIdx.z;

    int index_b_z_16 = blockIdx.z * 16;
    if (threadIdx.x == 0 && threadIdx.y == 0)
        hikm_buff[blockIdx.x][blockIdx.y][blockIdx.z] = 0;
    if (blockIdx.z >= i)
    {
        MakeIdentity(u_buff, 4);
        for (int ctr = 0; ctr < index_z; ctr++)
        {
            if ((ctr == index_x - 1) && (ctr == index_y - 1))
                CudaMatrixMultiply(u_buff, Q_sq, (T + index_b_z_16), u_buff);
            else if ((ctr == index_x - 1) || (ctr == index_y - 1))
                CudaMatrixMultiply(u_buff, Q, (T + index_b_z_16), u_buff);
            else
                CudaMatrixMultiply(u_buff, (T + index_b_z_16), u_buff);
            __syncthreads();
        }
        if (blockIdx.x == 0 && blockIdx.y == 0 && blockIdx.z == 0)
            res_matrix[threadIdx.y * 4 + threadIdx.x] = u_buff[threadIdx.y * 4 + threadIdx.x];
        __syncthreads();
        calc_hikm_buff(u_buff, (Uij + blockIdx.z * 6 * 4 * 4 + i * 4 * 4), (J + i * 16), speed, &temp);
        if (threadIdx.x == 0 && threadIdx.y == 0)
            hikm_buff[blockIdx.x][blockIdx.y][blockIdx.z] = temp;
        __syncthreads();
        //Массив выше сформирован по j(blockIdx). Теперь надо просуммировать по j и получить квадратную матрицу для m и k
//        if (blockIdx.z == 0 && threadIdx.x == 0) {

//            hi = hikm_buff[blockIdx.x][blockIdx.y][blockIdx.z]);

//        }
    }
    __syncthreads();

    if (threadIdx.y == 0 && threadIdx.x == 0 && blockIdx.z == 0)
    {
        int buff(0);
        for (int i = 0; i < 6; i++)
            buff += hikm_buff[blockIdx.x][blockIdx.y][i];
        hikm_buff[blockIdx.x][blockIdx.y][0] = buff;
    }
    __syncthreads();
    //Получили матрицу результатов размером 6x6. Теперь надо просуммировать по blockIdx.y
    if (threadIdx.y == 0 && threadIdx.x == 0 && blockIdx.z == 0 && blockIdx.y == 0)
    {
        int buff(0);
        for (int i = 0; i < 6; i++)
        {
            buff += hikm_buff[blockIdx.x][i][0];
        }
        hikm_buff[blockIdx.x][0][0] = buff;
    }
    __syncthreads();
    if (blockIdx.x == 0 && threadIdx.x == 0 && blockIdx.x == 0 && blockIdx.y == 0 && blockIdx.z == 0)
    {
//        for (int i = 0; i < 6; i++)

//            *hi += hikm_buff[i][0][0];

    }
}

__global__ void SetHi(scalar *hi)
{
    *hi = 123.99;
}
float CudaCalcucaleHikm(scalar *T, scalar *U, scalar *J, scalar *speed, int i)
{

    scalar *gege = new scalar (8.22);
    __global__ scalar *hi; //scalar allocated in device memory
    cudaMalloc((void **) &hi, sizeof(scalar));//memory allocation in device memory

    __global__ scalar u_buff[16];
//    cudaMalloc((void **) g_t, sizeof(float) * 4 * 4 * 6);
//    cudaMalloc((void **) g_j, sizeof(float) * 4 * 4 * 6);
//    cudaMalloc((void **) g_s, sizeof(float) * 6);
//    cudaMalloc((void **) g_u, sizeof(float) * 4 * 4 * 6);
//    cudaMalloc((void **) u_buf, sizeof(float) * 4 * 4);
//    cudaMemcpy(g_t, T, sizeof(scalar) * 4 * 4 * 6, cudaMemcpyHostToDevice);
//    cudaMemcpy(g_j, J, sizeof(scalar) * 4 * 4 * 6, cudaMemcpyHostToDevice);
//    cudaMemcpy(g_s, speed, sizeof(scalar) * 6, cudaMemcpyHostToDevice);
    dim3 blocks = dim3(6, 6, 6);
    dim3 threads = dim3(4, 4);
//    __constant__ scalar *J_c;
//    __constant__ scalar *U_c;
//    __constant__ scalar *T_c;
//    __constant__ scalar *speed_c;
//    cudaMemcpyToSymbol(J_c, J, sizeof(scalar) * 6 * 4 * 4, cudaMemcpyHostToDevice);
//    cudaMalloc((void **) &J_c,     sizeof(scalar) * 6 * 4 * 4);
//    cudaMalloc((void **) &U_c,     sizeof(scalar) * 6 * 6 * 4 * 4);
//    cudaMalloc((void **) &T_c,     sizeof(scalar) * 6 * 4 * 4);
//    cudaMalloc((void **) &speed_c, sizeof(scalar) * 6);
//    cudaMalloc((void **) &hi_g, sizeof(scalar));
    cudaMemcpyToSymbol("J_c",     J,     sizeof(scalar) * 6 * 4 * 4,     0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol("U_c",     U,     sizeof(scalar) * 6 * 6 * 4 * 4, 0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol("T_c",     T,     sizeof(scalar) * 6 * 4 * 4,     0, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol("speed_c", speed, sizeof(scalar) * 6,             0, cudaMemcpyHostToDevice);
//    cudaMemcpy(hi_c,      hi,    sizeof(scalar), cudaMemcpyHostToDevice);

//    cudaMemcpy(J, J_c, sizeof(scalar) * 6 * 4 * 4,      cudaMemcpyDeviceToHost);
//    cudaMemcpy(T, T_c, sizeof(scalar) * 6 * 4 * 4,      cudaMemcpyDeviceToHost);

//    CudaMatrixCout(J, "J_c matrix", 6*4*4);
//    CudaMatrixCout(T, "T matrix",   6*4*4);
//    cout << " __________________ " << endl;

    scalar * res_matr = new scalar [16];
    SetHi<<<dim3(1), dim3(1)>>>(hi);
    cudaMemcpy(gege, hi, sizeof(scalar), cudaMemcpyDeviceToHost);//copy
    cout << "h_i from cuda = \ngege = " << *gege << " hi = " << hi << endl;;
    CalcucaleHjkm<<<blocks, threads>>>(T_c, U_c, J_c, speed_c, i, hi, res_matr);
//    CudaMatrixCout(res_matr, "result matrix hikm", 16);
//    CudaMatrixMultiply<<<dim3(1), dim3(4,4)>>>();
    cudaMemcpy(gege, hi, sizeof(scalar), cudaMemcpyDeviceToHost);//copy

    cout << "h_i from cuda = \ngege = " << *gege << " hi = " << hi << endl;;
//    return hi;
}


//need to create arrays U, J,
__global__ void CudaMatrixMultiply4x4(double *A, double *B, double *C)
{
    const int n = SIZE;
    __shared__ double a_s[n][n],
                      b_s[n][n],
                      c_s[n][n];
    a_s[threadIdx.x][threadIdx.y] = A[threadIdx.y*n + threadIdx.x];
    b_s[threadIdx.x][threadIdx.y] = B[threadIdx.y*n + threadIdx.x];

    int row = threadIdx.y;//threadIdx.x / n;
    int col = threadIdx.x;//threadIdx.x % n;
//    C[row*n + col] = A[row*n] * B[col] + A[row*n + 1]*B[col + 1*n] + A[row*n + 2] * B[col + 2*n] + A[row*n + 3] * B[col + 3*n];

    C[row*n + col] = a_s[row][0] * b_s[0][row] + a_s[row][1] * b_s[1][row] + a_s[row][2] * b_s[2][row] + a_s[row][3] * b_s[3][row];
}

__global__ void CudaCheckKF(int *threadX, int *threadY)
{
    threadX[threadIdx.x] = 2;
    threadY[threadIdx.y] = 1;
}
void CudaCheckKoefficients(int *counter, int *size)
{
    using namespace std;
    int *ctr_x, *ctr_y;
    cudaMalloc((void**) &ctr_x, sizeof(int) * *size);
    cudaMalloc((void**) &ctr_y, sizeof(int) * *size);
    dim3 threads = dim3(SIZE,SIZE);
    dim3 blocks = dim3(1);
    CudaCheckKF<<<blocks, threads>>>(ctr_x, ctr_y);
    cudaMemcpy(counter, ctr_x, sizeof(int) * *size, cudaMemcpyDeviceToHost);
    for (int i = 0; i < *size;i++)
    {
        cout << "i = " << i;
        cout << " th_x cout " << *(counter + i)  << endl;
    }
    cudaMemcpy(counter, ctr_y, sizeof(int) * 20, cudaMemcpyDeviceToHost);
    for (int i = 0; i < *size;i++)
    {
        cout << "i = " << i;
        cout << " th_y cout " << *(counter + i)  << endl;
    }
}
__global__ void CudaMatrixMultiplyGlobal(scalar *A, scalar *B, scalar *res)
{
//For 2 matrix multiplication
    const int n = SIZE;
    __shared__ double a_s[n*n],
                      b_s[n*n],
                      res_s[n*n];
    a_s[threadIdx.y*n + threadIdx.x] = A[threadIdx.y*n + threadIdx.x];
    b_s[threadIdx.y*n + threadIdx.x] = B[threadIdx.y*n + threadIdx.x];

    int row = threadIdx.y;//threadIdx.x / n;
    int col = threadIdx.x;//threadIdx.x % n;
//    C[row*n + col] = A[row*n] * B[col] + A[row*n + 1]*B[col + 1*n] + A[row*n + 2] * B[col + 2*n] + A[row*n + 3] * B[col + 3*n];

    res_s[row*n + col] = a_s[row*n] * b_s[col] + a_s[row*n + 1] * b_s[col + 1*n] + a_s[row*n + 2] * b_s[col + 2*n] + a_s[row*n + 3] * b_s[col + 3*n];
    res[threadIdx.y*n + threadIdx.x] = res_s[threadIdx.y*n + threadIdx.x];
}
void CudaMatrixMultiplication(double *A, double *B, double *C)
{

    using namespace std;
    std::cout << "Cuda started" << std::endl;
    cudaEvent_t start, stop;

    double  *h_a = 0,
            *h_b = 0,
            *h_c = 0;
    cudaMalloc((void **) &h_a, sizeof(double)*16);
    cudaMalloc((void **) &h_b, sizeof(double)*16);
    cudaMalloc((void **) &h_c, sizeof(double)*16);
    cudaMemcpy(h_a, A, sizeof(double)*16, cudaMemcpyHostToDevice);
    cudaMemcpy(h_b, B, sizeof(double)*16, cudaMemcpyHostToDevice);


    dim3 threads = dim3(4,4);
    dim3 blocks = dim3(1);

    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start);

    CudaMatrixMultiplyGlobal<<<blocks, threads>>>(h_a, h_b, h_c);
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaMemcpy(C, h_c, sizeof(double)*BLOCK_SIZE, cudaMemcpyDeviceToHost);

//    for (int i = 0; i < 16; i++)
//    {
//        std::cout << *(C + i);
//        if ((i + 1) % 4 == 0)
//            cout << endl;
//    }
//    for (int i = 0; i < BLOCK_SIZE; i++)
//        cout << " matrix C " << *(C + i) << endl;
    cudaFree(h_a);
    cudaFree(h_b);
    cudaFree(h_c);
    float milliseconds(0.0);
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "Cuda ended" << std::endl;
    std::cout << "Cuda time in milliseconds " << milliseconds << std::endl;
//    cout << "CUDA Duration " << tim.duration().count() << endl;
}
