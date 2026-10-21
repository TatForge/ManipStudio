#ifndef CUDA_MATH_H
#define CUDA_MATH_H
#include <iostream>
using namespace std;
void CudaMatrixMultiplication(double *A, double *B, double *C);
void CudaCheckKoefficients(int *counter, int *size);
void CudaCheckDevice();
void CudaCalcucaleHikm(scalar *T, scalar *U, scalar *J, scalar *speed, int i);



#endif // CUDA_MATH_H
