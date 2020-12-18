#ifndef CUDA_MATH_H
#define CUDA_MATH_H
#include <iostream>

void CudaMatrixMultiplication(double *A, double *B, double *C);
void CudaCheckKoefficients(int *counter, int *size);
void CudaCheckDevice();
float CudaCalcucaleHikm(scalar *T, scalar *U, scalar *J, scalar *speed, int i);
//void CudaCheckKS(int *threadX, int *threadY);

#endif // CUDA_MATH_H
