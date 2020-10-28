#include "my_math.h"

matrix matrix::multuply(const matrix &matrix1, const matrix &matrix2)
{
    matrix result(matrix1.rows, matrix2.cols);
    scalar buff = 0;
    buff = 0;
    for (int i = 0; i < matrix1.rows; i++)
    {
        for (int j = 0 ; j < matrix2.cols; j++)
        {
            for (int k = 0; k < matrix1.cols; k++)
            {
                buff += matrix1.elem(i, k) * matrix2.elem(k, j);
            }
            result.elem(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

matrix matrix::operator *(const matrix &matrix2)
{
    matrix result(this->rows, matrix2.cols);
    scalar buff = 0;
    for (int i = 0; i < this->rows; i++)
    {
        for (int j = 0 ; j < matrix2.cols; j++)
        {
            for (int k = 0; k < this->cols; k++)
            {
                auto l = this->elem(i, k);
                auto l1 = matrix2.elem(k,j);
                buff += this->elem(i, k) * matrix2.elem(k, j);
            }
            result.elem(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

matrix matrix::operator =(const matrix &matrix1)
{
    if (data != nullptr)
    {
        delete [] data;
    }
    rows = matrix1.rows;
    cols = matrix1.cols;
    dataNewMemory();
    memcpy(data, matrix1.data, sizeof(scalar) * rows * cols);

    return *this;
}
