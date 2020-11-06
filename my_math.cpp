#include "my_math.h"

using namespace RoboMath;

scalar &Matrix::elem(const int &row, const int &col) const
{
    assert(col >= 0 && col <= cols);
    assert(row >= 0 && row <= rows);
    return *(data + index(row, col));
}

Matrix Matrix::Multuply(const Matrix &matrix1, const Matrix &matrix2)
{
    Matrix result(matrix1.rows, matrix2.cols);
    scalar buff = 0;
    buff = 0;
    for (counter i = 0; i < matrix1.rows; i++)
    {
        for (counter j = 0 ; j < matrix2.cols; j++)
        {
            for (counter k = 0; k < matrix1.cols; k++)
            {
                buff += matrix1.elem(i, k) * matrix2.elem(k, j);
            }
            result.elem(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

void Matrix::AddDiagonal(const scalar &d)
{
    int len = min(rows, cols);
    scalar *scalarP(data);
    for (counter i  = 0; i < len; i++)
    {
        *scalarP += d;
        scalarP += cols + 1;
    }
}

void Matrix::SetSubmatrix(const Matrix &source)
{
    assert(source.rows <= rows && source.cols <= cols);
    int step = cols - source.cols;
    scalar *from = source.GetDataPtr();
    scalar *to = data;
    for (counter i = 0; i < source.rows; i++)
    {
        for (counter j = 0; j < source.cols; j++)
        {
            *to = *from;
            ++to;
            ++from;
        }
        to += step;
    }
    for (counter i =0; i < rows*cols; i++)
    {
        cout << " data " << *(this->data + i) << endl;
    }
}

void Matrix::SetColumn(Vector &v, int ColNumber)
{
    assert(ColNumber <= cols);
    assert(v.GetLength() == rows);
    scalar *to = data + ColNumber - 1;
    scalar *from = v.GetDataPtr();
    for (counter i = 0; i < v.GetLength(); i++)
    {
        *to= *from;
        to += this->cols;
        ++from;
    }
}

void Matrix::SolveGauss()
{
    int num_iter = min(cols, rows); // почему берется min, а не rows, ведь проверка по столбцам??
    // num_iter = rows;
    //First of all find maximum element in first col
    scalar *main_elem_ptr = data;
    scalar *max_row_ptr = nullptr;
    scalar *to = nullptr;
    scalar *from = nullptr;
    double alpha = 0.0;
    int row_length = cols;

    int diagonal_step = cols + 1;
    for (counter i = 0; i < (num_iter); i++)
    {

        scalar *row_for_swap = main_elem_ptr;
        scalar max_abs = fabs(*main_elem_ptr);
        max_row_ptr = main_elem_ptr;
        //Find row with maximum (abs) element value
        for (counter j = 1; j < num_iter; j++)
        {
            if (*(data + cols * j) > max_abs)
            {
                max_abs = *(data + cols * j);
                max_row_ptr = data + cols * j;
            }
            else if ( - *(data + cols * j) > max_abs)
            {
                max_abs = -*(data + cols * j);
                max_row_ptr = data + cols * j;
            }
        }
        //swap elements
        if (max_row_ptr != main_elem_ptr)
        {
            scalar *temp_ptr = main_elem_ptr;
            for (counter i = 0; i < cols; i++)
            {
                scalar temp = *temp_ptr;
                *temp_ptr = *max_row_ptr;
                *max_row_ptr  = temp;
                temp_ptr ++;
                max_row_ptr  ++;
            }
        }

        //subtrack
        row_for_swap = main_elem_ptr;
        for (counter i = 0; i < (num_iter - 1); i++)
        {
            row_for_swap += cols;
            to = row_for_swap;
            from = main_elem_ptr;
            alpha = (*to)/(*from);
            *to = 0.0;
            for(counter j = 0; j < (row_length - 1); j++)
            {
                ++to;
                ++from;
                *to -= (*from) * alpha;
            }
        }
        main_elem_ptr += diagonal_step;
        this->MatrixCout();
        --row_length;
        --num_iter;
    }
}

void Matrix::GetTheRoots(Vector &V)
{
    assert(V.GetLength() == rows);

    scalar *row_elem_ptr = data + cols * rows - 2;
    scalar *right_val_ptr = data + cols * rows - 1;
    scalar *result_ptr_last = V.GetDataPtr() + V.GetLength() - 1;
    scalar accum;

    for (counter i = rows; i > 0; i--)
    {
        accum = *(right_val_ptr);
        right_val_ptr -= cols;
        scalar *row_ptr = row_elem_ptr;
        scalar *result_ptr = result_ptr_last;
        for(counter j = 0; j < rows - i; j++)
        {
            accum -= (*row_ptr) * (*result_ptr);
            --row_ptr;
            --result_ptr;
        }
        *result_ptr = accum / *row_ptr;
        row_elem_ptr -= cols;
    }
}

Matrix Matrix::operator *(const Matrix &matrix2)
{
    Matrix result(rows, matrix2.cols);
    scalar buff = 0;
    for (counter i = 0; i < this->rows; i++)
    {
        for (counter j = 0 ; j < matrix2.cols; j++)
        {
            for (counter k = 0; k < this->cols; k++)
            {
                buff += this->elem(i, k) * matrix2.elem(k, j);
            }
            result.elem(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

Vector Matrix::operator *(const Vector &vector2)
{
    const scalar *vector_data_ptr;
    Vector result(this->GetRowsCount());
    scalar *result_ptr = result.GetDataPtr();
    scalar *jacoby_data_ptr = data;
    for (short i = 0; i < this->GetRowsCount(); i++)
    {
        vector_data_ptr = vector2.GetDataPtr();
        *result_ptr = 0;
        for (counter i = 0; i < vector2.GetLength(); i++)
        {
            *result_ptr += (*jacoby_data_ptr) * (*vector_data_ptr);
            ++jacoby_data_ptr;
            ++vector_data_ptr;
        }
        ++result_ptr;
    }
    return result;
}

Matrix Matrix::operator =(const Matrix &matrix1)
{
    if (data != nullptr)
    {
        delete [] data;
    }
    rows = matrix1.rows;
    cols = matrix1.cols;
    DataNewMemory();
    memcpy(data, matrix1.data, sizeof(scalar) * rows * cols);

    return *this;
}

void Matrix::transpose()
{
    assert(this->data != nullptr);
    assert(this->cols > 1 && this->rows > 1);
    scalar *tdata = new scalar [rows*cols];

    for (counter i = 0; i < rows; i++)
        for (counter j = 0; j < cols; j++)
        {
#ifdef DEBUG_COUT
            cout << " data " << *(data + i*rows + j) << endl;
#endif
            *(tdata + j*rows + i) = *(data + i*cols + j);
        }
#ifdef DEBUG_COUT
    for(counter i = 0; i < cols * rows; i++)
    {
        cout << *(tdata + i) << endl;
    }
#endif
    int trows = cols;
    cols = rows;
    rows = trows;
    memcpy(data, tdata, sizeof(scalar) * rows * cols);
}

void Matrix::MatrixCout()
{
    for (counter i = 0; i < rows; i++)
    {
        for (counter j = 0; j < cols; j++)
        {
            cout << *(data + i * cols + j) << "\t";
        }
        cout << endl;
    }
}



scalar &Vector::operator ()(counter number)
{
    assert(number >= 0 && number <= length);
    return *(data + number);
}

Vector &Vector::operator=(const Vector &V)
{
    if (data != nullptr)
        delete[] data;
    length = V.length;
    dataNewMemory(length);
//    for (counter i = 0; i < length; i++)
//    {
//        *(data + i) = *(V.GetDataPtr() + i);
//    }
    memcpy(data, V.data, sizeof(scalar) * length);
}
