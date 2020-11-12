#include "my_math.h"

using namespace RoboMath;

//const Vector3 Vector3::unit_x(1.0, 0.0, 0.0);
//const Vector3 Vector3::unit_y(0.0, 1.0, 0.0);
//const Vector3 Vector3::unit_z(0.0, 0.0, 1.0);
//const Vector3 Vector3::unit_mz(0.0, 0.0, -1.0);

Matrix::Matrix()
{
    rows = 0;
    cols = 0;
    data = nullptr;
}

Matrix::Matrix(const int &rows, const int &cols)
{
    this->rows = rows;
    this->cols = cols;
    DataNewMemory(cols, rows);
}

Matrix::Matrix(const Matrix &source)
{
    rows = source.rows;
    cols = source.cols;
    DataNewMemory();
    memcpy(data, source.data, sizeof(scalar) * rows * cols);
}

Matrix::Matrix(const initializer_list<initializer_list<scalar> > &il)
{
    //        int il_sec;
    rows = il.size();
    cols = il.begin()->size();
    DataNewMemory(cols, rows);

    int i = 0;
    for (auto l : il)
    {
        for (auto elem : l )
        {
            //                il_sec =  l.size();
            *(data + i) = elem;
#ifdef DEBUG_COUT
            cout << *(data + i);
#endif
            i++;
        }
        //            cout << endl;
    }
}

Matrix::Matrix(const initializer_list<scalar> &il)

{
    //Only for square matrix
    int i = 0;
    rows = sqrt(il.size());
    cols = sqrt(il.size());
    DataNewMemory(cols, rows);
    copy(il.begin(), il.end(), data);
}

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

size_t Matrix::Size() const
{
    return (sizeof(scalar) * this->cols * this->rows);
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
    //Получение корней СЛАУ методом Гаусса
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

Matrix Matrix::operator*(const Matrix &matrix2)
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

Vector Matrix::operator*(const Vector &vector2)
{
    const scalar *vector_data_ptr;
    Vector result(this->rows);
    scalar *result_ptr = result.GetDataPtr();
    scalar *jacoby_data_ptr = data;
    for (short i = 0; i < this->rows; i++)
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

Matrix Matrix::operator=(const Matrix &matrix1)
{
    if (this == &matrix1)
        return *this;
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

scalar &Matrix::operator()(const int &row, const int &col)
{
    assert(col >= 0 && col <= this->cols);
    assert(row >= 0 && row <= this->rows);
    return *(data + index(row, col));
}

void Matrix::Transpose()
{
    assert(this->data != nullptr && "Matrix has no data in 'Matrix::Transpose'");
    assert(this->cols > 1 && this->rows > 1 && "Matrix rows or cols < 1 in 'Matrix::Transpose'");
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



Vector::Vector()
{
    length = 0;
    data = nullptr;
}

Vector::Vector(int size)
{
    length = size;
    dataNewMemory(length);
}

Vector::Vector(initializer_list<scalar> il)
{
    length = il.size();
    dataNewMemory(length);
    initializer_list<scalar>::iterator it = il.begin();
    for (int i = 0; i < length; i++)
    {
        *(data + i) = *it++;
    }
}

Vector::~Vector()
{
    delete [] data;
}

scalar &Vector::operator()(counter number)
{
    assert(number >= 0 && number <= length);
    return *(data + number);
}

Vector &Vector::operator=(const Vector &V)
{

    if (this == &V)
        return *this;
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

Vector &Vector::operator=(const double &value)
{
    for (int i = 0; i < this->length; i++)
    {
        *(this->data + i) = value;
    }
}

Vector Vector::operator-(const Vector &V)
{
    assert(this->length == V.length && "The operator '-' has different vector lengths");
    Vector result(this->length);
    *(result.data + 0) = (*(V.data + 0) - *(this->data + 0));
    *(result.data + 1) = (*(V.data + 1) - *(this->data + 1));
    *(result.data + 2) = (*(V.data + 2) - *(this->data + 2));
    return result;

}

void Vector3::Rotate(const scalar &theta, const Vector3 &V)
{
    scalar c = cos(theta);
    scalar s = sin(theta);
    scalar dotw = (x * V.x + y * V.y + z * V.z);
    scalar v0x = dotw * V.x;
    scalar v0y = dotw * V.y;
    scalar v0z = dotw * V.z;
    scalar v1x = x - v0x;
    scalar v1y = y - v0y;
    scalar v1z = z - v0z;
    scalar v2x = V.y * v1z - V.z * v1y;
    scalar v2y = V.z * v1x - V.x * v1z;
    scalar v2z = V.x * v1y - V.y * v1x;
    x = v0x + c*v1x + s*v2x;
    y = v0y + c*v1y + s*v2y;
    z = v0z	+ c*v1z + s*v2z;
}

void Vector3::Zero()
{
    this->x = 0;
    this->y = 0;
    this->z = 0;
}

Vector3 Vector3::Normalize() const
{
    scalar l = this->Length();
    if (l == 0)
        return Vector3(0, 0, 0);
    return Vector3(x/l, y/l, z/l);
}

Vector3 Vector3::operator=(const Vector &V)
{
    assert(V.GetLength() == 3 && "vector size != 3 in Vector3 operator '=' (V3 = V)");
    x = *(V.GetDataPtr() + 0);
    y = *(V.GetDataPtr() + 1);
    z = *(V.GetDataPtr() + 2);
}

Vector3 Vector3::operator=(const Vector3 &V)
{
    this->x = V.x;
    this->y = V.y;
    this->z = V.z;
}

Vector3 Vector3::operator -(const Vector3 &V) const
{
    return Vector3(this->x - V.x, this->y - V.y, this->z - V.z);
}

Vector3 Vector3::operator+(const Vector3 &V) const
{
    return Vector3(this->x + V.x, this->y + V.y, this->z + V.z);
}

Vector3 &Vector3::operator +=(const Vector3 &V)
{
    this->x += V.x;
    this->y += V.y;
    this->z += V.z;
    return *this;
}

Vector3 &Vector3::operator*(const scalar &value)
{
    x *= value;
    y *= value;
    z *= value;
    return *this;
}

Quaternion::Quaternion(const scalar &x_in, const scalar &y_in, const scalar &z_in, const scalar &w_in)
{
    w = w_in;
    x = x_in;
    y = y_in;
    z = z_in;
}

Quaternion::Quaternion(const Vector3 &v, const scalar &angle)
{
    Vector3 v_t = v.Normalize();
    this->w = cos(angle / 2);
    this->x = v_t.x * sin(angle / 2);
    this->y = v_t.y * sin(angle / 2);
    this->z = v_t.z * sin(angle / 2);
}

void Quaternion::Create(const Vector3 &rotate_vector, const scalar &rotate_angle)
{

    auto r_v = rotate_vector.Normalize();
    w = cos(rotate_angle / 2);
    x = r_v.x * sin(rotate_angle / 2);
    y = r_v.y * sin(rotate_angle / 2);
    z = r_v.z * sin(rotate_angle / 2);
}
