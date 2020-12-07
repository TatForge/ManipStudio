#include "my_math.h"

//#include <QDebug>

using namespace RoboMath;

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
    assert(il.size() == rows*cols);
    DataNewMemory(cols, rows);
    copy(il.begin(), il.end(), data);
}

scalar &Matrix::at(const int &row, const int &col) const
{
    assert(col >= 0 && col <= cols);
    assert(row >= 0 && row <= rows);
    return *(data + index(row, col));
}

Matrix Matrix::Multuply(const Matrix &matrix1, const Matrix &matrix2) const
{
    Matrix result(matrix1.rows, matrix2.cols);
    scalar buff = 0;
    for (counter i = 0; i < matrix1.rows; i++)
    {
        for (counter j = 0 ; j < matrix2.cols; j++)
        {
            for (counter k = 0; k < matrix1.cols; k++)
            {
                buff += matrix1.at(i, k) * matrix2.at(k, j);
            }
            result.at(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

Matrix Matrix::MultiplyTranspose(const Matrix &matrix) const
{
    assert(cols == matrix.cols && "Failed assertion in 'Matrix::MultiplyTransform'");

    Matrix result(rows, matrix.rows);
    scalar buff(0);
    for (counter i = 0; i < rows; i++)
    {
        for (counter j = 0 ; j < matrix.rows; j++)
        {
            for (counter k = 0; k < cols; k++)
            {
                buff += this->at(i, k) * matrix.at(j, k);
            }
            result.at(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

Vector Matrix::MultiplyTranspose(const Vector &v) const
{
    assert(cols == v.GetLength() && "Failed assertion in 'Matrix::MultiplyTransform'");
    Vector result(rows);
    scalar buff(0);
    for (counter i = 0; i < rows; i++)
    {
        for (counter j = 0; j < cols; j++)
        {
            buff += *(data + i * cols + j) * v(j);
        }
        result(i) = buff;
        buff = 0;
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

void Matrix::PlusMatrix(const Matrix &m)
{
    assert(rows == m.rows && cols == m.cols);
    for (int i = 0; i < rows * cols; i++)
        *(data + i) += *(m.data + i);
}

void Matrix::SetSubmatrix(const Matrix &source, const int col, const int row)
{
    assert(source.rows <= this->rows && source.cols <= this->cols);
    assert((col + source.cols) <= this->cols && (row + source.rows) <= this->rows);
    int step = this->cols - source.cols;
    scalar *from = source.GetDataPtr();
    scalar *to = this->data + col + row * this->cols;
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
#ifdef DEBUG_COUT
    for (counter i =0; i < rows*cols; i++)
    {
        cout << " data " << *(this->data + i) << endl;
    }
#endif

}

void Matrix::SetColumn(const Vector &v, const int ColNumber)
{
    assert(ColNumber <= cols && ColNumber >= 0);
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
void Matrix::SetColumn(const Vector3 &v, const int ColNumber)
{
    assert(ColNumber <= (cols - 1) && ColNumber >= 0);
    assert(rows == 3);
    scalar *to = data + ColNumber;
    *(to           ) = v.x;
    *(to +     cols) = v.y;
    *(to + 2 * cols) = v.z;
}

void Matrix::SetSubColumn(const Vector3 &v, const int ColNumber, const int RowNumber)
{
    assert(ColNumber <= (cols - 1) && ColNumber >= 0);
    assert(RowNumber <= rows - 3);
    scalar *to = data + ColNumber + cols * RowNumber;
    *(to           ) = v.x;
    *(to +     cols) = v.y;
    *(to + 2 * cols) = v.z;
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
    for (; num_iter > 1; num_iter--)
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
            for (counter i = 0; i < row_length; i++)
            {
                scalar temp = *temp_ptr;
                *temp_ptr = *max_row_ptr;
                *max_row_ptr  = temp;
                temp_ptr ++;
                max_row_ptr  ++;
            }
        }
//        this->MatrixCout();
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
//        this->MatrixCout();
        main_elem_ptr += diagonal_step;
//        this->MatrixCout();
        --row_length;
//        --num_iter;
    }
}

void Matrix::GetTheRoots(Vector &V) const
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
        scalar *cur_elem_ptr = row_elem_ptr;
        scalar *result_ptr = result_ptr_last;
        for(counter j = 0; j < rows - i; j++)
        {
            accum -= (*cur_elem_ptr) * (*result_ptr);
            --cur_elem_ptr;
            --result_ptr;
        }
        *result_ptr = accum / *cur_elem_ptr;
        row_elem_ptr -= cols;
    }
}

void Matrix::MakeIdentity()
{
    //for square matrix only
    assert(cols >= 2 && "Identity matrix creation error (size < 2)");
    assert(cols == rows && "Identity matrix creation error (matrix not square)");
    int one_pos = 0;
    for (counter i = 0; i < cols * rows; i++)
    {
        if (i == one_pos)
        {
            *(data + i) = 1;
            one_pos += (cols + 1);
        }
        else
            *(data + i) = 0;
    }
}

void Matrix::MakeZero()
{
    for (counter i = 0; i < cols * rows; i++)
    {
        *(data + i) = 0;
    }
}

Matrix Matrix::operator*(const Matrix &matrix2) const
{
    Matrix result(rows, matrix2.cols);
    scalar buff = 0;
    for (counter i = 0; i < this->rows; i++)
    {
        for (counter j = 0 ; j < matrix2.cols; j++)
        {
            for (counter k = 0; k < this->cols; k++)
            {
                buff += this->at(i, k) * matrix2.at(k, j);
            }
            result.at(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

Vector Matrix::operator*(const Vector &vector2) const
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

Matrix Matrix::operator*(const scalar &value) const
{
    Matrix result(*this);
    scalar buff = 0;
    for (counter i = 0; i < rows; i++)
    {
        for (counter j = 0 ; j < cols; j++)
        {
            result(i, j) = result(i,j) * value;
        }
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

Matrix Matrix::operator+(const Matrix &matrix) const
{
    assert(rows == matrix.rows && cols == matrix.cols);
    Matrix result(rows, cols);
    for (int i = 0; i < rows * cols; i++)
    {
        *(data + i) = *(matrix.data + i);
    }
    return result;
}

bool Matrix::operator==(const Matrix &m) const
{
    assert(Size() == m.Size());
    //Сравнение векторов с точностью до 6 знака
    for (counter i = 0; i < m.rows * m.cols; i++)
    {
        if (!(round(*(data + i)*100000)/100000 == round(*(m.data + i) * 100000)/100000)){
            cout << *(data + i) << " " << *(m.data + i);
            return false;
        }
    }
    return true;
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
    delete[] tdata;
}

void Matrix::MatrixCout() const
{
//#ifdef DEBUG_COUT
    for (counter i = 0; i < rows; i++)
    {
        for (counter j = 0; j < cols; j++)
        {
            cout << setw(9) << *(data + i * cols + j) << "\t";
        }
        cout << endl;
    }
    cout << endl;
//#endif
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

Vector::Vector(int size, scalar value)
{
    length = size;
    dataNewMemory(length);
    for (counter i = 0; i < size; i++)
        *(data + i) = value;
}

Vector::Vector(const initializer_list<scalar> il)
{
    length = il.size();
    dataNewMemory(length);
    initializer_list<scalar>::iterator it = il.begin();
    for (counter i = 0; i < length; i++)
    {
        *(data + i) = *it++;
    }
}

Vector::~Vector()
{
    if (data != nullptr)
    {
        delete[] data;
    }
}

scalar Vector::MaxAbs()
{
    scalar result (0.0);
    for (counter i = 0; i < this->length; i++)
    {
        if (abs(*(this->data + i)) > result)
        {
            result = abs(*(this->data + i));
        }
    }
    return result;
}

scalar &Vector::operator()(counter number) const
{
    assert(number >= 0 && number <= length);
    return *(data + number);
}

Vector &Vector::operator=(const Vector &v)
{

    if (this == &v)
        return *this;
    if (data != nullptr)
        delete[] data;
    length = v.length;
    dataNewMemory(length);
//    for (counter i = 0; i < length; i++)
//    {
//        *(data + i) = *(V.GetDataPtr() + i);
//    }
    memcpy(data, v.data, sizeof(scalar) * length);
    return *this;
}

Vector &Vector::operator=(const double &value)
{
    for (int i = 0; i < this->length; i++)
    {
        *(this->data + i) = value;
    }
    return *this;
}

Vector Vector::operator*(const Vector &v)
{
    return Vector{

    };
}

scalar Vector::MultiplyTranspose(const Vector &v)
{
    //Умножение строки на столбец
    scalar result(0);
    for (int i = 0; i < v.GetLength(); i++)
        result += *(data + i) * (*(v.data + i));
    return result;
}

void Vector::operator*=(const scalar &value)
{
    for (counter i = 0; i < length; i++)
    {
        *(data + i) *= value;
    }
}

bool Vector::operator==(const Vector &v) const
{
    assert(length == v.length);
    //Сравнение векторов с точностью до 6 знака
    for (counter i = 0; i < v.length; i++)
    {
        if (!(round(*(data + i)*100000)/100000 == round(*(v.data + i) * 100000)/100000))
            return false;
    }
    return true;
}


Vector Vector::operator-(const Vector &v)
{
    assert(this->length == v.length && "The operator '-' has different vector lengths");
    Vector result(this->length);
    *(result.data + 0) = (*(v.data + 0) - *(this->data + 0));
    *(result.data + 1) = (*(v.data + 1) - *(this->data + 1));
    *(result.data + 2) = (*(v.data + 2) - *(this->data + 2));
    return result;

}

void Vector::VectorCout(const char *ch) const
{
    cout << ch << endl;
    for (counter i = 0; i < length; i++)
    {
        cout << setw(8) << *(data + i) << "  ";
    }
    cout << endl << endl;

}

void Vector::dataNewMemory(const int &length)
{
    data = new scalar [length];
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

void Vector3::Clear()
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
    return *this;
}

Vector3 Vector3::operator =(const Vector3 &V)
{
    this->x = V.x;
    this->y = V.y;
    this->z = V.z;
    return *this;
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

Vector3 Vector3::operator*(const Vector3 &V) const
{
    Vector3 result;
    result.x = y * V.z - z * V.y;
    result.y = z * V.x - x * V.z;
    result.z = x * V.y - y * V.x;
    return result;
}

Quaternion::Quaternion(const scalar &x_in, const scalar &y_in, const scalar &z_in, const scalar &w_in)
{
    w = w_in;
    x = x_in;
    y = y_in;
    z = z_in;
}

Quaternion::Quaternion(const Vector3 &coordXYZ, const scalar &angle)
{
    Vector3 v_t = coordXYZ.Normalize();
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

void Quaternion::EulerToQuat(const Vector3 &angles)
{
    //Input angles must be like (ZYX - Yaw, Pitch, Roll)
    scalar h1 = angles.z * 0.5;
    scalar h2 = angles.y * 0.5;
    scalar h3 = angles.x * 0.5;
    scalar cos1 = cos(h1);
    scalar sin1 = sin(h1);
    scalar cos2 = cos(h2);
    scalar sin2 = sin(h2);
    scalar cos3 = cos(h3);
    scalar sin3 = sin(h3);
    x = sin3 * cos2 * cos1 - cos3 * sin2 * sin1;
    y = cos3 * sin2 * cos1 + sin3 * cos2 * sin1;
    z = cos3 * cos2 * sin1 - sin3 * sin2 * cos1;
    w = cos3 * cos2 * cos1 + sin3 * sin2 * sin1;
}

Vector3 Quaternion::GetEulerAngles() const
{
    scalar roll  = atan2((2 * (w * x + y * z)), (1 - 2 * (x * x + y * y)));
    scalar sinp = 2 * (w * y - z * x);
    scalar pitch;
    abs(sinp) >= 1 ? pitch = std::copysign(M_PI / 2, sinp) : pitch = asin(sinp);
    scalar yaw   = atan2((2 * (w * z + x * y)), (1 - 2 * (y * y + z * z)));
    return Vector3(roll, pitch, yaw);
}

void Quaternion::Invert()
{
    x *= - 1;
    y *= - 1;
    z *= - 1;
}

scalar Quaternion::GetLength() const
{
    return (x * x + y * y + z * z + w * w);
}

Vector3 Quaternion::GetAxis() const
{
    scalar s = 1.0 - w * w;
    if (s == 0)
        return Vector3(0, 0, 0);
    s = 1.0 / sqrt(s);
    return Vector3(x * s, y * s, z * s);
}

long double Quaternion::GetAngle() const
{
    return (2.0 * acosl(w));
}

Quaternion Quaternion::operator*(const Quaternion &q) const
{
    scalar ww = w * q.w - x * q.x - y * q.y - z * q.z;
    scalar xx = w * q.x + x * q.w + y * q.z - z * q.y;
    scalar yy = w * q.y - x * q.z + y * q.w + z * q.x;
    scalar zz = w * q.z + x * q.y - y * q.x + z * q.w;
    return Quaternion(xx, yy, zz, ww);
}

Quaternion Quaternion::operator*(const Vector3 &v) const
{
    scalar ww = - x * v.x - y * v.y - z * v.z;
    scalar xx =   w * v.x + y * v.z - z * v.y;
    scalar yy =   w * v.y - x * v.z + z * v.x;
    scalar zz =   w * v.z + x * v.y - y * v.x;
    return Quaternion(xx, yy, zz, ww);
}

void Quaternion::Scale(const scalar &s)
{
    x *= s;
    y *= s;
    z *= s;
    w *= s;
}

void Quaternion::Normalize()
{
    scalar l = sqrt(x * x + y * y + z * z + w * w);
    x /= l;
    y /= l;
    z /= l;
}

Vector3 Quaternion::RotateVector(const Vector3 &v)
{
    if (x == 0 && y == 0 && z == 0)
        return v;
    Vector3 v_t = v.Normalize();
    scalar l = v.Length();
    this->Normalize();
    auto t = *this * v_t;
    this->Invert();
    t = t * (*this);
    v_t.x = t.x;
    v_t.y = t.y;
    v_t.z = t.z;
    v_t = v_t * l;
    return v_t;
}

Tensor::Tensor()
{
}

Tensor::Tensor(const initializer_list<scalar> &il)
{
    assert(il.size() == 6);
    int i = 0;
    for (scalar el:il)
    {
        data[i] = el;
        i++;
    }
}

scalar Trace(const Matrix &ma)
{
    //only for square matrix
    assert(ma.GetColsCount() == ma.GetRowsCount());
    //        Vector v(ma.GetColsCount());
    scalar result(0);
    counter diag_elem = 0;
    for (int i = 0; i < ma.GetColsCount(); i++)
    {
        result += *(ma.GetDataPtr() + i * ma.GetColsCount() + i);
    }
    return result;
}
