#include "my_math.h"

//#include <QDebug>

using namespace RoboMath;

//static Matrix temp_1(4,4);
//static Matrix temp_2(4,4);
Matrix* temp_1 = new Matrix{{6,6,6,6},
                            {6,6,6,6},
                            {6,6,6,6},
                            {6,6,6,6}};
Matrix* temp_2 = new Matrix{{7,7,7,7},
                            {7,7,7,7},
                            {7,7,7,7},
                            {7,7,7,7}};

Matrix::Matrix()
{
    m_rows = 0;
    m_cols = 0;
    data = nullptr;
}

Matrix::Matrix(const int &rows, const int &cols)
{
    this->m_rows = rows;
    this->m_cols = cols;
    DataNewMemory(cols, rows);
}

Matrix::Matrix(const Matrix &source) : Matrix (source.m_rows, source.m_cols)
{
    memcpy(data, source.data, sizeof(scalar) * m_rows * m_cols);
}

Matrix::Matrix(Matrix &&source)
{
    this->m_cols = source.m_cols;
    this->m_rows = source.m_rows;
    this->data = source.data;
    source.data = nullptr;
}

Matrix::Matrix(const initializer_list<initializer_list<scalar> > &il) : Matrix (il.size(), il.begin()->size())
{
    int i = 0;
    for (auto l : il)
    {
        for (auto elem : l)
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

scalar &Matrix::at(const int &row, const int &col) const
{
    assert(col >= 0 && col <= m_cols);
    assert(row >= 0 && row <= m_rows);
    return *(data + index(row, col));
}

Matrix Matrix::Multuply(const Matrix &matrix1, const Matrix &matrix2) const
{
    Matrix result(matrix1.m_rows, matrix2.m_cols);
    scalar buff = 0;
    for (counter i = 0; i < matrix1.m_rows; i++)
    {
        for (counter j = 0 ; j < matrix2.m_cols; j++)
        {
            for (counter k = 0; k < matrix1.m_cols; k++)
            {
                buff += matrix1.at(i, k) * matrix2.at(k, j);
            }
            result.at(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

void Matrix::Multiply(const Matrix &matrix2, Matrix *ret_matrix) const
{
    scalar buff = 0;
    for (counter i = 0; i < matrix2.m_rows; i++)
    {
        for (counter j = 0 ; j < matrix2.m_cols; j++)
        {
            for (counter k = 0; k < matrix2.m_cols; k++)
            {
                buff += matrix2.at(i, k) * matrix2.at(k, j);
            }
            ret_matrix->at(i, j) = buff;
            buff = 0;
        }
    }
}

Matrix Matrix::MultiplyTranspose(const Matrix &matrix) const
{
    assert(m_cols == matrix.m_cols && "Failed assertion in 'Matrix::MultiplyTransform'");

    Matrix result(m_rows, matrix.m_rows);
    scalar buff(0);
    for (counter i = 0; i < m_rows; i++)
    {
        for (counter j = 0 ; j < matrix.m_rows; j++)
        {
            for (counter k = 0; k < m_cols; k++)
            {
                buff += this->at(i, k) * matrix.at(j, k);
            }
            result.at(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}

Vector Matrix::MultiplyTranspose(Vector &v) const
{
    assert(m_cols == v.GetLength() && "Failed assertion in 'Matrix::MultiplyTransform'");
    Vector result(m_rows);
    scalar buff(0);
    for (counter i = 0; i < m_rows; i++)
    {
        for (counter j = 0; j < m_cols; j++)
        {
            buff += *(data + i * m_cols + j) * v(j);
        }
        result(i) = buff;
        buff = 0;
    }
    return result;
}

size_t Matrix::Size() const
{
    return (sizeof(scalar) * this->m_cols * this->m_rows);
}

void Matrix::AddDiagonal(const scalar &d)
{
    int len = min(m_rows, m_cols);
    scalar *scalarP(data);
    for (counter i  = 0; i < len; i++)
    {
        *scalarP += d;
        scalarP += m_cols + 1;
    }
}

void Matrix::PlusMatrix(const Matrix &m)
{
    assert(m_rows == m.m_rows && m_cols == m.m_cols);
    for (int i = 0; i < m_rows * m_cols; i++)
        *(data + i) += *(m.data + i);
}

void Matrix::SetSubmatrix(const Matrix &source, const int col, const int row)
{
    assert(source.m_rows <= this->m_rows && source.m_cols <= this->m_cols);
    assert((col + source.m_cols) <= this->m_cols && (row + source.m_rows) <= this->m_rows);
    int step = this->m_cols - source.m_cols;
    scalar *from = source.GetDataPtr();
    scalar *to = this->data + col + row * this->m_cols;
    for (counter i = 0; i < source.m_rows; i++)
    {
        for (counter j = 0; j < source.m_cols; j++)
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
    assert(ColNumber <= m_cols && ColNumber >= 0);
    assert(v.GetLength() == m_rows);
    scalar *to = data + ColNumber - 1;
    scalar *from = v.GetDataPtr();
    for (counter i = 0; i < v.GetLength(); i++)
    {
        *to= *from;
        to += this->m_cols;
        ++from;
    }
}
void Matrix::SetColumn(const Vector3 &v, const int ColNumber)
{
    assert(ColNumber <= (m_cols - 1) && ColNumber >= 0);
    assert(m_rows == 3);
    scalar *to = data + ColNumber;
    *(to           ) = v.x;
    *(to +     m_cols) = v.y;
    *(to + 2 * m_cols) = v.z;
}

void Matrix::SetSubColumn(const Vector3 &v, const int ColNumber, const int RowNumber)
{
    assert(ColNumber <= (m_cols - 1) && ColNumber >= 0);
    assert(RowNumber <= m_rows - 3);
    scalar *to = data + ColNumber + m_cols * RowNumber;
    *(to           ) = v.x;
    *(to +     m_cols) = v.y;
    *(to + 2 * m_cols) = v.z;
}

void Matrix::SolveGauss()
{
    int num_iter = min(m_cols, m_rows); // почему берется min, а не rows, ведь проверка по столбцам??
    // num_iter = rows;
    //First of all find maximum element in first col
    scalar *main_elem_ptr = data;
    scalar *max_row_ptr = nullptr;
    scalar *to = nullptr;
    scalar *from = nullptr;
    double alpha = 0.0;
    int row_length = m_cols;

    int diagonal_step = m_cols + 1;
    for (; num_iter > 1; num_iter--)
    {

        scalar *row_for_swap = main_elem_ptr;
        scalar max_abs = fabs(*main_elem_ptr);
        max_row_ptr = main_elem_ptr;
        //Find row with maximum (abs) element value
        for (counter j = 1; j < num_iter; j++)
        {
            if (*(data + m_cols * j) > max_abs)
            {
                max_abs = *(data + m_cols * j);
                max_row_ptr = data + m_cols * j;
            }
            else if ( - *(data + m_cols * j) > max_abs)
            {
                max_abs = -*(data + m_cols * j);
                max_row_ptr = data + m_cols * j;
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
            row_for_swap += m_cols;
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
    assert(V.GetLength() == m_rows);

    scalar *row_elem_ptr = data + m_cols * m_rows - 2;
    scalar *right_val_ptr = data + m_cols * m_rows - 1;
    scalar *result_ptr_last = V.GetDataPtr() + V.GetLength() - 1;
    scalar accum;

    for (counter i = m_rows; i > 0; i--)
    {
        accum = *(right_val_ptr);
        right_val_ptr -= m_cols;
        scalar *cur_elem_ptr = row_elem_ptr;
        scalar *result_ptr = result_ptr_last;
        for(counter j = 0; j < m_rows - i; j++)
        {
            accum -= (*cur_elem_ptr) * (*result_ptr);
            --cur_elem_ptr;
            --result_ptr;
        }
        *result_ptr = accum / *cur_elem_ptr;
        row_elem_ptr -= m_cols;
    }
}

void Matrix::MakeIdentity()
{
    //for square matrix only
    assert(m_cols >= 2 && "Identity matrix creation error (size < 2)");
    assert(m_cols == m_rows && "Identity matrix creation error (matrix not square)");
    int one_pos = 0;
    for (counter i = 0; i < m_cols * m_rows; i++)
    {
        if (i == one_pos)
        {
            *(data + i) = 1;
            one_pos += (m_cols + 1);
        }
        else
            *(data + i) = 0;
    }
}

void Matrix::MakeZero()
{
    for (counter i = 0; i < m_cols * m_rows; i++)
    {
        *(data + i) = 0;
    }
}
Matrix Matrix::operator *(const Matrix &matrix2) const
{
    Matrix result(m_rows, matrix2.m_cols);
    scalar buff = 0;
//    Matrix *res = temp_1;
//    if (*this == *temp_1)
//        res = temp_2;
    for (counter i = 0; i < this->m_rows; i++)
    {
        for (counter j = 0 ; j < matrix2.m_cols; j++)
        {
            for (counter k = 0; k < this->m_cols; k++)
            {
                buff += at(i, k) * matrix2.at(k, j);
            }
            result.at(i, j) = buff;
            buff = 0;
        }
    }
    return result;
}
//Matrix &Matrix::operator *(const Matrix &matrix2) const
//{
////    Matrix result(rows, matrix2.cols);
//    scalar buff = 0;
//    Matrix *res = temp_1;
//    if (*this == *temp_1)
//        res = temp_2;
//    for (counter i = 0; i < this->m_rows; i++)
//    {
//        for (counter j = 0 ; j < matrix2.m_cols; j++)
//        {
//            for (counter k = 0; k < this->m_cols; k++)
//            {
//                buff += at(i, k) * matrix2.at(k, j);
//            }
//            res->at(i, j) = buff;
//            buff = 0;
//        }
//    }
//    return *res;
//}

Vector Matrix::operator*(const Vector &vector2) const
{
    const scalar *vector_data_ptr;
    Vector result(this->m_rows);
    scalar *result_ptr = result.GetDataPtr();
    scalar *jacoby_data_ptr = data;
    for (short i = 0; i < this->m_rows; i++)
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
    for (counter i = 0; i < m_rows; i++)
    {
        for (counter j = 0 ; j < m_cols; j++)
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
    m_rows = matrix1.m_rows;
    m_cols = matrix1.m_cols;
    DataNewMemory();
    memcpy(data, matrix1.data, sizeof(scalar) * m_rows * m_cols);

    return *this;
}

void Matrix::operator =(Matrix &&matrix1)
{
//    if (this->data == matrix1.data)
//        return *this;
    if (this->data != nullptr)
        delete[] data;
    this->m_cols = matrix1.m_cols;
    this->m_rows = matrix1.m_rows;
    this->data = matrix1.data;
    matrix1.data = nullptr;
}

Matrix Matrix::operator+(const Matrix &matrix) const
{
    assert(m_rows == matrix.m_rows && m_cols == matrix.m_cols);
    Matrix result(m_rows, m_cols);
    for (int i = 0; i < m_rows * m_cols; i++)
    {
        *(data + i) = *(matrix.data + i);
    }
    return result;
}

bool Matrix::operator==(const Matrix &m) const
{
    assert(Size() == m.Size());
    //Сравнение векторов с точностью до 6 знака
    for (counter i = 0; i < m.m_rows * m.m_cols; i++)
    {
        if (!(round(*(data + i)*100000)/100000 == round(*(m.data + i) * 100000)/100000)){
//            cout << *(data + i) << " " << *(m.data + i);
            return false;
        }
    }
    return true;
}

scalar &Matrix::operator()(const int &row, const int &col)
{
    assert(col >= 0 && col <= this->m_cols);
    assert(row >= 0 && row <= this->m_rows);
    return *(data + index(row, col));
}

void Matrix::Transpose()
{
    assert(this->data != nullptr && "Matrix has no data in 'Matrix::Transpose'");
    assert(this->m_cols > 1 && this->m_rows > 1 && "Matrix rows or cols < 1 in 'Matrix::Transpose'");
    scalar *tdata = new scalar [m_rows*m_cols];

    for (counter i = 0; i < m_rows; i++)
        for (counter j = 0; j < m_cols; j++)
        {
#ifdef DEBUG_COUT
            cout << " data " << *(data + i*rows + j) << endl;
#endif
            *(tdata + j*m_rows + i) = *(data + i*m_cols + j);
        }
#ifdef DEBUG_COUT
    for(counter i = 0; i < cols * rows; i++)
    {
        cout << *(tdata + i) << endl;
    }
#endif
    int trows = m_cols;
    m_cols = m_rows;
    m_rows = trows;
    memcpy(data, tdata, sizeof(scalar) * m_rows * m_cols);
    delete[] tdata;
}

void Matrix::MatrixCout(const char *ch) const
{
//#ifdef DEBUG_COUT
    cout << "___________________________"
         << ch
         << "___________________________"
         << endl;
    for (counter i = 0; i < m_rows; i++)
    {
        for (counter j = 0; j < m_cols; j++)
        {
            cout << setw(9) << *(data + i * m_cols + j) << "\t";
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

scalar &Vector::operator()(const counter &number)
{
    assert(number >= 0 && number <= length);
    return *(data + number);
}

scalar &Vector::at(const counter &number)
{
    return *(data + number);
}

scalar Vector::at(const counter &number) const
{
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

Matrix4x4::Matrix4x4()
{
    MakeNull();
    this->a11 = 0;
    this->a12 = 0;
    this->a13 = 0;
    this->a14 = 0;
    this->a21 = 0;
    this->a22 = 0;
    this->a23 = 0;
    this->a24 = 0;
    this->a31 = 0;
    this->a32 = 0;
    this->a33 = 0;
    this->a34 = 0;
    this->a41 = 0;
    this->a42 = 0;
    this->a43 = 0;
    this->a44 = 0;
}

Matrix4x4::Matrix4x4(const scalar &a11, const scalar &a12, const scalar &a13, const scalar &a14, const scalar &a21, const scalar &a22, const scalar &a23, const scalar &a24, const scalar &a31, const scalar &a32, const scalar &a33, const scalar &a34, const scalar &a41, const scalar &a42, const scalar &a43, const scalar &a44)
{
    this->a11 = a11;
    this->a12 = a12;
    this->a13 = a13;
    this->a14 = a14;
    this->a21 = a21;
    this->a22 = a22;
    this->a23 = a23;
    this->a24 = a24;
    this->a31 = a31;
    this->a32 = a32;
    this->a33 = a33;
    this->a34 = a34;
    this->a41 = a41;
    this->a42 = a42;
    this->a43 = a43;
    this->a44 = a44;
}

void Matrix4x4::operator=(const Matrix &m)
{
    a11 = m.at(0,0);
    a12 = m.at(0,1);
    a13 = m.at(0,2);
    a14 = m.at(0,3);
    a21 = m.at(1,0);
    a22 = m.at(1,1);
    a23 = m.at(1,2);
    a24 = m.at(1,3);
    a31 = m.at(2,0);
    a32 = m.at(2,1);
    a33 = m.at(2,2);
    a34 = m.at(2,3);
    a41 = m.at(3,0);
    a42 = m.at(3,1);
    a43 = m.at(3,2);
    a44 = m.at(3,3);
}

void Matrix4x4::operator*=(const scalar &sc)
{
    a11 *= sc;
    a12 *= sc;
    a13 *= sc;
    a14 *= sc;
    a21 *= sc;
    a22 *= sc;
    a23 *= sc;
    a24 *= sc;
    a31 *= sc;
    a32 *= sc;
    a33 *= sc;
    a34 *= sc;
    a41 *= sc;
    a42 *= sc;
    a43 *= sc;
    a44 *= sc;
}

Matrix4x4 Matrix4x4::MultiplyTranspose(const Matrix4x4 &b)
{
    Matrix4x4 result;
    result.a11 = a11*b.a11 + a12*b.a12 + a13*b.a13 + a14*b.a14;
    result.a12 = a11*b.a21 + a12*b.a22 + a13*b.a23 + a14*b.a24;
    result.a13 = a11*b.a31 + a12*b.a32 + a13*b.a33 + a14*b.a34;
    result.a14 = a11*b.a41 + a12*b.a42 + a13*b.a43 + a14*b.a44;

    result.a21 = a21*b.a11 + a22*b.a12 + a23*b.a13 + a24*b.a14;
    result.a22 = a21*b.a21 + a22*b.a22 + a23*b.a23 + a24*b.a24;
    result.a23 = a21*b.a31 + a22*b.a32 + a23*b.a33 + a24*b.a34;
    result.a24 = a21*b.a41 + a22*b.a42 + a23*b.a43 + a24*b.a44;

    result.a31 = a31*b.a11 + a32*b.a12 + a33*b.a13 + a34*b.a14;
    result.a32 = a31*b.a21 + a32*b.a22 + a33*b.a23 + a34*b.a24;
    result.a33 = a31*b.a31 + a32*b.a32 + a33*b.a33 + a34*b.a34;
    result.a34 = a31*b.a41 + a32*b.a42 + a33*b.a43 + a34*b.a44;

    result.a41 = a41*b.a11 + a42*b.a12 + a43*b.a13 + a44*b.a14;
    result.a42 = a41*b.a21 + a42*b.a22 + a43*b.a23 + a44*b.a24;
    result.a43 = a41*b.a31 + a42*b.a32 + a43*b.a33 + a44*b.a34;
    result.a44 = a41*b.a41 + a42*b.a42 + a43*b.a43 + a44*b.a44;
    return result;
}

Vector Matrix4x4::MultiplyTranspose(const Vector &v)
{
    Vector res(v.GetLength());
    //            v.VectorCout("input vector");
    res(0) = a11*v.at(0) + a12*v.at(1) + a13*v.at(2) + a14*v.at(3);
    res(1) = a21*v.at(0) + a22*v.at(1) + a23*v.at(2) + a24*v.at(3);
    res(2) = a31*v.at(0) + a32*v.at(1) + a33*v.at(2) + a34*v.at(3);
    res(3) = a41*v.at(0) + a42*v.at(1) + a43*v.at(2) + a44*v.at(3);
    res.VectorCout("res vector");
    return res;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4 &b)
{
    Matrix4x4 result;
    result.a11 = a11*b.a11 + a12*b.a21 + a13*b.a31 + a14*b.a41;
    result.a12 = a11*b.a12 + a12*b.a22 + a13*b.a32 + a14*b.a42;
    result.a13 = a11*b.a13 + a12*b.a23 + a13*b.a33 + a14*b.a43;
    result.a14 = a11*b.a14 + a12*b.a24 + a13*b.a34 + a14*b.a44;

    result.a21 = a21*b.a11 + a22*b.a21 + a23*b.a31 + a24*b.a41;
    result.a22 = a21*b.a12 + a22*b.a22 + a23*b.a32 + a24*b.a42;
    result.a23 = a21*b.a13 + a22*b.a23 + a23*b.a33 + a24*b.a43;
    result.a24 = a21*b.a14 + a22*b.a24 + a23*b.a34 + a24*b.a44;

    result.a31 = a31*b.a11 + a32*b.a21 + a33*b.a31 + a34*b.a41;
    result.a32 = a31*b.a12 + a32*b.a22 + a33*b.a32 + a34*b.a42;
    result.a33 = a31*b.a13 + a32*b.a23 + a33*b.a33 + a34*b.a43;
    result.a34 = a31*b.a14 + a32*b.a24 + a33*b.a34 + a34*b.a44;

    result.a41 = a41*b.a11 + a42*b.a21 + a43*b.a31 + a44*b.a41;
    result.a42 = a41*b.a12 + a42*b.a22 + a43*b.a32 + a44*b.a42;
    result.a43 = a41*b.a13 + a42*b.a23 + a43*b.a33 + a44*b.a43;
    result.a44 = a41*b.a14 + a42*b.a24 + a43*b.a34 + a44*b.a44;
    return result;
}

void Matrix4x4::MatrixCout(const char *ch) const
{
    cout << "___________________________"
         << ch
         << "___________________________"
         << endl;
    const int w = 13;
    cout << setw(w) << a11 << setw(w) << a12 << setw(w) << a13 << setw(w) << a14 << endl;
    cout << setw(w) << a21 << setw(w) << a22 << setw(w) << a23 << setw(w) << a24 << endl;
    cout << setw(w) << a31 << setw(w) << a32 << setw(w) << a33 << setw(w) << a34 << endl;
    cout << setw(w) << a41 << setw(w) << a42 << setw(w) << a43 << setw(w) << a44 << endl;
}

void Matrix4x4::MakeIdentity()
{
    MakeNull();
    a11 = 1;
    a22 = 1;
    a33 = 1;
    a44 = 1;
}

void Matrix4x4::MakeNull()
{
    this->a11 = 0;
    this->a12 = 0;
    this->a13 = 0;
    this->a14 = 0;
    this->a21 = 0;
    this->a22 = 0;
    this->a23 = 0;
    this->a24 = 0;
    this->a31 = 0;
    this->a32 = 0;
    this->a33 = 0;
    this->a34 = 0;
    this->a41 = 0;
    this->a42 = 0;
    this->a43 = 0;
    this->a44 = 0;
}
