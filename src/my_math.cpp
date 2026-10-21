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
    m_data_ptr = nullptr;
}

Matrix::Matrix(const int &rows, const int &cols)
{
    this->m_rows = rows;
    this->m_cols = cols;
    DataNewMemory(cols, rows);
}

Matrix::Matrix(const Matrix &source) : Matrix (source.m_rows, source.m_cols)
{
    memcpy(m_data_ptr, source.m_data_ptr, sizeof(scalar) * m_rows * m_cols);
}

Matrix::Matrix(Matrix &&source)
{
    this->m_cols = source.m_cols;
    this->m_rows = source.m_rows;
    this->m_data_ptr = source.m_data_ptr;
    source.m_data_ptr = nullptr;
}

Matrix::Matrix(const initializer_list<initializer_list<scalar> > &il) : Matrix (il.size(), il.begin()->size())
{
    int i = 0;
    for (auto l : il)
    {
        for (auto elem : l)
        {
            //                il_sec =  l.size();
            *(m_data_ptr + i) = elem;
#ifdef DEBUG_COUT
            cout << *(data + i);
#endif
            i++;
        }
        //            cout << endl;
    }
}

Matrix::~Matrix()
{
    if(m_data_ptr != nullptr)
        delete[] m_data_ptr;
}

scalar &Matrix::at(const int &row, const int &col) const
{
    assert(col >= 0 && col < m_cols);
    assert(row >= 0 && row < m_rows);
    return *(m_data_ptr + index(row, col));
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
    //ret_matrix = this * matrix2
    assert(m_cols == matrix2.m_rows);
    assert(ret_matrix->m_rows == m_rows && ret_matrix->m_cols == matrix2.m_cols);
    scalar buff = 0;
    for (counter i = 0; i < m_rows; i++)
    {
        for (counter j = 0 ; j < matrix2.m_cols; j++)
        {
            for (counter k = 0; k < m_cols; k++)
            {
                buff += at(i, k) * matrix2.at(k, j);
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
            buff += *(m_data_ptr + i * m_cols + j) * v(j);
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
    int len = RoboMath::min(m_rows, m_cols);
    scalar *scalarP(m_data_ptr);
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
        *(m_data_ptr + i) += *(m.m_data_ptr + i);
}

void Matrix::SetSubmatrix(const Matrix &source, const int col, const int row)
{
    assert(source.m_rows <= this->m_rows && source.m_cols <= this->m_cols);
    assert((col + source.m_cols) <= this->m_cols && (row + source.m_rows) <= this->m_rows);
    int step = this->m_cols - source.m_cols;
    scalar *from = source.GetDataPtr();
    scalar *to = this->m_data_ptr + col + row * this->m_cols;
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
    scalar *to = m_data_ptr + ColNumber - 1;
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
    scalar *to = m_data_ptr + ColNumber;
    *(to           ) = v.x;
    *(to +     m_cols) = v.y;
    *(to + 2 * m_cols) = v.z;
}

void Matrix::SetSubColumn(const Vector3 &v, const int ColNumber, const int RowNumber)
{
    assert(ColNumber <= (m_cols - 1) && ColNumber >= 0);
    assert(RowNumber <= m_rows - 3);
    scalar *to = m_data_ptr + ColNumber + m_cols * RowNumber;
    *(to             ) = v.x;
    *(to +     m_cols) = v.y;
    *(to + 2 * m_cols) = v.z;
}

void Matrix::SolveGauss()
{
    //Forward elimination with partial pivoting on the augmented matrix [A | b]:
    //afterwards A is upper triangular and GetTheRoots() does back substitution.
    const int n = min(m_rows, m_cols);
    for (int c = 0; c < n - 1; c++)
    {
        //row with the largest |element| in column c, at or below the diagonal
        int pivot = c;
        scalar max_abs = fabs(at(c, c));
        for (int r = c + 1; r < m_rows; r++)
        {
            if (fabs(at(r, c)) > max_abs)
            {
                max_abs = fabs(at(r, c));
                pivot = r;
            }
        }
        if (pivot != c)
        {
            for (int k = 0; k < m_cols; k++)
                std::swap(at(c, k), at(pivot, k));
        }
        if (at(c, c) == 0)
            continue; //singular column, nothing to eliminate
        for (int r = c + 1; r < m_rows; r++)
        {
            scalar alpha = at(r, c) / at(c, c);
            at(r, c) = 0.0;
            for (int k = c + 1; k < m_cols; k++)
                at(r, k) -= at(c, k) * alpha;
        }
    }
}

void Matrix::GetTheRoots(Vector &V) const
{
    //Получение корней СЛАУ методом Гаусса
    assert(V.GetLength() == m_rows);

    scalar *row_elem_ptr = m_data_ptr + m_cols * m_rows - 2;
    scalar *right_val_ptr = m_data_ptr + m_cols * m_rows - 1;
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
            *(m_data_ptr + i) = 1;
            one_pos += (m_cols + 1);
        }
        else
            *(m_data_ptr + i) = 0;
    }
}

void Matrix::MakeZero()
{
    for (counter i = 0; i < m_cols * m_rows; i++)
    {
        *(m_data_ptr + i) = 0;
    }
}
Matrix Matrix::operator *(const Matrix &matrix2) const
{
    Matrix result(m_rows, matrix2.m_cols);
    scalar buff = 0;
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
    scalar *jacoby_data_ptr = m_data_ptr;
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
    if (m_data_ptr != nullptr)
    {
        delete [] m_data_ptr;
    }
    m_rows = matrix1.m_rows;
    m_cols = matrix1.m_cols;
    DataNewMemory();
    memcpy(m_data_ptr, matrix1.m_data_ptr, sizeof(scalar) * m_rows * m_cols);

    return *this;
}

void Matrix::operator =(Matrix &&matrix1)
{
//    if (this->data == matrix1.data)
//        return *this;
    if (this->m_data_ptr != nullptr)
        delete[] m_data_ptr;
    m_cols = matrix1.m_cols;
    m_rows = matrix1.m_rows;
    m_data_ptr = matrix1.m_data_ptr;
    matrix1.m_data_ptr = nullptr;
}

Matrix Matrix::operator+(const Matrix &matrix) const
{
    assert(m_rows == matrix.m_rows && m_cols == matrix.m_cols);
    Matrix result(m_rows, m_cols);
    for (int i = 0; i < m_rows * m_cols; i++)
    {
        *(result.m_data_ptr + i) = *(m_data_ptr + i) + *(matrix.m_data_ptr + i);
    }
    return result;
}

bool Matrix::operator==(const Matrix &m) const
{
    assert(Size() == m.Size());
    //Сравнение векторов с точностью до 6 знака
    for (counter i = 0; i < m.m_rows * m.m_cols; i++)
    {
        if (!(round(*(m_data_ptr + i)*100000)/100000 == round(*(m.m_data_ptr + i) * 100000)/100000)){
//            cout << *(data + i) << " " << *(m.data + i);
            return false;
        }
    }
    return true;
}

scalar &Matrix::operator()(const int &row, const int &col)
{
    assert(col >= 0 && col < this->m_cols);
    assert(row >= 0 && row < this->m_rows);
    return *(m_data_ptr + index(row, col));
}

void Matrix::Transpose()
{
    assert(this->m_data_ptr != nullptr && "Matrix has no data in 'Matrix::Transpose'");
    assert(this->m_cols > 1 && this->m_rows > 1 && "Matrix rows or cols < 1 in 'Matrix::Transpose'");
    scalar *tdata = new scalar [m_rows*m_cols];

    for (counter i = 0; i < m_rows; i++)
        for (counter j = 0; j < m_cols; j++)
        {
#ifdef DEBUG_COUT
            cout << " data " << *(data + i*rows + j) << endl;
#endif
            *(tdata + j*m_rows + i) = *(m_data_ptr + i*m_cols + j);
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
    memcpy(m_data_ptr, tdata, sizeof(scalar) * m_rows * m_cols);
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
            cout << setw(11) << *(m_data_ptr + i * m_cols + j) << "\t";
        }
        cout << endl;
    }
    cout << endl;
//#endif
}

void Matrix::DataNewMemory()
{
    m_data_ptr = new scalar[m_rows*m_cols];
}

void Matrix::DataNewMemory(const int &rows, const int &cols)
{
    m_data_ptr = new scalar[rows*cols];
#ifdef DEBUG_COUT
    cout << "Data matrix was initialized with rows = "
                 << rows << " end cols = " << cols << endl;
    cout << endl;
#endif
}



Vector::Vector()
{
    m_length = 0;
    m_data = nullptr;
}

Vector::Vector(int size)
{
    m_length = size;
    dataNewMemory(m_length);
}

Vector::Vector(int size, scalar value)
{
    m_length = size;
    dataNewMemory(m_length);
    for (counter i = 0; i < size; i++)
        *(m_data + i) = value;
}

Vector::Vector(const initializer_list<scalar> il)
{
    m_length = il.size();
    dataNewMemory(m_length);
    initializer_list<scalar>::iterator it = il.begin();
    for (counter i = 0; i < m_length; i++)
    {
        *(m_data + i) = *it++;
    }
}

Vector::Vector(const Vector &v)
{
    dataNewMemory(v.m_length);
    m_length = v.m_length;
    memcpy(m_data, v.m_data, sizeof(scalar) * m_length);
}

Vector::~Vector()
{
    if (m_data != nullptr)
    {
        delete[] m_data;
    }
}

scalar Vector::MaxAbs()
{
    scalar result (0.0);
    for (counter i = 0; i < this->m_length; i++)
    {
        if (abs(*(this->m_data + i)) > result)
        {
            result = abs(*(this->m_data + i));
        }
    }
    return result;
}

scalar &Vector::operator()(const counter &number)
{
    assert(number >= 0 && number <= m_length);
    return *(m_data + number);
}

scalar &Vector::at(const counter &number)
{
    return *(m_data + number);
}

scalar Vector::at(const counter &number) const
{
    return *(m_data + number);
}

Vector &Vector::operator=(const Vector &v)
{

    if (this == &v)
        return *this;
    if (m_data != nullptr)
        delete[] m_data;
    m_length = v.m_length;
    dataNewMemory(m_length);
//    for (counter i = 0; i < length; i++)
//    {
//        *(data + i) = *(V.GetDataPtr() + i);
//    }
    memcpy(m_data, v.m_data, sizeof(scalar) * m_length);
    return *this;
}
Vector3 Vector3::operator/(const scalar &value)
{
//    x /= value;//wtf?
//    y /= value;
//    z /= value;
//    return *this;
    assert(value != 0 && " Error Vector3 / scalar; scalar = 0");
    return Vector3(x / value, y / value, z / value);
}

scalar Vector3::Angle(const Vector3 &V)
{
    scalar var = Dot(V) / (Length() * V.Length());
    //rounding can push |cos| slightly above 1
    if (var > 1)
        var = 1;
    else if (var < -1)
        var = -1;
    return acos(var);
}
Vector &Vector::operator=(const double &value)
{
    for (int i = 0; i < this->m_length; i++)
    {
        *(this->m_data + i) = value;
    }
    return *this;
}


scalar Vector::MultiplyTranspose(const Vector &v)
{
    //Умножение строки на столбец
    scalar result(0);
    for (int i = 0; i < v.GetLength(); i++)
        result += *(m_data + i) * (*(v.m_data + i));
    return result;
}

void Vector::operator*=(const scalar &value)
{
    for (counter i = 0; i < m_length; i++)
    {
        *(m_data + i) *= value;
    }
}

bool Vector::operator==(const Vector &v) const
{
    assert(m_length == v.m_length);
    //Сравнение векторов с точностью до 6 знака
    for (counter i = 0; i < v.m_length; i++)
    {
        if (!(round(*(m_data + i)*100000)/100000 == round(*(v.m_data + i) * 100000)/100000))
            return false;
    }
    return true;
}


Vector Vector::operator-(const Vector &v)
{
    assert(this->m_length == v.m_length && "The operator '-' has different vector lengths");
    Vector result(this->m_length);
    for (int i = 0; i < m_length; i++)
        *(result.m_data + i) = *(m_data + i) - *(v.m_data + i);
    return result;

}

void Vector::VectorCout(const char *ch) const
{
    cout << ch << endl;
    for (counter i = 0; i < m_length; i++)
    {
        cout << setw(8) << *(m_data + i) << "  ";
    }
    cout << endl << endl;

}

void Vector::dataNewMemory(const int &length)
{
    m_data = new scalar [length];
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

scalar Vector3::Dot(const Vector3 &V) const
{
    return (x * V.x + y * V.y + z * V.z);
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
    x = V.x;
    y = V.y;
    z = V.z;
    return *this;
}

Vector3 Vector3::operator -(const Vector3 &V) const
{
    return Vector3(x - V.x, y - V.y, z - V.z);
}

Vector3 Vector3::operator-() const
{
    return Vector3(-x, -y, -z);
}

Vector3 Vector3::operator+(const Vector3 &V) const
{
    return Vector3(x + V.x, y + V.y, z + V.z);
}

Vector3 &Vector3::operator +=(const Vector3 &V)
{
    x += V.x;
    y += V.y;
    z += V.z;
    return *this;
}

Vector3 &Vector3::operator-=(const Vector3 &V)
{
    x -= V.x;
    y -= V.y;
    z -= V.z;
    return *this;
}

Vector3 Vector3::operator*(const scalar &value)
{
    //    x *= value;
    //    y *= value;
    //    z *= value;
    //    return *this;
    return Vector3(x * value, y * value, z * value);
}

scalar Vector3::TriangleS(const Vector3 &V)
{
//    scalar a1 = y * V.z - z * V.y;
//    scalar a2 = z * V.x - x * V.z;
//    scalar a3 = x * V.y - y * V.x;
    return sqrt(pow((y * V.z - z * V.y), 2) + pow((z * V.x - x * V.z), 2) + pow((x * V.y - y * V.x), 2));
}

bool Vector3::operator <(Vector3 &v)
{
    return ((x < v.x) ||
            ((x == v.x) && (y < v.y)) ||
            ((x == v.x) && (y == v.y) && (z < v.z)));
}

bool Vector3::operator!=(Vector3 &v)
{
    return (x != v.x) || (y != v.y) || (z != v.z);
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
    Normalize();
}

Vector3 Quaternion::GetEulerAngles() const
{
    scalar sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1 - 1e-10)
    {
        //gimbal lock (pitch = +-90): only roll -+ yaw is defined, report it as roll with yaw = 0
        return Vector3(2 * atan2(x, w), std::copysign(M_PI / 2, sinp), 0);
    }
    scalar roll  = atan2((2 * (w * x + y * z)), (1 - 2 * (x * x + y * y)));
    scalar pitch = asin(sinp);
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
    if (w < -1)
        return (2.0 * acosl(-1));
    if (w > 1)
        return (2.0 * acosl(1));
    assert(w >= -1 && w <= 1 && "-1 < w < 1 assertion failed");
    return (2.0 * acosl(w));
}

Quaternion Quaternion::operator*(const Quaternion &q) const
{
    Quaternion res;
    res.w = w * q.w - x * q.x - y * q.y - z * q.z;
    res.x = w * q.x + x * q.w + y * q.z - z * q.y;
    res.y = w * q.y - x * q.z + y * q.w + z * q.x;
    res.z = w * q.z + x * q.y - y * q.x + z * q.w;
    res.Normalize();
    return res;
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
    w /= l;
    x /= l;
    y /= l;
    z /= l;
}

Vector3 Quaternion::RotateVector(const Vector3 &v) const
{
    if (x == 0 && y == 0 && z == 0)
        return v;
    Vector3 v_t = v.Normalize();
    scalar l = v.Length();
    Quaternion buff = *this;
    buff.Normalize();
    auto t = buff * v_t;
    buff.Invert();
    t = t * buff;
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

//scalar Trace(const Matrix &ma)
//{
//    //only for square matrix
//    assert(ma.GetColsCount() == ma.GetRowsCount());
//    //        Vector v(ma.GetColsCount());
//    scalar result(0);
//    for (int i = 0; i < ma.GetColsCount(); i++)
//    {
//        result += *(ma.GetDataPtr() + i * ma.GetColsCount() + i);
//    }
//    return result;
//}

Matrix4x4::Matrix4x4()
{
//    MakeNull();
}

Matrix4x4::Matrix4x4(const initializer_list<scalar> il) noexcept
{
    int i = 0;
    for (auto el : il)
    {
        a[i] = el;
        i++;
    }
}

void Matrix4x4::operator=(const Matrix &m) noexcept
{
    a[0]  = m.at(0,0);
    a[1]  = m.at(0,1);
    a[2]  = m.at(0,2);
    a[3]  = m.at(0,3);
    a[4]  = m.at(1,0);
    a[5]  = m.at(1,1);
    a[6]  = m.at(1,2);
    a[7]  = m.at(1,3);
    a[8]  = m.at(2,0);
    a[9]  = m.at(2,1);
    a[10] = m.at(2,2);
    a[11] = m.at(2,3);
    a[12] = m.at(3,0);
    a[13] = m.at(3,1);
    a[14] = m.at(3,2);
    a[15] = m.at(3,3);
}

void Matrix4x4::operator*=(const scalar &sc) noexcept
{
    for (int i = 0; i < 16; i++)
    {
        a[i] *= sc;
    }
}

Matrix4x4 Matrix4x4::MultiplyTranspose(const Matrix4x4 &b) noexcept
{
    Matrix4x4 result;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result.a[i * 4 + j] = a[i*4] * b.a[j * 4] + a[i*4 + 1] * b.a[j * 4 + 1] + a[i*4 + 2] * b.a[j * 4 + 2] + a[i*4 + 3] * b.a[j * 4 + 3];
        }
    }
    return result;
}

Vector Matrix4x4::MultiplyTranspose(const Vector &v) noexcept
{
    Vector res(v.GetLength());
    //            v.VectorCout("input vector");
    res(0) = a[0]*v.at(0) + a[1]*v.at(1) + a[2]*v.at(2) + a[3]*v.at(3);
    res(1) = a[4]*v.at(0) + a[5]*v.at(1) + a[6]*v.at(2) + a[7]*v.at(3);
    res(2) = a[8]*v.at(0) + a[9]*v.at(1) + a[10]*v.at(2) + a[11]*v.at(3);
    res(3) = a[12]*v.at(0) + a[13]*v.at(1) + a[14]*v.at(2) + a[15]*v.at(3);
//    res.VectorCout("res vector");
    return res;
}

Matrix4x4 Matrix4x4::operator*(const Matrix4x4 &b) const noexcept
{
    Matrix4x4 result;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            result.a[i * 4 + j] = a[i * 4] * b.a[j] + a[i*4 + 1] * b.a[4 + j] + a[i*4 + 2] * b.a[8 + j] + a[i*4 + 3] * b.a[12 + j];
        }
    }
    return result;
}


void Matrix4x4::MatrixCout(const char *ch) const noexcept
{
    cout << "___________________________"
         << ch
         << "___________________________"
         << endl;
    const int w = 12;
    cout << setw(w) << a[0]  << setw(w) << a[1]  << setw(w) << a[2]  << setw(w) << a[3]  << endl;
    cout << setw(w) << a[4]  << setw(w) << a[5]  << setw(w) << a[6]  << setw(w) << a[7]  << endl;
    cout << setw(w) << a[8]  << setw(w) << a[9]  << setw(w) << a[10] << setw(w) << a[11] << endl;
    cout << setw(w) << a[12] << setw(w) << a[13] << setw(w) << a[14] << setw(w) << a[15] << endl;
}

void Matrix4x4::MakeIdentity() noexcept
{
    MakeNull();
    a[0] = 1;
    a[5] = 1;
    a[10] = 1;
    a[15] = 1;
}

void Matrix4x4::MakeNull() noexcept
{
    for (counter i = 0; i < 16; i++)
    {
        a[i] = 0;
    }
}

bool Matrix4x4::operator ==(const Matrix4x4 &m) const noexcept
{
    for (int i = 0; i < 16; i++)
    {
        if (a[i] != m.a[i])
            return false;
    }
    return true;
}

scalar &Matrix4x4::operator()(const int &row, const int &col) noexcept
{
    return a[row * 4 + col];
}

scalar Matrix4x4::at(const int &row, const int &col) const noexcept
{
    return a[row * 4 + col];
}

int Matrix4x4::MaxElem() const noexcept
{
    //return maximum element col number
    scalar max_elem = a[0];
    int max_col = 0;
    for (int i = 1; i < 16; i++)
    {
        if (!(i == 3 || i == 7 || i >= 11 ))
            if (max_elem < a[i])
            {
                max_elem = a[i];
                max_col = i % 4;
            }
    }
    return max_col;
}

Quaternion Matrix4x4::ConvertToQuaternion() const noexcept
{
    //Shepperd's method: branch on the largest of (trace, m00, m11, m22) so the
    //square root argument is always >= 1 (stable also for 180 degree turns)
    const scalar m00 = a[0], m01 = a[1], m02 = a[2];
    const scalar m10 = a[4], m11 = a[5], m12 = a[6];
    const scalar m20 = a[8], m21 = a[9], m22 = a[10];
    const scalar trace = m00 + m11 + m22;
    RoboMath::Quaternion res;
    if (trace > 0)
    {
        scalar s = sqrt(trace + 1.0) * 2; // s = 4w
        res = Quaternion((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s);
    }
    else if (m00 > m11 && m00 > m22)
    {
        scalar s = sqrt(1.0 + m00 - m11 - m22) * 2; // s = 4x
        res = Quaternion(0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s);
    }
    else if (m11 > m22)
    {
        scalar s = sqrt(1.0 + m11 - m00 - m22) * 2; // s = 4y
        res = Quaternion((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s);
    }
    else
    {
        scalar s = sqrt(1.0 + m22 - m00 - m11) * 2; // s = 4z
        res = Quaternion((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s);
    }
    res.Normalize();
    return res;
}

Vector3 Matrix4x4::ConvertToVector3() const noexcept
{
    return Vector3(a[3],a[7],a[11]);
}

Polynom::Polynom()
{
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
            joint[i].a[j] = 0;
    }
}

scalar Polynom::GetAngle(const int &joint_number, const scalar &t_m) const
{
    scalar t = t_m / 1000.0;
    return  joint[joint_number].a[5] * pow(t, 5) + joint[joint_number].a[4] * pow(t, 4) + joint[joint_number].a[3] * pow(t, 3) +
            joint[joint_number].a[2] * pow(t, 2) + joint[joint_number].a[1] * t + joint[joint_number].a[0];
}

scalar Polynom::GetSpeed(const int &joint_number, const scalar &t_m) const
{
    scalar t = t_m / 1000.0;
    return  5 * joint[joint_number].a[5] * pow(t, 4) + 4 * joint[joint_number].a[4] * pow(t, 3) + 3 * joint[joint_number].a[3] * pow(t, 2) +
            2 * joint[joint_number].a[2] * pow(t, 1) + joint[joint_number].a[1];
}

scalar Polynom::GetAcceleration(const int &joint_number, const scalar &t_m) const
{
    scalar t = t_m / 1000.0;
    return  20 * joint[joint_number].a[5] * pow (t, 3) + 12 * joint[joint_number].a[4] * pow(t, 2) + 6 * joint[joint_number].a[3] * t + 2 * joint[joint_number].a[2];
}

void Polynom::CalcPolynom(const Angles * const a0, const Angles * const a1, const scalar t_millisec)
{
    scalar t = t_millisec / 1000.0;
    for (int i = 0; i < 6; i++)
    {
        joint[i].a[0] =   a0->q[i];
        joint[i].a[1] =   a0->q_p[i];
        joint[i].a[2] =   0.5 * a0->q_pp[i];
        joint[i].a[3] = - 10/pow(t, 3) * a0->q[i] - 6/pow(t, 2) * a0->q_p[i] - 3/(2*t)          * a0->q_pp[i]
                + 10/pow(t, 3) * a1->q[i] - 4/pow(t, 2) * a1->q_p[i] + 1/(2*t)          * a1->q_pp[i];
        joint[i].a[4] =   15/pow(t, 4) * a0->q[i] + 8/pow(t, 3) * a0->q_p[i] + 3/(2*pow(t, 2))  * a0->q_pp[i]
                - 15/pow(t, 4) * a1->q[i] + 7/pow(t, 3) * a1->q_p[i] - 1/pow(t, 2)      * a1->q_pp[i];
        joint[i].a[5] = - 6 /pow(t, 5) * a0->q[i] - 3/pow(t, 4) * a0->q_p[i] - 1/(2*pow(t, 3))  * a0->q_pp[i]
                + 6 /pow(t, 5) * a1->q[i] - 3/pow(t, 4) * a1->q_p[i] + 1/(2*pow(t, 3))  * a1->q_pp[i];
    }
}

void Polynom::CalcPolynomLine(const Angles * const a0, const Angles * const a1, const int t_millisec)
{
    scalar t = t_millisec / 1000.0;
    for (int i = 0; i < 6; i++)
    {
        joint[i].a[0] = a0->q[i];
        joint[i].a[1] = a0->q_p[i];
        joint[i].a[2] = 0.5 * a0->q_pp[i];
        joint[i].a[3] = - 1/pow(t, 3) * a0->q[i] - 1/pow(t, 2) * a0->q_p[i] - 0.5/t * a0->q_pp[i] + 1/pow(t, 3) * a1->q[i];
    }
}

void Polynom::Cout(const char *ch) const
{
    cout << ch << endl;
    //            for (int i = 0; i < 6; i++)
    //            {
    //                cout << elem[i] << endl;
    //            }
    cout << "__________________________" << endl;
}

scalar CalcDistancePointToLine(Vector3 p, const Vector3 *l1, const Vector3 *l2)
{
    return sqrt(pow((l1->y - p.y) * (l2->z - l1->z) - (l1->z - p.z) * (l2->y - l1->y), 2) +
                pow((l1->x - p.x) * (l2->z - l1->z) - (l1->z - p.z) * (l2->x - l1->x), 2) +
                pow((l1->x - p.x) * (l2->y - l1->y) - (l1->y - p.y) * (l2->x - l1->x), 2))/
            sqrt(pow(l2->x - l1->x, 2) + pow(l2->y - l1->y, 2) + pow(l2->z - l1->z, 2));
}

scalar Distance(const StateVector &v1, const StateVector &v2)
{
    return (sqrt(pow(v1.x - v2.x, 2) + pow(v1.y - v2.y, 2) + pow(v1.z - v2.z, 2)));
}

MatrixOfMatrix6x6::MatrixOfMatrix6x6()
{
    rows = 6;
    cols = 6;
    for (int i = 0; i < 36; i++)
    {
        Matrix *temp = new Matrix(4,4);
        temp->MakeIdentity();
        *(data + i) = *temp;
    }
}

Matrix &MatrixOfMatrix6x6::operator()(const int &row, const int &col)
{
    return *(data + row * cols + col);
}

//scalar Trace(const Matrix4x4 &m)
//{
//    return (m.a[0] + m.a[5] + m.a[10] + m.a[15]);
//}

Vector3 StateVector::GetCoord() const
{
    return Vector3(x, y, z);
}

Vector3 StateVector::GetAngles() const
{
    return Vector3(roll, pitch, yaw);
}

CheckTime::CheckTime()
{
    start = std::chrono::high_resolution_clock::now();
    checked = false;
}

CheckTime::~CheckTime()
{
    if (!checked)
    {
        stop = std::chrono::high_resolution_clock::now();
        d = stop - start;
        cout << "duration = " << d.count() << " microsec" << endl;
    }
}

std::chrono::duration<double> CheckTime::duration()
{
    stop = std::chrono::high_resolution_clock::now();
    d = stop - start;
    checked = true;
    return d;
}

Angles::Angles()
{
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        q[i] = 0;
        q_p[i] = 0;
        q_pp[i] = 0;
    }
}

bool Angles::IsNull() const
{
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        if (q[i] != 0)
            return false;
        if (q_p[i] != 0)
            return false;
        if (q_pp[i] != 0)
            return false;
    }
    return true;
}


scalar Trace(const Matrix4x4 &m)
{
    return (m.a[0] + m.a[5] + m.a[10] + m.a[15]);
}

scalar Trace(const Matrix &ma)
{
    assert(ma.GetColsCount() == ma.GetRowsCount());
    scalar result(0);
    for (int i = 0; i < ma.GetColsCount(); i++)
    {
        result += *(ma.GetDataPtr() + i * ma.GetColsCount() + i);
    }
    return result;
}
