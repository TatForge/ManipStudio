#ifndef MY_MATH_H
#define MY_MATH_H


#include <initializer_list>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <memory>
#include <cstring>
#include <chrono>
#include <assert.h>


#define RAD M_PI/180
#define DEG 180/M_PI

#define DOUBLE_SCALAR
//#define DEBUG_COUT


#ifdef DOUBLE_SCALAR
typedef double scalar;
#else
typedef float scalar;
#endif

const scalar g = 9.80665;

typedef unsigned short counter;
using namespace std;

inline static scalar ToRad(scalar degrees) {return degrees * RAD;};
inline static scalar ToDeg(scalar radians) {return radians * DEG;};

namespace RoboMath
{

    class Quaternion;
    class Vector;
    class Vector3;
    class Matrix;
    struct Polynom
    {
        scalar elem[6]{0,0,0,0,0,0};
    };

    class Vector
    {
    public:
        Vector();
        Vector(int size);
        Vector(int size, scalar value);
        Vector(const initializer_list<scalar> il);
        ~Vector();

        int GetLength() const {return length;};
        scalar MaxAbs();
        scalar* GetDataPtr() const {return data;};
        scalar& operator()(const counter &number);
        scalar& at(const counter &number);
        scalar at(const counter &number) const;
        Vector& operator=(const Vector &v);
        Vector& operator=(const double &value);
        Vector operator*(const Vector &v);
        scalar MultiplyTranspose(const Vector &v);
        void  operator*=(const scalar &value);
        bool operator==(const Vector &v) const;
        Vector  operator-(const Vector &v);
        void VectorCout(const char *ch) const;
    protected:
        int length;
        scalar *data;
        void dataNewMemory(const int &length);
    };
    class Tensor
    {
    public:
        Tensor();
        Tensor(const initializer_list<scalar> &il);
        scalar& xx(){return *data;};
        scalar& yy(){return *(data + 1);};
        scalar& zz(){return *(data + 2);};
        scalar& xy(){return *(data + 3);};
        scalar& xz(){return *(data + 4);};
        scalar& yz(){return *(data + 5);};
    private:
        scalar data[6];
    };

    class Vector3
    {
    public:

        Vector3() : x(0), y(0), z(0) {};
        Vector3(const scalar &xn, const scalar &yn, const scalar &zn)
            : x(xn), y(yn), z(zn) {};
        void Rotate(const scalar &theta, const RoboMath::Vector3 &V);
        void Clear();
        scalar  Length()    const {return sqrt(x * x + y * y + z * z);};
        Vector3 Normalize() const;
        Vector3 operator= (const Vector &V);
        Vector3 operator= (const Vector3 &V);
        Vector3 operator- (const Vector3 &V) const;
        Vector3 operator+ (const Vector3 &V) const;
        Vector3 operator* (const Vector3 &V) const;
        Vector3 &operator+= (const Vector3 &V);
        Vector3 &operator* (const scalar &value);
    public:
        scalar x;
        scalar y;
        scalar z;
    };

    class Matrix
    {
    public:
        Matrix();
        Matrix(const int &rows, const int &cols);
        Matrix(const Matrix& source);
        Matrix(Matrix &&source);
        Matrix(const initializer_list<initializer_list<scalar>> &il);
        ~Matrix()
        {
            if(data != nullptr)
                delete[] data;
        }
        scalar &at(const int &row, const int &col) const;

        Matrix Multuply(const Matrix &matrix1, const Matrix &matrix2) const;
        void Multiply(const Matrix &matrix2, Matrix *ret_matrix) const;
        Matrix MultiplyTranspose(const Matrix &matrix) const;
        Vector MultiplyTranspose(Vector &v) const;

        size_t Size() const;
        void Transpose();
        void AddDiagonal(const scalar &d);
        void PlusMatrix(const Matrix &m);
        void SetSubmatrix(const Matrix &source, const int col, const int row);
        void SetColumn(const Vector &v, const int ColNumber);
        void SetColumn(const Vector3 &v, const int col);
        void SetSubColumn(const Vector3 &v, const int ColNumber, const int RowNumber);
        void SolveGauss();               //Решение СЛАУ методом Гаусса (приведение к ступенчатому виду)
        void GetTheRoots(Vector &V) const;     //Получение корней уравнения
        void MakeIdentity();
        void MakeZero();
//        Matrix &operator *(const Matrix &matrix2) const;
        Matrix operator*(const Matrix &matrix2) const;
        Vector operator*(const Vector &vector2) const;
        Matrix operator*(const scalar &value) const;
        Matrix operator=(const Matrix &matrix1);
        void operator=(Matrix &&matrix1);
        Matrix operator+(const Matrix &matrix) const;
        bool operator==(const Matrix &m) const;

        scalar& operator()(const int &row, const int &col);
        int GetColsCount() const { return m_cols; }
        int GetRowsCount() const { return m_rows; }
        scalar* GetDataPtr() const {return data;};
        void MatrixCout(const char *ch = "Matrix") const;


    private:
        int m_rows;
        int m_cols;
        scalar *data;
        void DataNewMemory()
        {
            data = new scalar[m_rows*m_cols];
        }
        void DataNewMemory(const int &rows, const int &cols)
        {
            data = new scalar[rows*cols];
    #ifdef DEBUG_COUT
            cout << "Data matrix was initialized with rows = "
                 << rows << " end cols = " << cols << endl;
            cout << endl;
    #endif
        }
        int index(const int &row, const int &col) const { return row * m_cols + col;}
    };

    class Matrix4x4
    {
    public:
        scalar a11,a12,a13,a14,a21,a22,a23,a24,a31,a32,a33,a34,a41,a42,a43,a44;
        Matrix4x4()
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
        };
        Matrix4x4(const scalar &a11, const scalar &a12, const scalar &a13, const scalar &a14,
                  const scalar &a21, const scalar &a22, const scalar &a23, const scalar &a24,
                  const scalar &a31, const scalar &a32, const scalar &a33, const scalar &a34,
                  const scalar &a41, const scalar &a42, const scalar &a43, const scalar &a44)
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
        void operator=(const Matrix &m)
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
        void operator*=(const scalar &sc)
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
        Matrix4x4 MultiplyTranspose(const Matrix4x4 &b)
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
        Vector MultiplyTranspose(const Vector &v)
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
        Matrix4x4 operator*(const Matrix4x4 &b)
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
        void MatrixCout(const char *ch)
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
        void MakeIdentity()
        {
            MakeNull();
            a11 = 1;
            a22 = 1;
            a33 = 1;
            a44 = 1;
        }
        void MakeNull()
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
    };


    class Quaternion
    {
    public:
        Quaternion() : x(0), y(0), z(0), w(1){};
        Quaternion(const scalar &x_in, const scalar &y_in, const scalar &z_in, const scalar &w_in);
        Quaternion(const RoboMath::Vector3 &coordXYZ, const scalar &angle);
        void Create(const RoboMath::Vector3 &rotate_vector, const scalar &rotate_angle);
        void EulerToQuat(const Vector3 &angles);
        Vector3 GetEulerAngles() const;
        void Invert();
        scalar GetLength() const;
        Vector3 GetAxis() const;
        long double GetAngle() const;
        Quaternion operator* (const Quaternion &q) const;
        Quaternion operator* (const Vector3 &v) const;
        void Scale(const scalar &s);
        void Normalize();
        Vector3 RotateVector(const Vector3 &v);
    private:
        scalar x;
        scalar y;
        scalar z;
        scalar w;
    };
    class MatrixOfMatrix6x6
    {
    public:
        MatrixOfMatrix6x6()
        {
            rows = 6;
            cols = 6;
            for (int i = 0; i < 36; i++)
            {
                Matrix *temp = new Matrix(4,4);
                temp->MakeIdentity();
                *(data + i) = *temp;
            }
//            for (int i = 0 ; i < 36; i++)
//            {
//                cout << "Elem number" << (data + i) << endl;
//                (data + i)->operator()(3,3) = i;
//                (data + i)->MatrixCout();
//            }
//            (data + 15)->MatrixCout();
        }
        Matrix& operator()(const int &row, const int &col)
        {
            return *(data + row * cols + col);
        }
    private:
        counter rows, cols;
        Matrix *data = new Matrix[36];
    };

    const Vector3 unit_x(1.0, 0.0, 0.0);
    const Vector3 unit_y(0.0, 1.0, 0.0);
    const Vector3 unit_z(0.0, 0.0, 1.0);
    const Vector3 unit_mz(0.0, 0.0, -1.0);

    static scalar Trace(const Matrix4x4 &m)
    {
        return (m.a11 + m.a22 + m.a33 + m.a44);
    }
    static scalar Trace(const Matrix &ma)
    {
        //only for square matrix
        assert(ma.GetColsCount() == ma.GetRowsCount());
        scalar result(0);
        counter diag_elem = 0;
        for (int i = 0; i < ma.GetColsCount(); i++)
        {
            result += *(ma.GetDataPtr() + i * ma.GetColsCount() + i);
        }
        return result;
    }
}

class checkTime
{
public:
    checkTime()
    {
        start = std::chrono::high_resolution_clock::now();
        checked = false;
    }
    ~checkTime()
    {
        if (!checked)
        {
            stop = std::chrono::high_resolution_clock::now();
            d = stop - start;
            cout << "duration = " << d.count() << endl << " microsec";
        }
    }
    std::chrono::duration<double> duration()
    {
        stop = std::chrono::high_resolution_clock::now();
        d = stop - start;
        checked = true;
        return d;
    }

private:
    std::chrono::duration<double> d;
    std::chrono::_V2::system_clock::time_point start;
    std::chrono::_V2::system_clock::time_point stop;
    bool checked;
};
class AllMath
{
    template<typename T> static inline T min(T t1, T t2) {return (t1 < t2 ? t1 : t2);};
    template<typename T> static inline T max(T t1, T t2) {return (t1 > t2 ? t1 : t2);};

};

#endif // MY_MATH_H
