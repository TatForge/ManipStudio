#include <initializer_list>
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>
#include <cstring>
#include <chrono>
#include <assert.h>

#ifndef MY_MATH_H
#define MY_MATH_H

#define DOUBLE_SCALAR
//#define DEBUG_COUT

#ifdef DOUBLE_SCALAR
typedef double scalar;
#else
typedef float scalar;
#endif
typedef unsigned short counter;

using namespace std;


namespace RoboMath
{

    class Quaternion;
    class Vector;
    class Vector3;
    class Matrix;

    class Vector
    {
    public:
        Vector();
        Vector(int size);
        Vector(initializer_list<scalar> il);
        ~Vector();

        int GetLength() const {return length;};
        scalar* GetDataPtr () const {return data;};
        scalar& operator()(counter number);
        Vector& operator=(const Vector &V);
        Vector& operator=(const double &value);
        Vector  operator-(const Vector &V);
//        friend ostream &operator<< (ostream& os, const Vector &V);
    private:
        int length;
        scalar *data;
        void dataNewMemory(const int &length)
        {
            data = new scalar [length];
        }
    };


    class Vector3
    {
    public:

        Vector3() : x(0), y(0), z(0) {};
        Vector3(const scalar &xn, const scalar &yn, const scalar &zn)
            : x(xn), y(yn), z(zn) {};
        void Rotate(const scalar &theta, const RoboMath::Vector3 &V);
        void Zero();
        scalar Length() const {return sqrt(x * x + y * y + z * z);};
        Vector3 Normalize() const;
        Vector3 operator= (const Vector &V);
        Vector3 operator= (const Vector3 &V);
        Vector3 operator- (const Vector3 &V) const;
        Vector3 operator+ (const Vector3 &V) const;
        Vector3 &operator+= (const Vector3 &V);
        Vector3 &operator* (const scalar &value);
        scalar x, y, z;
    };


    static const Vector3 unit_x(1.0, 0.0, 0.0);
    static const Vector3 unit_y(0.0, 1.0, 0.0);
    static const Vector3 unit_z(0.0, 0.0, 1.0);
    static const Vector3 unit_mz(0.0, 0.0, -1.0);

    class Matrix
    {
    public:
        Matrix();
        Matrix(const int &rows, const int &cols);
        Matrix(const Matrix& source);
        Matrix(const initializer_list<initializer_list<scalar>> &il);
        Matrix(const initializer_list<scalar> &il);
        ~Matrix()
        {
    //        if(data != nullptr)
    //            delete[] data;
        }
        scalar &elem(const int &row, const int &col) const;

        Matrix Multuply(const Matrix &matrix1, const Matrix &matrix2);
        size_t Size() const;
        void Transpose();
        void AddDiagonal(const scalar &d);
        void SetSubmatrix(const Matrix &source);
        void SetColumn(Vector &v, int ColNumber);
        void SolveGauss();               //Решение СЛАУ методом Гаусса (приведение к ступенчатому виду)
        void GetTheRoots(Vector &V);     //Получение корней уравнения
        Matrix operator*(const Matrix &matrix2);
        Vector operator*(const Vector &vector2);

        Matrix operator=(const Matrix &matrix1);

        scalar& operator()(const int &row, const int &col);
        int GetColsCount() const { return cols; }
        int GetRowsCount() const { return rows; }
        scalar* GetDataPtr() const {return data;};
        void MatrixCout();


    private:
        int rows;
        int cols;
        scalar *data;
        void DataNewMemory()
        {
            data = new scalar[rows*cols];
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
        int index(const int &row, const int &col) const { return row * cols + col;}
    };
    class Quaternion
    {
    public:
//        Quaternion() : x(0), y(0), z(0), w(0){};
        Quaternion(const scalar &x_in, const scalar &y_in, const scalar &z_in, const scalar &w_in);
        Quaternion(const RoboMath::Vector3 &v, const scalar &angle);
        void Create(const RoboMath::Vector3 &rotate_vector, const scalar &rotate_angle);
        void Invert()
        {
            x *= -1;
            y *= -1;
            z *= -1;
        }
        scalar GetLength() const
        {
            return (x * x + y * y + z * z + w * w);
        }
        Quaternion operator* (const Quaternion &q) const
        {
            scalar ww = w * q.w - x * q.x - y * q.y - z * q.z;
            scalar xx = w * q.x + x * q.w + y * q.z - z * q.y;
            scalar yy = w * q.y - x * q.z + y * q.w + z * q.x;
            scalar zz = w * q.z + x * q.y - y * q.x + z * q.w;
            return Quaternion(xx, yy, zz, ww);/*w * q.x + x * q.w + y * q.z - z * q.y,
                              w * q.y - x * q.z + y * q.w + z * q.x,
                              w * q.z + x * q.y - y * q.x + z * q.w,
                              w * q.w - x * q.x - y * q.y - z * q.z);*/
        }
        Quaternion operator* (const Vector3 &v) const
        {
            scalar ww = - x * v.x - y * v.y - z * v.z;
            scalar xx = w * v.x + y * v.z - z * v.y;
            scalar yy = w * v.y - x * v.z + z * v.x;
            scalar zz = w * v.z + x * v.y - y * v.x;
            return Quaternion(xx, yy, zz, ww);//(w * v.x + x * 0 + y * v.z - z * v.y,
                              //w * v.y - x * v.z + y * 0 + z * v.x,
                              //w * v.z + x * v.y - y * v.x + z * 0,
                              //w * 0 - x * v.x - y * v.y - z * v.z);
        }
        void Scale(const scalar &s)
        {
            x *= x;
            y *= y;
            z *= z;
            w *= w;
        }
        void Normalize()
        {
            scalar l = sqrt(x * x + y * y + z * z + w * w);
            x /= l;
            y /= l;
            z /= l;
        }

        Vector3 RotateVector(const Vector3 &v)
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


    public:
        scalar x;
        scalar y;
        scalar z;
        scalar w;
//        RoboMath::Vector3 r_a; // rotation axis

    };
}

class checkTime
{
public:
    checkTime()
    {
        start = std::chrono::high_resolution_clock::now();
    }
    ~checkTime()
    {
        stop = std::chrono::high_resolution_clock::now();
        d = stop - start;
        cout << "duration = " << d.count() << endl;
    }
private:
    std::chrono::duration<double> d;
    std::chrono::_V2::system_clock::time_point start;
    std::chrono::_V2::system_clock::time_point stop;
};

class AllMath
{
    template<typename T> static inline T min(T t1, T t2) {return (t1 < t2 ? t1 : t2);};
    template<typename T> static inline T max(T t1, T t2) {return (t1 > t2 ? t1 : t2);};
};

#endif // MY_MATH_H
