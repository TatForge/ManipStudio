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
#define JOINTS_COUNT 6

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

    struct Angles
    {
        scalar q_pp[JOINTS_COUNT];
        scalar q_p[JOINTS_COUNT];
        scalar q[JOINTS_COUNT];
        Angles()
        {
            for (int i = 0; i < JOINTS_COUNT; i++)
            {
                q[i] = 0;
                q_p[i] = 0;
                q_pp[i] = 0;
            }
        }
    };

    class Polynom
    {
    public:
        struct elem
        {
            scalar a[6];
        };
        elem joint[6];
        Polynom();
        scalar GetAngle(const int &joint_number, const scalar &t) const;
        scalar GetSpeed(const int &joint_number, const scalar &t) const;
        scalar GetAcceleration(const int &joint_number, const scalar &t) const;
        void CalcPolynom(const Angles *const a0, const Angles *const a1, const scalar t);
        void CalcPolynomLine(const Angles *const a0, const Angles *const a1, const int t_millisec);

        void Cout(const char *ch) const;
        friend bool CheckForLinear(Polynom &this_pol, Vector3 l1, Vector3 l2, const scalar &t, const scalar &delta_t, const scalar &delta_r);
    };
//    struct Point3
//    {
//        Point3(scalar &x_in, scalar &y_in, scalar &z_in) :
//            x(x_in), y(y_in), z(z_in){};
//        scalar x;
//        scalar y;
//        scalar z;
//    };

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
    class Matrix4x4
    {
    public:
        scalar a[16];
        Matrix4x4();;
        Matrix4x4(const initializer_list<scalar> il);
        void operator=(const Matrix &m);
        void operator*=(const scalar &sc);
        Matrix4x4 MultiplyTranspose(const Matrix4x4 &b);
        Vector MultiplyTranspose(const Vector &v);
        Matrix4x4 operator*(const Matrix4x4 &b);
        void MatrixCout(const char *ch) const;
        void MakeIdentity();
        void MakeNull();
        bool operator == (const Matrix4x4 &m) const
        {
            for (int i = 0; i < 16; i++)
            {
                if (a[i] != m.a[i])
                    return false;
            }
            return true;
        }
        scalar &operator()(const int &row, const int &col)
        {
            return a[row * 4 + col];
        }
        scalar at(const int &row, const int &col) const
        {
            return a[row * 4 + col];
        }
        int MaxElem()
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
        Quaternion ConvertToQuaternion()
        {
            RoboMath::Quaternion res;
            scalar trace = a[0]  + a[5] + a[10] + 1;
            scalar s;
            if (trace > 0)
            {
                s = 0.5 / sqrt(trace);
                res = Quaternion((a[9] - a[6]) * s,
                                 (a[2] - a[8]) * s,
                                 (a[4] - a[1]) * s,
                                  0.25 / s);
            } else {
                switch (MaxElem()){
                case 0:
                    s = sqrt(1.0 + a[0] - a[5] - a[10]) * 2;
                    res = Quaternion((0.5 / s),
                                     (a[1] + a[4]) / s,
                                     (a[2] + a[8]) / s,
                                     (a[6] + a[9]) / s);
                case 1:
                    s = sqrt(1.0 + a[5] - a[0] - a[10]) * 2;
                    res = Quaternion((a[1] + a[4]) / s,
                                     (0.5 / s),
                                     (a[6] + a[9]) / s,
                                     (a[2] + a[8]) / s);
                case 2:
                    s = sqrt(1.0 + a[10] - a[0] - a[5]) * 2;
                    res = Quaternion((a[2] + a[8]) / s,
                                     (a[6] + a[9]) / s,
                                     (0.5 / s),
                                     (a[1] + a[4]) / s);
                }

            }
            return res;
        }
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
        return (m.a[0] + m.a[5] + m.a[10] + m.a[15]);
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
    struct StateVector
    {
        StateVector(scalar &x_in, scalar &y_in, scalar &z_in, scalar &roll_in, scalar &pitch_in, scalar &yaw_in) :
            x(x_in), y(y_in), z(z_in), roll(roll_in), pitch(pitch_in), yaw(yaw_in){};
        StateVector(scalar &&x_in, scalar &&y_in, scalar &&z_in, scalar &&roll_in, scalar &&pitch_in, scalar &&yaw_in) :
            x(x_in), y(y_in), z(z_in), roll(roll_in), pitch(pitch_in), yaw(yaw_in){};
        StateVector(scalar x_in, scalar y_in, scalar z_in, Vector3 &a) :
            x(x_in), y(y_in), z(z_in), roll(a.x), pitch(a.y), yaw(a.z){};
        StateVector(scalar x_in, scalar y_in, scalar z_in, Vector3 &&a) :
            x(x_in), y(y_in), z(z_in), roll(a.x), pitch(a.y), yaw(a.z){};
        StateVector(Vector3 &r, scalar &roll_in, scalar &pitch_in, scalar &yaw_in) :
            x(r.x), y(r.y), z(r.z), roll(roll_in), pitch(pitch_in), yaw(yaw_in){};
        StateVector(Vector3 &r, Vector3 &a) :
            x(r.x), y(r.y), z(r.z), roll(a.x), pitch(a.y), yaw(a.z){};
        Vector3 GetCoord() const
        {
            return Vector3(x, y, z);
        };
        Vector3 GetAngles() const
        {
            return Vector3(roll, pitch, yaw);
        }
        scalar x;
        scalar y;
        scalar z;
        scalar roll;
        scalar pitch;
        scalar yaw;
    };
}
scalar CalcDistancePointToLine(RoboMath::Vector3 p, RoboMath::Vector3 *l1, RoboMath::Vector3 *l2);
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
