#ifndef MY_MATH_H
#define MY_MATH_H


#include <initializer_list>
#include <vector>
#include <iostream>
#include <iomanip>
#include <memory>
#include <cstring>
#include <chrono>
#include <assert.h>
#include <constants.h>



typedef unsigned short counter;
using namespace std;

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
        Angles();
        bool IsNull() const;
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
        scalar GetAngle(const int &joint_number, const scalar &t_m) const;
        scalar GetSpeed(const int &joint_number, const scalar &t_m) const;
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
        Vector(const Vector &v);
        ~Vector();

        int GetLength() const {return m_length;};
        scalar MaxAbs();
        scalar* GetDataPtr() const {return m_data;};
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
        int m_length;
        scalar *m_data;
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
        Vector3(const Vector3 &v) : x(v.x), y(v.y), z(v.z) {};
        void Rotate(const scalar &theta, const RoboMath::Vector3 &V);
        void Clear();
        scalar  Length()    const {return sqrt(x * x + y * y + z * z);};
        scalar Dot(const Vector3 &v) const;

        Vector3 Normalize() const;
        Vector3 operator= (const Vector &V);
        Vector3 operator= (const Vector3 &V);
        Vector3 operator- (const Vector3 &V) const;
        Vector3 operator- () const;
        Vector3 operator+ (const Vector3 &V) const;
        Vector3 operator* (const Vector3 &V) const;
        Vector3 &operator+= (const Vector3 &V);
        Vector3 &operator* (const scalar &value);
        bool operator< (Vector3 &v);
        bool operator!= (Vector3 &v)
        {
            return (x != v.x) || (y != v.y) || (z != v.z);
        }
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
        ~Matrix();
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
        void DataNewMemory();
        void DataNewMemory(const int &rows, const int &cols);
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
        Vector3 RotateVector(const Vector3 &v) const;
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
        Matrix4x4();
        Matrix4x4(const initializer_list<scalar> il);
        void operator=(const Matrix &m);
        void operator*=(const scalar &sc);
        Matrix4x4 MultiplyTranspose(const Matrix4x4 &b);
        Vector MultiplyTranspose(const Vector &v);
        Matrix4x4 operator*(const Matrix4x4 &b);
        void MatrixCout(const char *ch) const;
        void MakeIdentity();
        void MakeNull();
        bool operator == (const Matrix4x4 &m) const;
        scalar &operator()(const int &row, const int &col);
        scalar at(const int &row, const int &col) const;
        int MaxElem() const;
        Quaternion ConvertToQuaternion() const;
        Vector3 ConvertToVector3() const;
    };
    class MatrixOfMatrix6x6
    {
    public:
        MatrixOfMatrix6x6();
        Matrix& operator()(const int &row, const int &col);
    private:
        counter rows, cols;
        Matrix *data = new Matrix[36];
    };

    const Vector3 unit_x(1.0, 0.0, 0.0);
    const Vector3 unit_y(0.0, 1.0, 0.0);
    const Vector3 unit_z(0.0, 0.0, 1.0);
    const Vector3 unit_mz(0.0, 0.0, -1.0);


    struct StateVector
    {
        StateVector() : x(0), y(0), z(0), roll(0), pitch(0), yaw(0){};
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
        Vector3 GetCoord() const;
        Vector3 GetAngles() const;
        scalar x;
        scalar y;
        scalar z;
        scalar roll;
        scalar pitch;
        scalar yaw;
    };

    template<typename T> inline static T min(T t1, T t2) {return (t1 < t2 ? t1 : t2);}
    template<typename T> inline static T max(T t1, T t2) {return (t1 > t2 ? t1 : t2);}
    template<typename T> inline static T interpolate(T t1, T t2, scalar percent){
        if (percent == 0)
            return t1;
        else
        {
            return t1 + (t2 - t1) * percent;
        }
    }
    template<typename T> static bool sign(T t1, T t2)
    {
        if ((t1 < 0 && t2 < 0) || (t1 > 0 && t2 > 0))
            return true;
        else if ((t1 < 0 && t2 > 0) || (t2 > 0 && t1 < 0))
            return false;
        return true;
    }
    const Matrix4x4 q ({0, -1, 0, 0,
                        1,  0, 0, 0,
                        0,  0, 0, 0,
                        0,  0, 0, 0});
    const Matrix4x4 q_sq ({-1,   0, 0, 0,
                            0,  -1, 0, 0,
                            0,   0, 0, 0,
                            0,   0, 0, 0});
}
scalar Trace(const RoboMath::Matrix4x4 &m);

scalar Trace(const RoboMath::Matrix &ma);

scalar Distance(const RoboMath::StateVector &v1, const RoboMath::StateVector &v2);
scalar CalcDistancePointToLine(RoboMath::Vector3 p, const RoboMath::Vector3 *l1, const RoboMath::Vector3 *l2);
class checkTime
{
public:
    checkTime();
    ~checkTime();
    std::chrono::duration<double> duration();

private:
    std::chrono::duration<double> d;
    std::chrono::_V2::system_clock::time_point start;
    std::chrono::_V2::system_clock::time_point stop;
    bool checked;
};

//class Mesh
//{
//public:
//    Mesh()
//    {
//        m_points = nullptr;
//        m_size = 0;
//    }
//    Mesh(const RoboMath::Vector3 *points, const size_t size)
//    {
//        m_size = size;
//        try
//        {
//            m_points = new RoboMath::Vector3 [size];
//        }
//        catch (std::bad_alloc & exception)
//        {
//            cout << "Can't allocate memory for mesh" << endl;
//        }

//        mempcpy(m_points, points, size * sizeof(RoboMath::Vector3));
//    }
//    RoboMath::Vector3* point(unsigned int element) const
//    {
//        return m_points + element;
//    }
//    unsigned int Size() const
//    {
//        return m_size;
//    }

//private:
//    RoboMath::Vector3 *m_points;
//    unsigned int m_size;//points count
//};

#endif // MY_MATH_H
