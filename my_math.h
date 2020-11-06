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
    class Vector
    {
    public:
        Vector()
        {
            length = 0;
            data == nullptr;
        };
        Vector(int size)
        {
            length = size;
            dataNewMemory(length);
        }
        Vector(initializer_list<scalar> il)
        {
            length = il.size();
            dataNewMemory(length);
            initializer_list<scalar>::iterator it = il.begin();
            for (int i = 0; i < length; i++)
            {
                *(data + i) = *it++;
            }
        }
        ~Vector()
        {
            delete [] data;
        }

        int GetLength() const {return length;};
        scalar* GetDataPtr () const {return data;};
        scalar& operator()(counter number);
        Vector& operator= (const Vector &V);
//        friend ostream &operator<< (ostream& os, const Vector &V);
    private:
        int length;
        scalar *data;
        void dataNewMemory(const int &length)
        {
            data = new scalar [length];
        }
    };


    class Matrix
    {
    public:
        Matrix()
        {
            rows = 0;
            cols = 0;
            data = nullptr;
        }
        Matrix(const int &rows, const int &cols)
        {
            this->rows = rows;
            this->cols = cols;
            DataNewMemory(cols, rows);
        }
        Matrix(const Matrix& source)
        {
            rows = source.rows;
            cols = source.cols;
            DataNewMemory();
            memcpy(data, source.data, sizeof(scalar) * rows * cols);
        }
        Matrix(const initializer_list<initializer_list<scalar>> &il)
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
        Matrix(const initializer_list<scalar> &il)
        //Only for square matrix
        {
            int i = 0;
            rows = sqrt(il.size());
            cols = sqrt(il.size());
            DataNewMemory(cols, rows);
            copy(il.begin(), il.end(), data);
        }
        ~Matrix()
        {
    //        if(data != nullptr)
    //            delete[] data;
        }
        scalar &elem(const int &row, const int &col) const;

        Matrix Multuply(const Matrix &matrix1, const Matrix &matrix2);
        void AddDiagonal(const scalar &d);
        void SetSubmatrix(const Matrix &source);
        void SetColumn(Vector &v, int ColNumber);
        void SolveGauss();      //Решение СЛАУ методом Гаусса (приведение к ступенчатому виду)
        void GetTheRoots(Vector &V);     //Получение корней уравнения
        Matrix operator *(const Matrix &matrix2);
        Vector operator *(const Vector &vector2);

        Matrix operator =(const Matrix &matrix1);

        scalar& operator()(const int &row, const int &col)
        {
            assert(col >= 0 && col <= this->cols);
            assert(row >= 0 && row <= this->rows);
            return *(data + index(row, col));
        }
        void transpose();
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
