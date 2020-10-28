#include <initializer_list>
#include <vector>
#include <cmath>
#include <iostream>
#include <memory>
#include <cstring>
#include <chrono>

#ifndef MY_MATH_H
#define MY_MATH_H

#define DOUBLE_SCALAR
//#define DEBUG_COUT

#ifdef DOUBLE_SCALAR
typedef double scalar;
#else
typedef float scalar;
#endif


using namespace std;

class matrix
{
public:
    matrix()
    {
        rows = 0;
        cols = 0;
        data = nullptr;
    }
    matrix(const int &rows, const int &cols)
    {
        this->rows = rows;
        this->cols = cols;
        dataNewMemory(cols, rows);
    }
    matrix(const initializer_list<initializer_list<scalar>> &il)
    {
//        int il_sec;
        rows = il.size();
        cols = il.begin()->size();
        dataNewMemory(cols, rows);

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
    matrix(const initializer_list<scalar> &il)
    //Only for square matrix
    {
        int i = 0;
        rows = sqrt(il.size());
        cols = sqrt(il.size());
        dataNewMemory(cols, rows);
        copy(il.begin(), il.end(), data);
    }
    ~matrix()
    {
//        if(data != nullptr)
//            delete[] data;
    }
    scalar &elem(const int &row, const int &col) const
    {
        return *(data + index(row, col));
    }

    static matrix multuply(const matrix &matrix1, const matrix &matrix2);
    matrix operator *(const matrix &matrix2);

    matrix operator =(const matrix &matrix1);

    int getColsCount() const { return cols; }
    int getRowsCount() const { return rows; }
    void matrixCout()
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                cout << *(data + i * cols + j) << "\t";
            }
            cout << endl;
        }
    }
private:
    int rows;
    int cols;
    scalar *data;
    void dataNewMemory()
    {
        data = new scalar[rows*cols];
    }
    void dataNewMemory(const int &cols, const int &rows)
    {
        data = new scalar[rows*cols];
#ifdef DEBUG_COUT
        cout << "Data matrix was initialized with rows = "
             << rows << " end cols = " << cols << endl;
        cout << endl;
#endif
    }
    int index(const int &row, const int &col) const { return row * cols + col; }
};

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


#endif // MY_MATH_H
