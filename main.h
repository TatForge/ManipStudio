#ifndef MAIN_H
#define MAIN_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <my_math.h>

using namespace std;

class Elem
{
public :
    Elem(const double &xpos, const double &ypos, const double &zpos, int *rotationAxis);
    Elem() : x_pos(0), y_pos(0), z_pos(0), rotation_axis{0, 0, 0}{};

    void elemCout()
    {
        cout << "X = " << x_pos << " Y = " << y_pos << " Z = " << z_pos << "\n"
             << "Rotation Axis " << rotation_axis[0] << rotation_axis[1] << rotation_axis[2] << endl;
    }

    double getX(){return x_pos;};
    double getY(){return y_pos;};
    double getZ(){return z_pos;};
    double getOrX(){return rotation_axis[0];};
    double getOrY(){return rotation_axis[1];};
    double getOrZ(){return rotation_axis[2];};

    Eigen::Vector3i RA(){return rA;};
    void RAInit()
    {
        rA(0) = rotation_axis[0];
        rA(1) = rotation_axis[1];
        rA(2) = rotation_axis[2];
    }

private:
    double x_pos;
    double y_pos;
    double z_pos;
    int rotation_axis [3];
    Eigen::Vector3i rA;
};

#endif // MAIN_H
