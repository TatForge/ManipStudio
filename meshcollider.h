#ifndef MESHCOLLIDER_H
#define MESHCOLLIDER_H

#include <iostream>
#include <fstream>
#include "my_math.h"

using namespace std;
using namespace RoboMath;
//struct vertex
//{
//    vertex() : x(0), y(0), z(0){};
//    vertex(const float &x_in, const float &y_in, const float &z_in) : x(x_in), y(y_in), z(z_in){};
//    vertex(char *point)
//    {
//        char f1[4] = {point[0], point[1], point[2], point[3]};
//        char f2[4] = {point[4], point[5], point[6], point[7]};
//        char f3[4] = {point[8], point[9], point[10], point[11]};

//        x = *((char*)f1);
//        y = *((char*)f2);
//        z = *((char*)f3);
//    };
//    float x;
//    float y;
//    float z;
//};

class Triangle
{
public:
    Triangle()
    {
    }
    Vector3 v1;
    Vector3 v2;
    Vector3 v3;
//    float normal[3];
};

class MeshCollider
{
public:
    MeshCollider();
    bool MoveMesh(Quaternion &rotation, Vector3 &position)
    {
        for (int i = 0; i < size; i++)
        {
            mesh_ptr[i].v1 = rotation.RotateVector(mesh_ptr[i].v1);
            mesh_ptr[i].v2 = rotation.RotateVector(mesh_ptr[i].v2);
            mesh_ptr[i].v3 = rotation.RotateVector(mesh_ptr[i].v3);
        }
    }
    bool read_stl(std::string filename)
    {
        char header_info[80];
        char ch_triangle_numbers[4];
        char ch_data[2];
        float f[12];
        unsigned long triangle_numbers;
        ifstream mesh_file(filename.c_str(), ios::in | ios::binary);

        if (mesh_file.is_open())
        {
            mesh_file.read(header_info, 80);
            cout << "header: " << header_info << endl;
        }
        else
            cout << "cant open mesh" << endl;

        if (mesh_file.is_open())
        {
            mesh_file.read(ch_triangle_numbers, 4);
            triangle_numbers = *((unsigned int*) ch_triangle_numbers);
            size = triangle_numbers;
            cout << "number of triangles = " << triangle_numbers << endl;

            try {
                mesh_ptr = new Triangle[triangle_numbers];
            }  catch (std::bad_alloc &exception) {
                cout << "can't allocate memory for mesh" << endl;
            }
            if (triangle_numbers > 200)
                cout << " number of vertexes more than 150" << endl;
        }
        for (int i = 0; i < triangle_numbers; i++)
        {
            mesh_file.read((char*)f, 48);
            mesh_ptr[i].v1 = Vector3(f[3], f[4], f[5]);
            mesh_ptr[i].v2 = Vector3(f[6], f[7], f[8]);
            mesh_ptr[i].v3 = Vector3(f[9], f[10], f[11]);
            mesh_file.read(ch_data, 2);
        }
        return true;
    }
private:
    Triangle *mesh_ptr;
    unsigned int size;
};

#endif // MESHCOLLIDER_H
