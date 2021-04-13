#ifndef MESHCOLLIDER_H
#define MESHCOLLIDER_H

#include <iostream>
#include <fstream>
#include <algorithm>
#include "my_math.h"

using namespace std;
using namespace RoboMath;

struct Simplex
{
    Vector3 points[4];
    ushort size = 0;
    Simplex& operator = (std::initializer_list<Vector3> input);
    void PushFront(const Vector3 &point);
};

class MeshCollider
{
public:
    MeshCollider();
    MeshCollider(const std::string &filename);
    MeshCollider MoveMesh(const Quaternion &rotation, const Vector3 &position) const;
    bool MoveMesh(const Quaternion &rotation, const Vector3 &position);
    bool ReadStl(const std::string &filename);
    bool RemoveDublicateVertexes();
    void operator= (const MeshCollider &mc);
    ushort Size() const;
    Vector3 point(const ushort &elem) const;
    bool CheckCollision(const MeshCollider &mesh)
    {
//        cout << "Check collision started" << endl;
        Vector3 support = Support(mesh, Vector3(1, 0, 0));
        Simplex simplex;
        simplex.PushFront(support);
        Vector3 direction = - support;
        while (true)
        {
            support = Support(mesh, direction);
            //if <= than two objects not collide if one point of them in one place(like obj1 point in (2,2) &
            //obj2 point in (2,2)
            if (support.Dot(direction) < 0)
            {
//                cout << "Check collision ended" << endl;
                return false;
            }
            simplex.PushFront(support);
            if (NextSimplex(simplex, direction))
            {
//                cout << "Check collision ended" << endl;
                return true;
            }
        }
    }
private:
    vector<Vector3> m_mesh;
    ushort m_size;
    Vector3 GetFarthestPoint(const MeshCollider &object, const Vector3 &direction);
    Vector3 Support(const MeshCollider &object2, const Vector3 &direction);
    bool SameDirection(const Vector3 &direction, const Vector3 &vector);
    bool NextSimplex(Simplex &simplex, Vector3 &direction);
    bool Line(Simplex &simplex, Vector3 &direction);
    bool Triangle(Simplex &simplex, Vector3 &direction);
    bool Tetrahedron(Simplex &simplex, Vector3 &direction);
};


#endif // MESHCOLLIDER_H
