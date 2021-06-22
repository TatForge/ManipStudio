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
    Simplex& operator = (std::initializer_list<Vector3> input);
    void PushFront(const Vector3 &point);
    Vector3 points[4];
    ushort size = 0;
};

class MeshCollider
{
public:
    MeshCollider();
    MeshCollider(const std::string &filename);
    MeshCollider MoveMesh(const Quaternion &rotation, const Vector3 &position) const;
    bool MoveMesh(const Quaternion &rotation, const Vector3 &position);
    bool CheckCollision(const MeshCollider &mesh);
    bool ReadStl(const std::string &filename);
    Vector3 point(const ushort &elem) const;
    void operator= (const MeshCollider &mc);
    bool RemoveDublicateVertexes();
    ushort Size() const;
private:
    vector<Vector3> m_mesh;
    ushort m_size;
    Vector3 GetFarthestPoint(const MeshCollider &object, const Vector3 &direction);
    Vector3 Support(const MeshCollider &object2, const Vector3 &direction);
    bool SameDirection(const Vector3 &direction, const Vector3 &vector);
    bool Tetrahedron(Simplex &simplex, Vector3 &direction);
    bool NextSimplex(Simplex &simplex, Vector3 &direction);
    bool Triangle(Simplex &simplex, Vector3 &direction);
    bool Line(Simplex &simplex, Vector3 &direction);
};


#endif // MESHCOLLIDER_H
