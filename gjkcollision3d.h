#ifndef GJKCOLLISION_H
#define GJKCOLLISION_H

#include "my_math.h"
//#include "meshcollider.h"

using namespace RoboMath;

struct Simplex
{
    Vector3 points[4];
    ushort size = 0;
    Simplex& operator = (std::initializer_list<Vector3> input);
    void PushFront(const Vector3 &point)
    {
        for (unsigned short i = RoboMath::min(size, ushort(4)); i > 0; i--)
        {
            points[i] = points[i - 1];
        }
        points[0] = point;
        size = std::min(size + 1, 4);
    }
};

class GJKCollision
{
public:
    GJKCollision();
    Vector3 GetFarthestPoint(const MeshCollider &object, const Vector3 &direction);
    Vector3 Support(const MeshCollider &object1, const MeshCollider &object2, const Vector3 &direction);
    bool SameDirection(const Vector3 &direction, const Vector3 &vector);
    bool NextSimplex(Simplex &simplex, Vector3 &direction);
    bool Line(Simplex &simplex, Vector3 &direction);
    bool Triangle(Simplex &simplex, Vector3 &direction);
    bool Tetrahedron(Simplex &simplex, Vector3 &direction);

    bool CheckCollision(const MeshCollider &mesh1, const MeshCollider &mesh2)
    {
        Vector3 support = Support(mesh1, mesh2, Vector3(1, 0, 0));
        Simplex simplex;
        simplex.PushFront(support);
        Vector3 direction = - support;
        while (true)
        {
            support = Support(mesh1, mesh2, direction);
            //if <= than two objects not collide if one point of them in one place(like obj1 point in (2,2) &
            //obj2 point in (2,2)
            if (support.Dot(direction) <= 0)
                return false;
            simplex.PushFront(support);
            if (NextSimplex(simplex, direction))
                return true;
        }
    }
};

#endif // GJKCOLLISION_H
