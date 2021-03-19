#ifndef GJKCOLLISION_H
#define GJKCOLLISION_H

#include <my_math.h>

using namespace RoboMath;

struct Simplex
{
    Vector3 points[4];
    unsigned short size;
    Simplex& operator = (std::initializer_list<Vector3> input);
    void PushFront(const Vector3 &point)
    {
        for (unsigned short i = 1; i < size + 1; i++)
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
    Vector3 GetFarthestPoint(const Mesh &object, const Vector3 &direction);
    Vector3 Support(const Mesh &object1, const Mesh &object2, const Vector3 &direction);
    bool SameDirection(const Vector3 &direction, const Vector3 &vector);
    bool NextSimplex(Simplex &simplex, Vector3 &direction);
    bool Line(Simplex &simplex, Vector3 &direction);
    bool Triangle(Simplex &simplex, Vector3 &direction);
    bool Tetrahedron(Simplex &simplex, Vector3 &direction);

    bool GJK(const Mesh &mesh1, const Mesh &mesh2)
    {
        Vector3 support = Support(mesh1, mesh2, Vector3(1, 0, 0));
        Simplex simplex;
        simplex.PushFront(support);
        Vector3 direction = - support;
        while (true)
        {
            support = Support(mesh1, mesh2, direction);
            if (support.Dot(direction) <= 0)
                return false;
            simplex.PushFront(support);
            if (NextSimplex(simplex, direction))
                return true;
        }
    }
};

#endif // GJKCOLLISION_H
