#include "gjkcollision3d.h"

GJKCollision::GJKCollision()
{

}

bool GJKCollision::Tetrahedron(Simplex &simplex, RoboMath::Vector3 &direction)
{
    Vector3 a = simplex.points[0];
    Vector3 b = simplex.points[1];
    Vector3 c = simplex.points[2];
    Vector3 d = simplex.points[3];

    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ad = d - a;
    Vector3 ao =   - a;

    Vector3 abc = ab * ac;
    Vector3 acd = ac * ad;
    Vector3 adb = ad * ab;

    if (SameDirection(abc, ao))
    {
        return Triangle(simplex = {a, b, c}, direction);
    }

    if (SameDirection(acd, ao))
    {
        return Triangle(simplex = {a, c, d}, direction);
    }

    if (SameDirection(adb, ao))
    {
        return Triangle(simplex = {a, d, b}, direction);
    }
    return true;
}

bool GJKCollision::Line(Simplex &simplex, RoboMath::Vector3 &direction)
{
    Vector3 a = simplex.points[0];
    Vector3 b = simplex.points[1];

    Vector3 ab = b - a;
    Vector3 ao =   - a;
    if (SameDirection(ab, ao)) {
        direction = ab * ao * ab;
    }

    else {
        simplex = { a };
        direction = ao;
    }

    return false;
}

bool GJKCollision::Triangle(Simplex &simplex, Vector3 &direction)
{
    Vector3 a = simplex.points[0];
    Vector3 b = simplex.points[1];
    Vector3 c = simplex.points[2];

    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ao = - a;

    Vector3 abc = ab * ac;

    if (SameDirection(abc * ac, ao))
    {
        if (SameDirection(ac, ao))
        {
            simplex = {a, c};
            direction = ab * ao * ab;
        }
        else return Line(simplex = {a, b}, direction);
    }
    else
    {
        if (SameDirection(ab * abc, ao))
        {
            return Line(simplex = {a, b}, direction);
        }
        else
        {
            if (SameDirection(abc, ao))
            {
                direction = abc;
            }
            else
            {
                simplex = {a, c, b};
                direction = -abc;
            }
        }
    }
    return false;
}

RoboMath::Vector3 GJKCollision::GetFarthestPoint(const MeshCollider &object, const RoboMath::Vector3 &direction)
{
    short farthest_point_index(0);
    float max_dist = object.point(0).Dot(direction);
    for (size_t i = 1; i < object.Size(); i++)
    {
        if (object.point(i).Dot(direction) > max_dist)
        {
            max_dist = object.point(i).Dot(direction);
            farthest_point_index = i;
        }
    }
    return object.point(farthest_point_index);
}

Vector3 GJKCollision::Support(const MeshCollider &object1, const MeshCollider &object2, const Vector3 &direction)
{
//    auto k = GetFarthestPoint(object1, direction);
//    auto l = GetFarthestPoint(object2, -direction);
//    auto kl = k - l;
    return GetFarthestPoint(object1, direction) - GetFarthestPoint(object2, -direction);
}

bool GJKCollision::SameDirection(const Vector3 &direction, const Vector3 &vector)
{
    return direction.Dot(vector) > 0;
}

bool GJKCollision::NextSimplex(Simplex &simplex, Vector3 &direction)
{
    switch (simplex.size)
    {
    case 2: return Line       (simplex, direction);
    case 3: return Triangle   (simplex, direction);
    case 4: return Tetrahedron(simplex, direction);
    }
    return false;
}

Simplex &Simplex::operator =(std::initializer_list<Vector3> input)
{
    short i(0);
    for (auto el : input)
    {
        points[i] = el;
        i++;
    }
    return *this;
}
