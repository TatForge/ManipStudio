#include "meshcollider.h"

MeshCollider::MeshCollider()
{

}

MeshCollider::MeshCollider(const std::string &filename)
{
    ReadStl(filename);
}
MeshCollider MeshCollider::MoveMesh(const Quaternion &rotation, const Vector3 &position) const
{
    MeshCollider mc;
    Quaternion buff = rotation;
    mc = *this;
    for (ushort i = 0; i < m_size; i++)
    {
        mc.m_mesh[i] = buff.RotateVector(m_mesh[i]);
    }
    for (ushort i = 0; i < m_size; i++)
    {
        mc.m_mesh[i] = position + mc.m_mesh[i];
    }
    return mc;
}

bool MeshCollider::MoveMesh(const Quaternion &rotation, const Vector3 &position)
{
    for (ushort i = 0; i < m_size; i++)
    {
        m_mesh[i] = rotation.RotateVector(m_mesh[i]);
    }
    for (ushort i = 0; i < m_size; i++)
    {
        m_mesh[i] = position + m_mesh[i];
    }
    return true;
}

bool MeshCollider::ReadStl(const string &filename)
{
    char header_info[80];
    char ch_triangle_numbers[4];
    char ch_data[2];
    float f[12];
    ushort vertex_numbers;
    ifstream mesh_file(filename.c_str(), ios::in | ios::binary);

    if (mesh_file.is_open())
    {
        mesh_file.read(header_info, 80);
    }
    else
        throw std::runtime_error("can't open mesh " + std::string(filename) + char('"'));

    if (mesh_file.is_open())
    {
        mesh_file.read(ch_triangle_numbers, 4);
        vertex_numbers = 3 * *((unsigned int*) ch_triangle_numbers);
        m_size = vertex_numbers;
//        cout << "number of triangles = " << vertex_numbers << endl;

        try {
            m_mesh.resize(vertex_numbers);
        }  catch (std::bad_alloc &exception) {
            cout << "can't allocate memory for mesh" << endl;
        }
        if (vertex_numbers > 200)
        {
//            cout << " number of vertexes more than 200" << endl;
        }
    }
    for (uint i = 0; i < m_size; i += 3)
    {
        mesh_file.read((char*)f, 48);
        m_mesh[i] = Vector3(f[3], f[4], f[5]);
        m_mesh[i + 1] = Vector3(f[6], f[7], f[8]);
        m_mesh[i + 2] = Vector3(f[9], f[10], f[11]);
        mesh_file.read(ch_data, 2);
    }
    if (RemoveDublicateVertexes())
        return true;
    return false;
}

bool MeshCollider::RemoveDublicateVertexes()
{
//    for (uint i = 0; i < m_size; i++)
//    {
//        cout << m_mesh[i].x << " " << m_mesh[i].y << " " << m_mesh[i].z << endl;
//    }
    if (m_size == 0 || m_mesh.empty())
    {
        m_mesh.clear();
        m_size = 0;
        return true;
    }
    sort(m_mesh.begin(), m_mesh.end());
//    for (uint i = 0; i < m_size; i++)
//    {
//        cout << m_mesh[i].x << " " << m_mesh[i].y << " " << m_mesh[i].z << endl;
//    }
    vector<Vector3> result;
    for (uint i = 0; i < m_size - 1; i++)
    {
//        cout << m_mesh[i].x << " " << m_mesh[i].y << " " << m_mesh[i].z << endl;
        if (m_mesh[i] != m_mesh[i + 1])
        {
            result.push_back(m_mesh[i]);
        }
    }
    result.push_back(m_mesh[m_size - 1]);
    m_mesh = result;
    m_size = result.size();
//    for (uint i = 0; i < result.size(); i++)
//    {
//        cout << result[i].x << " " << result[i].y << " " << result[i].z << endl;
//    }

//    for (uint i = 0; i < m_mesh.size(); i++)
//    {
//        cout <<"№ " << i << " " << m_mesh[i].x << " " << m_mesh[i].y << " " << m_mesh[i].z << endl;
//    }
//    cout << m_mesh.size() << endl;
    return true;
}

void MeshCollider::operator=(const MeshCollider &mc)
{
    m_size = mc.m_size;
    m_mesh.resize(mc.m_size);
    for (uint i = 0; i < m_size; i++)
    {
        m_mesh[i] = mc.m_mesh[i];
    }
}

ushort MeshCollider::Size() const
{
    return m_size;
}

Vector3 MeshCollider::point(const ushort &elem) const
{
    return m_mesh.at(elem);
}

bool MeshCollider::CheckCollision(const MeshCollider &mesh)
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


bool MeshCollider::Tetrahedron(Simplex &simplex, RoboMath::Vector3 &direction)
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

bool MeshCollider::Line(Simplex &simplex, RoboMath::Vector3 &direction)
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

bool MeshCollider::Triangle(Simplex &simplex, Vector3 &direction)
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
            direction = ac * ao * ac;
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

RoboMath::Vector3 MeshCollider::GetFarthestPoint(const MeshCollider &object, const RoboMath::Vector3 &direction)
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

Vector3 MeshCollider::Support(const MeshCollider &object2, const Vector3 &direction)
{
    return GetFarthestPoint(*this, direction) - GetFarthestPoint(object2, -direction);
}

bool MeshCollider::SameDirection(const Vector3 &direction, const Vector3 &vector)
{
    return direction.Dot(vector) > 0;
}

bool MeshCollider::NextSimplex(Simplex &simplex, Vector3 &direction)
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
    size = i;
    return *this;
}

void Simplex::PushFront(const Vector3 &point)
{
    for (ushort i = RoboMath::min(size, ushort(3)); i > 0; i--)
    {
        points[i] = points[i - 1];
    }
    points[0] = point;
    size = std::min(size + 1, 4);
}
