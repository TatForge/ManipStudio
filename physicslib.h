#ifndef PHYSICSLIB_H
#define PHYSICSLIB_H
#include <vector>
using namespace std;
typedef vector<float> fvector;

struct LibData;

class PhysicsLib
{
public:
    PhysicsLib();
    void MoveJ(const fvector &start_angle, const fvector &end_angpe, const float &v_tcp, const float &a_tcp, fvector &results);
    void MoveC(const fvector &start_angle, const fvector &p1, const fvector &p2, const fvector &p3, const float &v_tcp, const float &a_tcp, fvector &results);
    void MoveL(const fvector &start_angle, const fvector &state_vector_p1, const fvector &state_vector_p2, const float &v_tcp, const float &a_tcp, fvector &results);

private:
    LibData *lib_data;
};

#endif // PHYSICSLIB_H
