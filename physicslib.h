#ifndef PHYSICSLIB_H
#define PHYSICSLIB_H
#include <vector>

using namespace std;
typedef vector<float> fvector;

struct LibData;

class PhysicsLib
{
public:
    bool m_usable;
    PhysicsLib();
    /**
     * @brief Initialize - create Solver and manipulator clarr, throws exceptions
     */
    void Initialize();
    void MoveJ(const fvector &start_angle, const fvector &end_angle, const float &v_tcp, const float &a_tcp, fvector &results);
    void MoveC(const fvector &start_angle, const fvector &p1, const fvector &p2, const fvector &p3, const float &v_tcp, const float &a_tcp, fvector &results);
    void MoveL(const fvector &start_angle, const fvector &state_vector_p1, const fvector &state_vector_p2, const float &v_tcp, const float &a_tcp, fvector &results);
    /**
    * @brief Convert manipulator's angle position to tcp position
    *  angle vector[18] = Angles q[6],
      vector[16] = matrix [4x4]
    * @param &angle vector[18] = Angles q[6], q_p[6], q_pp[6]
    * @param &T     vector[16] = matrix [4x4]
    */
    void DirectKinematic(const fvector &angle, fvector &T);
    /**
     * @brief InverseKinematic - convert target tcp position to joint angles
     * @param target - target tcp position StateVector point (x,y,z,roll,pitch,yaw)
     * @param curr_angle - 6 angles of manipulator joints
     * @param res_angle - 6 angles of manipulator joints after InverseKinematics calculation
     */
    void InverseKinematic(const fvector &target, const fvector &curr_angle, fvector &res_angle);
private:
    LibData *lib_data;
};

#endif // PHYSICSLIB_H
