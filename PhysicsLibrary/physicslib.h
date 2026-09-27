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
    ~PhysicsLib();
    /**
     * @brief Initialize - create Solver and manipulator clarr, throws exceptions
     */
    void Initialize();
    /**
     * @brief MoveJ - Generate trajectory created by joints movement;
     * @param start_angle - std::vector<float> [18] mirror of Angles structure (q[6], q_p[6], q_pp[6]) of start angle;
     * @param end_angle - std::vector<float> [18] mirror of Angles structure (q[6], q_p[6], q_pp[6]) of end angle;
     * @param v_tcp - move speed of each joint, radians/sec;
     * @param a_tcp - acceleration of each joint, rad/sec^2;
     */
    void MoveJ(const fvector &start_angle, const fvector &end_angle, const float &v_joints, const float &a_joints, fvector &results);
    /**
     * @brief MoveC - Create circle trajectory based on 3 points;
     * @param start_angle - std::vector<float> [18] mirror of Angles structure (q[6], q_p[6], q_pp[6]) of start angle;
     * @param p1 - std::vector<float> [6] mirror of StateVector structure (x, y, z, roll, pitch, yaw) of first point;
     * @param p2 - std::vector<float> [6] mirror of StateVector structure (x, y, z, roll, pitch, yaw) of second point;
     * @param p3 - std::vector<float> [6] mirror of StateVector structure (x, y, z, roll, pitch, yaw) of third point;
     * @param v_tcp - move speed of TCP;
     * @param a_tcp - acceleration of TCP;
     * @return std::vector<float> size * [18](q, q_p, q_pp)
     */
    void MoveC(const fvector &start_angle, const fvector &p1, const fvector &p2, const fvector &p3, const float &v_tcp, const float &a_tcp, fvector &results);
    /**
     * @brief MoveL - Generate line trajectory;
     * @param start_angle - std::vector<float> [18] mirror of Angles structure (q[6], q_p[6], q_pp[6]) of start angle;
     * @param p1 - std::vector<float> [6] mirror of StateVector structure (x, y, z, roll, pitch, yaw) of first point;
     * @param p2 - std::vector<float> [6] mirror of StateVector structure (x, y, z, roll, pitch, yaw) of second point;
     * @param v_tcp - move speed of TCP;
     * @param a_tcp - acceleration of TCP;
     */
    void MoveL(const fvector &start_angle, const fvector &p1, const fvector &p2, const float &v_tcp, const float &a_tcp, fvector &results);
    /**
    * @brief Convert manipulator's angle position to TCP position;
    * @param &angle std::vector<float> [18] mirror of Angles structure (q[6], q_p[6], q_pp[6]) of angle, which need to be recalculated to the TCP position;
    * @param &T     std::vector<float> [6] - result of angle calculation;
    */
    void DirectKinematic(const fvector &angle, fvector &T);
    /**
     * @brief InverseKinematic - convert target TCP position to joint angles;
     * @param target - std::vector<float> [6] mirror of StateVector structure (x, y, z, roll, pitch, yaw) of target TCP position;
     * @param curr_angle - std::vector<float> [18] mirror of Angles structure (q[6], q_p[6], q_pp[6]) of current angle;
     * @param res_angle - std::vector<float> [18] mirror of Angles structure (q[6], q_p[6], q_pp[6]) of calcilated angle;
     */
    void InverseKinematic(const fvector &target, const fvector &curr_angle, fvector &res_angle);
    fvector GetExternalPosition();
    fvector GetInternalPosition();
private:
    LibData *lib_data;
};

#endif // PHYSICSLIB_H
