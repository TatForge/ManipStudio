// Console check of the core: loads the model from ManipulatorSettigns.ini and
// STLmodels/ (working directory), plans a MoveJ from the zero to the folded
// pose and prints a summary. The GUI is ManipStudio (Studio/).
#include "robo.h"
#include "solver.h"

#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#if ((!WITH_BENCHMARK) && (!WITH_GTESTS))
int main()
{
    Robo manipulator(JOINTS_COUNT);
    try
    {
        if (!manipulator.LoadManip())
        {
            std::cerr << "cannot read ManipulatorSettigns.ini" << std::endl;
            return 1;
        }
    }
    catch (const std::string &exc)
    {
        std::cerr << exc << std::endl;
        return 1;
    }

    Solver solver(manipulator);
    const Angles from = solver.GetZeroPosition();
    const Angles to = solver.GetInternalPosition();
    std::vector<float> trajectory;
    try
    {
        solver.MoveJ(from, to, 1.0f, 2.0f, DT_MILLISEC, trajectory);
    }
    catch (const std::exception &exc)
    {
        std::cerr << "MoveJ failed: " << exc.what() << std::endl;
        return 1;
    }

    const size_t samples = trajectory.size() / (sizeof(Angles) / sizeof(scalar));
    std::cout << "MoveJ zero -> folded: " << samples << " samples, " << (samples - 1) * DT_MILLISEC / 1000.0 << " s" << std::endl;
    const std::vector<Robo::Moments> moments = solver.GetTrajectoryMoments();
    const std::vector<scalar> limits = manipulator.GetMaxJointMoments();
    for (int j = 0; j < JOINTS_COUNT; j++)
    {
        scalar peak = 0;
        for (const Robo::Moments &m : moments)
            peak = std::max(peak, std::abs(m.tao[j]));
        std::cout << "  q" << j + 1 << ": peak |tau| = " << peak << " N*m (limit " << limits[j] << ")" << std::endl;
    }
    return 0;
}
#endif
