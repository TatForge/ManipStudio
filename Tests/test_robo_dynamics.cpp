// Robo: dynamics parameters (link masses, centres of mass, inertia), the
// payload and the joint radii of the collision model -- defaults, settings
// file round trip and validation.

#include "test_helpers.h"

#include "armshape.h"
#include "robo.h"

#include <QSettings>
#include <QTemporaryDir>

#include <memory>
#include <stdexcept>

using namespace RoboMath;
using test_helpers::ScopedChdir;
using test_helpers::ScopedSilence;

namespace {

// the values that were hard-coded in the Solver before they became settings
const scalar kMass[JOINTS_COUNT] = {2.7600, 2.9325, 2.5773, 1.6183, 1.3935, 0.2498};
const scalar kCom[JOINTS_COUNT][3] = {{0, -0.0046, -0.0467}, {0.3251, 0, -0.0204}, {0.2869, 0, 0.0235},
                                      {0.0757, 0, 0.0011},   {-0.0023, 0, 0.0364}, {0.0001, 0, 0.0193}};
const scalar kInertia[JOINTS_COUNT][6] = {{0.0120, 0.0117, 0.0032, 0, 0, 0}, {0.0055, 0.4014, 0.3983, 0, 0, -0.0276},
                                          {0.0050, 0.2629, 0.2596, 0, 0, 0.0228}, {0.0007, 0.0127, 0.0126, 0, 0, 0},
                                          {0.0033, 0.0033, 0.006, 0, 0, 0},      {0.0002, 0.0002, 0.0001, 0, 0, 0}};

void ExpectDefaults(const Robo &r)
{
    const auto &links = r.GetLinkDynamics();
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_DOUBLE_EQ(links[i].mass, kMass[i]) << "link " << i;
        for (int k = 0; k < 3; k++)
            EXPECT_DOUBLE_EQ(links[i].com[k], kCom[i][k]) << "link " << i;
        for (int k = 0; k < 6; k++)
            EXPECT_DOUBLE_EQ(links[i].inertia[k], kInertia[i][k]) << "link " << i;
    }
    // 5 kg at the centre of mass of the last link, as the Solver assumed
    EXPECT_DOUBLE_EQ(r.GetPayload().mass, 5.0);
    for (int k = 0; k < 3; k++)
        EXPECT_DOUBLE_EQ(r.GetPayload().com[k], kCom[5][k]);
    EXPECT_EQ(r.GetJointRadii(), ArmShape::DefaultJointRadii());
}

class RoboDynamicsSettings : public ::testing::Test
{
protected:
    void SetUp() override
    {
        ASSERT_TRUE(dir.isValid());
        // start from the project's settings file
        ASSERT_TRUE(QFile::copy(QString(TEST_PROJECT_DIR) + "ManipulatorSettigns.ini", dir.filePath("ManipulatorSettigns.ini")));
        QFile::setPermissions(dir.filePath("ManipulatorSettigns.ini"), QFile::ReadOwner | QFile::WriteOwner);
        chdir_guard = std::make_unique<ScopedChdir>(dir.path().toStdString());
    }

    QTemporaryDir dir;
    std::unique_ptr<ScopedChdir> chdir_guard;
};

} // namespace

TEST(RoboDynamics, DefaultsAreThePreviouslyHardCodedModel)
{
    Robo r(JOINTS_COUNT);
    ExpectDefaults(r);
}

TEST_F(RoboDynamicsSettings, ProjectSettingsFileHoldsTheDefaults)
{
    Robo r(JOINTS_COUNT);
    ASSERT_TRUE(r.LoadManip());
    ExpectDefaults(r);
}

TEST_F(RoboDynamicsSettings, MissingGroupsKeepTheDefaults)
{
    {
        QSettings ini("ManipulatorSettigns.ini", QSettings::IniFormat);
        ini.remove("dynamics");
        ini.remove("payload");
        ini.remove("shape");
    }
    Robo r(JOINTS_COUNT);
    ASSERT_TRUE(r.LoadManip());
    ExpectDefaults(r);
}

TEST_F(RoboDynamicsSettings, EditedValuesSurviveSaveAndLoad)
{
    Robo r(JOINTS_COUNT);
    ASSERT_TRUE(r.LoadManip());
    Robo::LinkDynamics link{3.5, {{0.1, -0.02, 0.03}}, {{0.01, 0.02, 0.03, 0.001, -0.002, 0.003}}};
    r.SetLinkDynamics(2, link);
    r.SetPayload({7.25, {{0.01, 0.02, 0.08}}});
    r.SetJointRadius(4, 0.052);
    {
        ScopedSilence quiet;
        r.WriteSettingsToFile();
    }
    Robo loaded(JOINTS_COUNT);
    ASSERT_TRUE(loaded.LoadManip());
    const Robo::LinkDynamics &back = loaded.GetLinkDynamics()[2];
    EXPECT_DOUBLE_EQ(back.mass, 3.5);
    for (int k = 0; k < 3; k++)
        EXPECT_DOUBLE_EQ(back.com[k], link.com[k]);
    for (int k = 0; k < 6; k++)
        EXPECT_DOUBLE_EQ(back.inertia[k], link.inertia[k]);
    EXPECT_DOUBLE_EQ(loaded.GetPayload().mass, 7.25);
    EXPECT_DOUBLE_EQ(loaded.GetPayload().com[2], 0.08);
    EXPECT_DOUBLE_EQ(loaded.GetJointRadii()[4], 0.052);
    // the rest stays at the defaults
    EXPECT_DOUBLE_EQ(loaded.GetLinkDynamics()[1].mass, kMass[1]);
    EXPECT_DOUBLE_EQ(loaded.GetJointRadii()[0], ArmShape::DefaultJointRadii()[0]);
}

TEST(RoboDynamics, InvalidValuesAreRejected)
{
    Robo r(JOINTS_COUNT);
    EXPECT_THROW(r.SetPayload({-1, {{0, 0, 0}}}), std::invalid_argument);
    EXPECT_THROW(r.SetLinkDynamics(0, {-0.1, {{0, 0, 0}}, {{0, 0, 0, 0, 0, 0}}}), std::invalid_argument);
    EXPECT_THROW(r.SetJointRadius(1, 0), std::invalid_argument);
    EXPECT_THROW(r.SetJointRadius(JOINTS_COUNT, 0.05), std::out_of_range);
    EXPECT_THROW(r.SetLinkDynamics(-1, r.GetLinkDynamics()[0]), std::out_of_range);
    // a rejected value changes nothing
    EXPECT_DOUBLE_EQ(r.GetPayload().mass, 5.0);
    EXPECT_DOUBLE_EQ(r.GetJointRadii()[1], ArmShape::DefaultJointRadii()[1]);
}

TEST(RoboDynamics, NoPayloadIsAllowed)
{
    Robo r(JOINTS_COUNT);
    r.SetPayload({0, {{0, 0, 0}}});
    EXPECT_DOUBLE_EQ(r.GetPayload().mass, 0.0);
}
