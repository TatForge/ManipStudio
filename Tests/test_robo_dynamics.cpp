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

TEST_F(RoboDynamicsSettings, DhTableAndMaxMomentsAreEditableAndSurviveSaveAndLoad)
{
    Robo r(JOINTS_COUNT);
    ASSERT_TRUE(r.LoadManip());
    const Robo::DHParams before = r.GetDH(1);
    EXPECT_DOUBLE_EQ(before.a, 0.375);
    EXPECT_DOUBLE_EQ(before.alpha, 0);
    EXPECT_DOUBLE_EQ(before.d, 0.1);
    r.SetDH(1, {0.4, 0.01, 0.11});
    r.SetMaxJointMoment(3, 95.5);
    {
        ScopedSilence quiet;
        r.WriteSettingsToFile();
    }
    Robo loaded(JOINTS_COUNT);
    ASSERT_TRUE(loaded.LoadManip());
    EXPECT_DOUBLE_EQ(loaded.GetDH(1).a, 0.4);
    EXPECT_DOUBLE_EQ(loaded.GetDH(1).alpha, 0.01);
    EXPECT_DOUBLE_EQ(loaded.GetDH(1).d, 0.11);
    EXPECT_DOUBLE_EQ(loaded.GetDH(2).a, 0.295);
    EXPECT_DOUBLE_EQ(loaded.GetMaxJointMoments()[3], 95.5);
}

TEST(RoboDynamics, InvalidDhAndMomentsAreRejected)
{
    Robo r(JOINTS_COUNT);
    EXPECT_THROW(r.SetDH(0, {std::nan(""), 0, 0}), std::invalid_argument);
    EXPECT_THROW(r.SetDH(JOINTS_COUNT, {0, 0, 0}), std::out_of_range);
    EXPECT_THROW(r.SetMaxJointMoment(0, 0), std::invalid_argument);
    EXPECT_THROW(r.SetMaxJointMoment(-1, 10), std::out_of_range);
}

TEST(RoboSpeedLimits, DefaultsAreSet)
{
    Robo r(JOINTS_COUNT);
    const auto v = r.GetMaxJointSpeeds();
    const auto a = r.GetMaxJointAccelerations();
    const scalar v_deg[JOINTS_COUNT] = {180, 180, 180, 360, 360, 360};
    const scalar a_deg[JOINTS_COUNT] = {360, 360, 360, 720, 720, 720};
    for (int i = 0; i < JOINTS_COUNT; i++)
    {
        EXPECT_NEAR(v[i], ToRad(v_deg[i]), 1e-12) << "joint " << i;
        EXPECT_NEAR(a[i], ToRad(a_deg[i]), 1e-12) << "joint " << i;
    }
}

TEST_F(RoboDynamicsSettings, SpeedLimitsSurviveSaveAndLoad)
{
    Robo r(JOINTS_COUNT);
    ASSERT_TRUE(r.LoadManip());
    r.SetMaxJointSpeed(2, 1.25);
    r.SetMaxJointAcceleration(5, 7.5);
    {
        ScopedSilence quiet;
        r.WriteSettingsToFile();
    }
    {
        // stored in degrees next to the angle limits
        QSettings ini("ManipulatorSettigns.ini", QSettings::IniFormat);
        EXPECT_NEAR(ini.value("joint limits/2/max_speed_deg_s").toDouble(), ToDeg(1.25), 1e-9);
        EXPECT_NEAR(ini.value("joint limits/5/max_accel_deg_s2").toDouble(), ToDeg(7.5), 1e-9);
    }
    Robo loaded(JOINTS_COUNT);
    ASSERT_TRUE(loaded.LoadManip());
    EXPECT_NEAR(loaded.GetMaxJointSpeeds()[2], 1.25, 1e-12);
    EXPECT_NEAR(loaded.GetMaxJointAccelerations()[5], 7.5, 1e-12);
    EXPECT_NEAR(loaded.GetMaxJointSpeeds()[0], Robo(JOINTS_COUNT).GetMaxJointSpeeds()[0], 1e-12);
}

TEST_F(RoboDynamicsSettings, MissingSpeedLimitsKeepTheDefaults)
{
    {
        QSettings ini("ManipulatorSettigns.ini", QSettings::IniFormat);
        for (int i = 0; i < JOINTS_COUNT; i++)
        {
            ini.remove("joint limits/" + QString::number(i) + "/max_speed_deg_s");
            ini.remove("joint limits/" + QString::number(i) + "/max_accel_deg_s2");
        }
    }
    Robo r(JOINTS_COUNT);
    ASSERT_TRUE(r.LoadManip());
    EXPECT_EQ(r.GetMaxJointSpeeds(), Robo(JOINTS_COUNT).GetMaxJointSpeeds());
    EXPECT_EQ(r.GetMaxJointAccelerations(), Robo(JOINTS_COUNT).GetMaxJointAccelerations());
}

TEST(RoboSpeedLimits, InvalidValuesAreRejected)
{
    Robo r(JOINTS_COUNT);
    EXPECT_THROW(r.SetMaxJointSpeed(0, 0), std::invalid_argument);
    EXPECT_THROW(r.SetMaxJointSpeed(0, std::nan("")), std::invalid_argument);
    EXPECT_THROW(r.SetMaxJointAcceleration(1, -2), std::invalid_argument);
    EXPECT_THROW(r.SetMaxJointSpeed(JOINTS_COUNT, 1), std::out_of_range);
    EXPECT_THROW(r.SetMaxJointAcceleration(-1, 1), std::out_of_range);
    EXPECT_NEAR(r.GetMaxJointSpeeds()[0], ToRad(180.0), 1e-12);
}
