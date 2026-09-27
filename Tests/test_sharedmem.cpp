// Phase 3: SharedMem -- the QSharedMemory wrapper the emulator uses to
// publish the manipulator state to the external visualiser.

#include "solver_test_access.h"
#include "test_helpers.h"

#include <QSharedMemory>

#include <memory>
#include <vector>

using test_helpers::ScopedSilence;

namespace {

// Unique key per test so leftovers of one test can't affect another.
QString Key(const char *name)
{
    return QString("manip_test_") + name;
}

std::unique_ptr<SharedMem> Make(const QString &key, size_t size)
{
    ScopedSilence quiet;
    return std::make_unique<SharedMem>(key, size);
}

} // namespace

TEST(SharedMem, ReportsElementCount)
{
    auto shm = Make(Key("size"), 18);
    EXPECT_EQ(shm->GetSize(), 18u);
}

TEST(SharedMem, ReaderSeesWhatWriterSent)
{
    auto writer = Make(Key("roundtrip"), 18);
    auto reader = Make(Key("roundtrip"), 18);
    std::vector<float> data(18);
    for (int i = 0; i < 18; i++)
        data[i] = 0.5f * i - 3;
    ASSERT_TRUE(writer->SendData(data.data(), sizeof(float) * data.size()));
    const float *got = static_cast<const float *>(reader->GetData());
    for (int i = 0; i < 18; i++)
        EXPECT_FLOAT_EQ(got[i], data[i]) << "i = " << i;
}

TEST(SharedMem, LaterWritesReplaceEarlierOnes)
{
    auto writer = Make(Key("overwrite"), 4);
    auto reader = Make(Key("overwrite"), 4);
    float a[4] = {1, 2, 3, 4}, b[4] = {-1, -2, -3, -4};
    ASSERT_TRUE(writer->SendData(a, sizeof(a)));
    ASSERT_TRUE(writer->SendData(b, sizeof(b)));
    const float *got = static_cast<const float *>(reader->GetData());
    for (int i = 0; i < 4; i++)
        EXPECT_FLOAT_EQ(got[i], b[i]);
}

TEST(SharedMem, LoadDataSizeReadsLeadingSizeField)
{
    auto writer = Make(Key("loadsize"), 8);
    std::vector<float> buf(8, 0);
    size_t n = 5;
    std::memcpy(buf.data(), &n, sizeof(n));
    ASSERT_TRUE(writer->SendData(buf.data(), sizeof(float) * buf.size()));
    EXPECT_EQ(writer->LoadDataSize(), 5u);
    EXPECT_EQ(writer->GetSize(), 5u);
}

TEST(SharedMem, LoadDataSizeRejectsOversizedHeader)
{
    auto writer = Make(Key("loadsize_big"), 8);
    std::vector<float> buf(8, 0);
    size_t n = MAX_DATA_SIZE + 1;
    std::memcpy(buf.data(), &n, sizeof(n));
    ASSERT_TRUE(writer->SendData(buf.data(), sizeof(float) * buf.size()));
    EXPECT_THROW(writer->LoadDataSize(), QString);
}

// Regression test for a fixed bug: SendData() memcpy()s `size` bytes into the segment
// without checking it against the segment size, so an oversized write
// silently runs past the end of the shared memory block.
TEST(SharedMem, SendDataRejectsMoreBytesThanTheSegmentHolds)
{
    auto shm = Make(Key("overflow"), 4); // 16 bytes
    float data[16] = {};
    EXPECT_FALSE(shm->SendData(data, sizeof(data)));
}

// Regression test for a fixed (minor) bug: QSharedMemory::create() already attaches, so the
// constructor's unconditional attach() afterwards always fails and prints
// "can't attach" for every successfully created segment.
TEST(SharedMem, ConstructionReportsNoError)
{
    testing::internal::CaptureStdout();
    {
        SharedMem shm(Key("noerror"), 4);
    }
    std::string out = testing::internal::GetCapturedStdout();
    EXPECT_EQ(out.find("can't attach"), std::string::npos) << out;
}

TEST(SharedMem, SegmentIsReleasedWhenLastUserIsDestroyed)
{
    {
        auto a = Make(Key("release"), 4);
        auto b = Make(Key("release"), 4);
    }
    QSharedMemory probe(Key("release"));
    EXPECT_FALSE(probe.attach());
}
