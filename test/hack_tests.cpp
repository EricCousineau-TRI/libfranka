#include <gmock/gmock.h>

#include <franka/hack.h>

using ::testing::ElementsAre;
using franka::BufferDelay;
using franka::FromString;

namespace {

std::vector<double>
CheckBufferDelay(int num_delay, const std::vector<double> inputs) {
  std::vector<double> outputs;
  BufferDelay dut(num_delay);
  for (const double u : inputs) {
    const double y = dut.Step(u);
    outputs.push_back(y);
  }
  return outputs;
}

}

TEST(Hack, TestBufferDelay) {
  const std::vector<double> inputs = {1, 2, 3, 4, 5};
  EXPECT_THAT(CheckBufferDelay(0, inputs), ElementsAre(1, 2, 3, 4, 5));
  EXPECT_THAT(CheckBufferDelay(1, inputs), ElementsAre(1, 1, 2, 3, 4));
  EXPECT_THAT(CheckBufferDelay(2, inputs), ElementsAre(1, 1, 1, 2, 3));
  EXPECT_THAT(CheckBufferDelay(3, inputs), ElementsAre(1, 1, 1, 1, 2));
  EXPECT_THAT(CheckBufferDelay(4, inputs), ElementsAre(1, 1, 1, 1, 1));
}

TEST(Hack, TestFromString) {
  EXPECT_EQ(FromString<int>("127"), 127);
  EXPECT_EQ(FromString<bool>("1"), true);
  EXPECT_EQ(FromString<double>("1.5"), 1.5);
}
