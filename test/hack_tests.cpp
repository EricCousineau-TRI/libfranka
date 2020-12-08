#include <gmock/gmock.h>

#include <franka/hack.h>

using ::testing::ElementsAre;
using franka::BufferDelay;
using franka::CurrentTimeSeconds;
using franka::FromString;
using franka::HackEntry;
using franka::logToCSV;

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

TEST(Hack, TestCsv) {
  std::vector<HackEntry> log;
  log.emplace_back();
  const std::string actual = logToCSV(log);
  const std::string expected = R"""(
host_time, time, success_rate, q[0], q[1], q[2], q[3], q[4], q[5], q[6], dq[0], dq[1], dq[2], dq[3], dq[4], dq[5], dq[6], tau_J[0], tau_J[1], tau_J[2], tau_J[3], tau_J[4], tau_J[5], tau_J[6], tau_ext_hat_filtered[0], tau_ext_hat_filtered[1], tau_ext_hat_filtered[2], tau_ext_hat_filtered[3], tau_ext_hat_filtered[4], tau_ext_hat_filtered[5], tau_ext_hat_filtered[6], q_d[0], q_d[1], q_d[2], q_d[3], q_d[4], q_d[5], q_d[6], dq_d[0], dq_d[1], dq_d[2], dq_d[3], dq_d[4], dq_d[5], dq_d[6], q_c[0], q_c[1], q_c[2], q_c[3], q_c[4], q_c[5], q_c[6], delta_angle_delayed
0, 0, 0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0,0,0,0,0,0,0, 0
)""";
  EXPECT_EQ("\n" + actual, expected);
}

TEST(Hack, TestCurrentTimeSeconds) {
  EXPECT_GE(CurrentTimeSeconds(), 0.0);
}
