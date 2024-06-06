#include <gtest/gtest.h>
#include "kr_mavros_interface/so3cmd_to_mavros_testor.hpp"
#include <mutex>
#include <thread>

class SO3CmdToMavrosTest: public testing::Test
{
  public:
    SO3CmdToMavrosTest()
      : tester(std::make_shared<SO3CmdToMavrosTester>()),
        executor(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
    }

    std::shared_ptr<SO3CmdToMavrosTester> tester;
    std::thread spin_thread;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;

    void SetUp() override
    {
      executor->add_node(tester);
      spin_thread = std::thread([this]() { executor->spin(); });
      rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void TearDown() override
    {
      executor->cancel();
      spin_thread.join();
    }
};

/**
 * @brief Test1: baseline test
 * Expected behavior: attitude_raw_msg_received = false, odom_msg_received = false
 */
TEST_F(SO3CmdToMavrosTest, Test1)
{
  std::cout << "Performing Test1!\n";
  {
    std::lock_guard<std::mutex> lock(tester->mutex);
    EXPECT_FALSE(tester->attitude_raw_msg_received_);
    EXPECT_FALSE(tester->odom_msg_received_);
  }
}

/**
 * @brief Test2: publish odom message and check if it is received
 * Expected behavior: odom_msg_received = true, odom_msg_.pose.pose = odom.pose.pose
 */
TEST_F(SO3CmdToMavrosTest, Test2)
{
    std::cout << "Performing Test2!\n";
    tester->publish_odom_msg();
    rclcpp::sleep_for(std::chrono::seconds(1));
    {
        std::lock_guard<std::mutex> lock(tester->mutex);
        EXPECT_TRUE(tester->odom_msg_received_);
        EXPECT_EQ(tester->odom_msg_->pose.position.x, 0.0);
        EXPECT_EQ(tester->odom_msg_->pose.position.y, 0.0);
        EXPECT_EQ(tester->odom_msg_->pose.position.z, 0.0);
        EXPECT_EQ(tester->odom_msg_->pose.orientation.w, 1.0);
        EXPECT_EQ(tester->odom_msg_->pose.orientation.x, 0.0);
        EXPECT_EQ(tester->odom_msg_->pose.orientation.y, 0.0);
        EXPECT_EQ(tester->odom_msg_->pose.orientation.z, 0.0);
    }
    
}

/**
 * @brief Test3: publish odom message and imu message and so3 command with motor disabled
 * Expected behavior: attitude_raw_msg_received = true, odom_msg_received = true, thrust = 0.0
 */
TEST_F(SO3CmdToMavrosTest, Test3)
{
    std::cout << "Performing Test3!\n";
    tester->populate_so3cmd_vector();
    tester->publish_odom_msg();
    tester->publish_imu_msg();
    rclcpp::sleep_for(std::chrono::seconds(1));
    tester->publish_so3_cmd(0);
    rclcpp::sleep_for(std::chrono::seconds(5));
    {
        std::lock_guard<std::mutex> lock(tester->mutex);
        EXPECT_TRUE(tester->attitude_raw_msg_received_);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.w, 1.0);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.x, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.y, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.z, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->body_rate.x, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->body_rate.y, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->body_rate.z, 0.0);
        EXPECT_NEAR(tester->attitude_raw_msg_->thrust, 0.0, 1e-4);
    }
}

/**
 * @brief Test4: publish odom message and imu message and so3 command with motor enabled
 * Expected behavior: attitude_raw_msg_received = true, odom_msg_received = true, thrust = 0.1
 */
TEST_F(SO3CmdToMavrosTest, Test4)
{
    std::cout << "Performing Test4!\n";
    tester->populate_so3cmd_vector();
    tester->publish_odom_msg();
    tester->publish_imu_msg();
    rclcpp::sleep_for(std::chrono::seconds(1));
    tester->publish_so3_cmd(2);
    rclcpp::sleep_for(std::chrono::seconds(5));
    {
        std::lock_guard<std::mutex> lock(tester->mutex);
        EXPECT_TRUE(tester->attitude_raw_msg_received_);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.w, 1.0);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.x, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.y, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->orientation.z, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->body_rate.x, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->body_rate.y, 0.0);
        EXPECT_EQ(tester->attitude_raw_msg_->body_rate.z, 0.0);
        EXPECT_NEAR(tester->attitude_raw_msg_->thrust, 0.2581, 1e-3);
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}