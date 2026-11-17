#include "LIVMapper.h"
#include "stop_signal.h"

// Ctrl+C 时不要立刻 rclcpp::shutdown():
// 信号处理默认会让 rcl 的 context 立即失效, 而 run() 循环此刻可能正卡在
// handleVIO() -> pubImage.publish() 里(image_transport 惰性创建 /rgb_img 的
// 底层 publisher), 于是抛 rclcpp::exceptions::RCLError("could not create
// publisher: rcl node's context is invalid") 且无人接管 -> std::terminate
// -> SIGABRT(exit -6), run() 永远返回不了, 后面的 savePCD() 也就不会执行,
// 表现就是"按了 Ctrl+C 但地图没保存、也没有任何报错"。
// 这里改成只置标志位, 让 run() 从循环顶部正常退出 -> savePCD() 落盘 -> 再 shutdown。
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    fast_livo_stop::install();
    rclcpp::Node::SharedPtr nh;
    image_transport::ImageTransport it_(nh);
    LIVMapper mapper(nh, "laserMapping");
    mapper.initializeSubscribersAndPublishers(nh, it_);
    mapper.run(nh);
    fast_livo_stop::report(std::cerr, rclcpp::ok());
  } catch (const std::exception &error) {
    if (g_stop_requested) fast_livo_stop::report(std::cerr, rclcpp::ok());
    std::cerr << "[NAV_FAILURE] " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
