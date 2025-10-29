class Timer{

public:
  Timer(std::string label, int8_t robot_id) : label_(label), robot_id_(robot_id), start_(std::chrono::steady_clock::now()) {
    SPDLOG_INFO("HEY DONT LOOK AT ME");
  }
  ~Timer() {
    SPDLOG_INFO("hey look at me");
    uint64_t time = static_cast<uint64_t>(std::chrono::
      duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start_).count());
    // Registry::instance().record(label_, time, robot_id_);
  }
private:
const std::chrono::steady_clock::time_point start_;
std::string label_;
int8_t robot_id_;
};