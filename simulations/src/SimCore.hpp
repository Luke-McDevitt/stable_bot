#pragma once
#include <vector>
#include <string>

struct JointState { std::string name; double pos{0}, vel{0}, eff{0}; };
struct ImuSample  { double qx{0}, qy{0}, qz{0}, qw{1}, wx{0}, wy{0}, wz{0}, ax{0}, ay{0}, az{0}; };

class SimCore {
public:
    bool loadWorld(const std::string& path);   // optional at first, return true
    void setFixedDt(double dt_seconds);        // e.g., 0.005
    void step();                               // advance by dt and update sensors/physics

    // Actuator commands in (choose what you actually use: pos/vel/eff)
    void setJointTargets(const std::vector<double>& targets);

    // Sensor reads out
    std::vector<JointState> readJointStates() const;
    ImuSample               readImu() const;

    // Time
    double simTimeSec() const;

private:
    double dt_{0.005};
    double t_{0.0};
    std::vector<JointState> joints_; // fill with your turret joints
    ImuSample imu_;
};
