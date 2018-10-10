#include <string>
#include <mutex>

#include <lcm/lcm-cpp.hpp>
#include "perls-lcmtypes++/acfrlcm/auv_acfr_nav_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_control_t.hpp"
#include "perls-lcmtypes++/acfrlcm/auv_spektrum_control_command_t.hpp"

#include "bot_param/param_client.h"

#include "pid.h"

enum class ControlSource : char
{
    Automatic,
    Dead,
    Manual
};

class ControllerBase
{
public:
    ControllerBase(std::string const &process_name, std::string const &vehicle_name);
    virtual ~ControllerBase();

    void run();
    std::string const &get_vehicle_name() const;

    double dt() const;
    double prev_rudder_angle;
    double prev_elev_angle;

    void quit();

    lcm::LCM &lc();

protected:
    // these are to be overriden by the inheriting
    // controllers
    virtual void manual_control(acfrlcm::auv_spektrum_control_command_t sc) = 0;
    virtual void automatic_control(acfrlcm::auv_control_t ac, acfrlcm::auv_acfr_nav_t nav) = 0;
    virtual void dead_control() = 0;
    virtual void init() = 0;

    // this can be change in the inherited controllers
    // if necessary. It is called with every spektrum control
    // message and sets an internal flag
    virtual ControlSource spektrum_control_mode(acfrlcm::auv_spektrum_control_command_t sc);

    // used to load parameters from the config
    // in practice the only needed parameters are PID
    // messages
    double get_param(std::string const &key);
    pid_gains_t get_pid(std::string const &key);

private:
    lcm::LCM lc_handle;
    BotParam *param;

    std::string root_key;
    std::string vehicle_name;

    bool exit_signalled;

    void lcm_thread();

    void control_callback(const lcm::ReceiveBuffer*rbuf, const std::string& channel,
                             const acfrlcm::auv_control_t *control);
    void spektrum_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                    const acfrlcm::auv_spektrum_control_command_t *cmd);
    void acfr_nav_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                              const acfrlcm::auv_acfr_nav_t *nav);

    // the three LCM messages we parse in
    acfrlcm::auv_control_t planner_command;
    acfrlcm::auv_spektrum_control_command_t spektrum_command;
    acfrlcm::auv_acfr_nav_t nav;

    int64_t planner_command_time;
    int64_t spektrum_command_time;
    int64_t nav_time;

    std::mutex planner_command_mutex;
    std::mutex spektrum_command_mutex;
    std::mutex nav_mutex;

    // micro-seconds after which we treat these as expired
    int64_t spektrum_timeout;
    int64_t planner_timeout;
    int64_t nav_timeout;

    ControlSource control_source;

    long long int interval_us;
    double time_step;
};
