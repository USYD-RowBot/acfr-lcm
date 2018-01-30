#include <thread>
#include <iostream>

#include <sys/timerfd.h>
#include <unistd.h>


#include "controller.hpp"

#include "acfr-common/spektrum-control.h"
#include "acfr-common/timestamp.h"

ControllerBase::ControllerBase(std::string const &process_name, std::string const &vehicle_name)
    : root_key("acfr." + process_name), vehicle_name(vehicle_name), exit_signalled(false),
    planner_command_time(0), spektrum_command_time(0), nav_time(0),
    control_source(ControlSource::Dead), interval_us(100000), time_step(interval_us*1.0e-6)
{
    this->param = bot_param_new_from_server(this->lc_handle.getUnderlyingLCM(), 1);

    std::string key = root_key + ".spektrum_timeout";
    std::cout << "key: " << key << std::endl;
    spektrum_timeout = bot_param_get_int_or_fail(param, key.c_str());

    key = root_key + ".planner_timeout";
    std::cout << "key: " << key << std::endl;
    planner_timeout = bot_param_get_int_or_fail(param, key.c_str());

    key = root_key + ".nav_timeout";
    std::cout << "key: " << key << std::endl;
    nav_timeout = bot_param_get_int_or_fail(param, key.c_str());
}

ControllerBase::~ControllerBase()
{
    bot_param_destroy(this->param);
}

double ControllerBase::dt() const
{
    return time_step;
}

ControlSource ControllerBase::spektrum_control_mode(acfrlcm::auv_spektrum_control_command_t sc)
{
    // RC_AUX1 is the top left triple state switch on the DX-6 controllers
    if (sc.values[RC_AUX1] > REAR_POS_CUTOFF)
    {
        return ControlSource::Automatic;
    }
    else if (sc.values[RC_AUX1] > CENTER_POS_CUTOFF)
    {
        return ControlSource::Dead;
    }
    else
    {
        return ControlSource::Manual;
    }
}

// this is the magic function
void ControllerBase::run()
{
    this->init();

    // we need to spin off the LCM thread
    std::thread lcm_thread_handle(&ControllerBase::lcm_thread, this);

    // also to subscribe to the threads we want
    this->lc_handle.subscribe(vehicle_name + ".AUV_CONTROL", &ControllerBase::control_callback, this);
    this->lc_handle.subscribe(vehicle_name + ".SPEKTRUM_CONTROL", &ControllerBase::spektrum_callback, this);
    this->lc_handle.subscribe(vehicle_name + ".ACFR_NAV", &ControllerBase::acfr_nav_callback, this);

    // now we set up the interval timers
    int fd;
    struct itimerspec itval;

    fd = timerfd_create(CLOCK_MONOTONIC, 0);
    unsigned long long wakeups_missed = 0;

    if (fd == -1)
    {
        std::cerr << "Couldn't create timer." << std::endl;
        perror("create timer");
        this->exit_signalled = true;
        lcm_thread_handle.join();

        return;
    }

    std::cout << "Interval: " << (long)interval_us * 1000 << "us\n";
    std::cout << "Time step: " << time_step << "s\n";

    itval.it_interval.tv_sec = 0;
    itval.it_interval.tv_nsec = (long)interval_us * 1000;
    itval.it_value.tv_sec = 0;
    itval.it_value.tv_nsec = (long)interval_us * 1000;

    int ret;

    ret = timerfd_settime(fd, 0, &itval, NULL);

    if (ret)
    {
        perror("Failed to set time.");
        return;
    }

    // if (ret) check for failure
    
    // now the period waits
    while (!this->exit_signalled)
    {
        unsigned long long missed;
        // this handles the timer waiting
        ret = read(fd, &missed, sizeof(missed));
        if (ret == -1)
        {
            perror("read timer");

            this->exit_signalled = true;
            lcm_thread_handle.join();

            return;
        }

        if (missed > 1)
        {
            wakeups_missed += (missed - 1);
            std::cerr << "Missed a wakeup" << std::endl;
        }

        // now we can get to the logic of running the controller
        int64_t now = timestamp_now();
        if (this->control_source == ControlSource::Automatic
                && this->planner_command_time + this->planner_timeout > now
                && this->nav_time + this->nav_timeout > now)
        {
            // only if in automatic and within timeout do we use automatic control
            std::unique_lock<std::mutex> pl(this->planner_command_mutex, std::defer_lock);
            std::unique_lock<std::mutex> nl(this->nav_mutex, std::defer_lock);
            std::lock(pl, nl);

            this->automatic_control(this->planner_command, this->nav);
        }
        else if (this->control_source == ControlSource::Manual && this->spektrum_command_time + this->spektrum_timeout > now)
        {
            // only if in manual mode, and within timeout do we run under joystick ccontrol
            std::lock_guard<std::mutex> lg(this->spektrum_command_mutex);

            this->manual_control(this->spektrum_command);
        }
        else
        {
            // the safe fallback
            std::cout << "Dead" << std::endl;
            this->dead_control();
        }
    }
}

std::string const &ControllerBase::get_vehicle_name() const
{
    return this->vehicle_name;
}

void ControllerBase::quit()
{
    this->exit_signalled = true;
};

lcm::LCM &ControllerBase::lc()
{
    return this->lc_handle;
};

pid_gains_t ControllerBase::get_pid(std::string const &key)
{
    pid_gains_t pid;

    pid.kp = this->get_param(key + ".kp");
    pid.ki = this->get_param(key + ".ki");
    pid.kd = this->get_param(key + ".kd");
    pid.sat = this->get_param(key + ".sat");
    pid.integral = 0.0;
    pid.prev_error = 0.0;

    return pid;
}

double ControllerBase::get_param(std::string const &key)
{
    std::string full_key = this->root_key + '.' + key;
    return bot_param_get_double_or_fail(this->param, full_key.c_str());
}

void ControllerBase::lcm_thread()
{
    while (!exit_signalled)
    {
        this->lc_handle.handleTimeout(1000);
    }
}

void ControllerBase::control_callback(const lcm::ReceiveBuffer*rbuf, const std::string& channel,
                            const acfrlcm::auv_control_t *control)
{
    std::lock_guard<std::mutex> lg(this->planner_command_mutex);

    this->planner_command_time = timestamp_now();
    this->planner_command = *control;
}

void ControllerBase::spektrum_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                const acfrlcm::auv_spektrum_control_command_t *control)
{
    std::lock_guard<std::mutex> lg(this->spektrum_command_mutex);

    this->control_source = this->spektrum_control_mode(*control);

    this->spektrum_command_time = timestamp_now();
    this->spektrum_command = *control;
}

void ControllerBase::acfr_nav_callback(const lcm::ReceiveBuffer *rbuf, const std::string& channel,
                            const acfrlcm::auv_acfr_nav_t *nav)
{
    std::lock_guard<std::mutex> lg(this->nav_mutex);

    this->nav_time = timestamp_now();
    this->nav = *nav;
}
