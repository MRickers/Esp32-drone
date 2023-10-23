#include "pid.h"
#include <esp_timer.h>

typedef struct pid_limits_
{
    float min;
    float max;
} pid_limits;

typedef struct pid_details_
{
    float setpoint;
    pid_params params;
    pid_limits output_limits;
    pid_cycle_time_t cycle_time_ms;
    float i_sum;
    float last_input;
    int used;
    int64_t last_update;
} pid_details;

static pid_details controllers[MAX_PID_CONTROLLERS];

void init_pid_controller(pid_controller_t pid, const pid_config *config)
{
    controllers[pid].params.kp = config->params.kp;
    controllers[pid].params.ki = config->params.ki;
    controllers[pid].params.kd = config->params.kd;
    controllers[pid].setpoint = config->setpoint;
    controllers[pid].output_limits.min = 0;
    controllers[pid].output_limits.max = 255;
    controllers[pid].cycle_time_ms = 60;
    controllers[pid].i_sum = 0;
    controllers[pid].last_input = 0;
    controllers[pid].used = 1;
    controllers[pid].last_update = esp_timer_get_time();
}

pid_controller_t create_pid(pid_config config)
{
    for (int i = 0; i < MAX_PID_CONTROLLERS; i++)
    {
        if (controllers[i].used == 0)
        {
            init_pid_controller(i, &config);
            return i;
        }
    }
    return MAX_PID_CONTROLLERS;
}

int delete_pid(pid_controller_t index)
{
    if (index < MAX_PID_CONTROLLERS)
    {
        controllers[index].used = 0;
        return PID_OK;
    }
    return PID_FAILED;
}

int set_output_limits(pid_controller_t pid, float min, float max)
{
    if (pid < MAX_PID_CONTROLLERS &&
        min < max)
    {
        controllers[pid].output_limits.min = min;
        controllers[pid].output_limits.max = max;
        return PID_OK;
    }
    return PID_FAILED;
}

int set_setpoint(pid_controller_t pid, float setpoint)
{
    if (pid < MAX_PID_CONTROLLERS)
    {
        controllers[pid].setpoint = setpoint;
        return PID_OK;
    }
    return PID_FAILED;
}

int set_pid_params(pid_controller_t pid, pid_params params)
{
    if (pid < MAX_PID_CONTROLLERS)
    {
        controllers[pid].params = params;
        return PID_OK;
    }
    return PID_FAILED;
}

int set_cycle_time(pid_controller_t pid, pid_cycle_time_t time_ms)
{
    if (pid < MAX_PID_CONTROLLERS)
    {
        controllers[pid].cycle_time_ms = time_ms;
        return PID_OK;
    }
    return PID_FAILED;
}

int healthy(pid_controller_t pid)
{
    if (pid < MAX_PID_CONTROLLERS)
    {
        if ((esp_timer_get_time() - controllers[pid].last_update) >= controllers[pid].cycle_time_ms)
        {
            return PID_OK;
        }
        return PID_UNHEALTY;
    }
    return PID_FAILED;
}