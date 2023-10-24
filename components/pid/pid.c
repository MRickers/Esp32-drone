#include <stdio.h>
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
    float i_error_sum;
    float previous_error;
    int used;
    int64_t last_update;
} pid_details;

static pid_details controllers[MAX_PID_CONTROLLERS];

static int is_used(pid_controller_t pid)
{
    return ((pid < MAX_PID_CONTROLLERS) && controllers[pid].used == 1) ? 1 : 0;
}

void init_pid_controller(pid_controller_t pid, const pid_config *config)
{
    controllers[pid].used = 1;
    if (is_used(pid))
    {
        controllers[pid].setpoint = config->setpoint;
        controllers[pid].output_limits.min = 0;
        controllers[pid].output_limits.max = 255;
        controllers[pid].cycle_time_ms = 60;
        controllers[pid].i_error_sum = 0;
        controllers[pid].previous_error = 0;
        set_pid_params(pid, config->params);
        controllers[pid].last_update = esp_timer_get_time() - controllers[pid].cycle_time_ms;
    }
    else
    {
        controllers[pid].used = 0;
    }
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

pid_err delete_pid(pid_controller_t pid)
{
    if (is_used(pid))
    {
        controllers[pid].used = 0;
        return PID_OK;
    }
    return PID_FAILED;
}

pid_err set_output_limits(pid_controller_t pid, float min, float max)
{
    if (is_used(pid) && min < max)
    {
        controllers[pid].output_limits.min = min;
        controllers[pid].output_limits.max = max;
        return PID_OK;
    }
    return PID_FAILED;
}

pid_err set_setpoint(pid_controller_t pid, float setpoint)
{
    if (is_used(pid))
    {
        controllers[pid].setpoint = setpoint;
        return PID_OK;
    }
    return PID_FAILED;
}

pid_err set_pid_params(pid_controller_t pid, pid_params params)
{
    if (is_used(pid))
    {
        controllers[pid].params = params;

        return PID_OK;
    }
    return PID_FAILED;
}

pid_err set_cycle_time(pid_controller_t pid, pid_cycle_time_t time_ms)
{
    if (is_used(pid) && time_ms > 0)
    {
        controllers[pid].cycle_time_ms = time_ms;
        return PID_OK;
    }
    return PID_FAILED;
}

pid_err update(pid_controller_t pid, float input, float *output)
{
    if (is_used(pid) &&
        ((esp_timer_get_time() - controllers[pid].cycle_time_ms) >= controllers[pid].cycle_time_ms))
    {
        float dt = (float)controllers[pid].cycle_time_ms / 1000;

        float error = controllers[pid].setpoint - input;
        controllers[pid].i_error_sum += error * dt;
        float derivative = (error - controllers[pid].previous_error) / dt;
        controllers[pid].previous_error = error;

        float P = controllers[pid].params.kp * error;
        float I = controllers[pid].params.ki * controllers[pid].i_error_sum;
        float D = controllers[pid].params.kd * derivative;
        float controller_output = P + I + D;
        *output = controller_output;

        if (controller_output > controllers[pid].output_limits.max)
        {
            *output = controllers[pid].output_limits.max;
        }
        else if (controller_output < controllers[pid].output_limits.min)
        {
            *output = controllers[pid].output_limits.min;
        }
        controllers[pid].last_update = esp_timer_get_time();

        return PID_OK;
    }
    return PID_FAILED;
}

pid_err healthy(pid_controller_t pid)
{
    if (is_used(pid))
    {
        return ((esp_timer_get_time() - controllers[pid].last_update) <= controllers[pid].cycle_time_ms) ? PID_OK : PID_UNHEALTY;
    }
    return PID_FAILED;
}

pid_err get_pid_params(pid_controller_t pid, pid_params *params)
{
    if (is_used(pid))
    {
        *params = controllers[pid].params;
        return PID_OK;
    }
    return PID_FAILED;
}

pid_err get_cycle_time(pid_controller_t pid, pid_cycle_time_t *cycle_time_ms)
{
    if (is_used(pid))
    {
        *cycle_time_ms = controllers[pid].cycle_time_ms;
        return PID_OK;
    }
    return PID_FAILED;
}

pid_err get_setpoint(pid_controller_t pid, float *setpoint)
{
    if (is_used(pid))
    {
        *setpoint = controllers[pid].setpoint;
        return PID_OK;
    }
    return PID_FAILED;
}