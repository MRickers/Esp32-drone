#ifndef _PID_H_
#define _PID_H_
#include <stdint.h>

typedef enum pid_err_
{
    PID_OK,
    PID_FAILED,
    PID_UNHEALTY,
} pid_err;

typedef uint8_t pid_controller_t;
// Cycle time of the controller [ms]
typedef uint32_t pid_cycle_time_t;

typedef struct pid_params_
{
    float kp;
    float ki;
    float kd;
} pid_params;

typedef struct pid_config_
{
    pid_params params;
    float setpoint;
} pid_config;

#define MAX_PID_CONTROLLERS 4

pid_controller_t create_pid(pid_config);
pid_err delete_pid(pid_controller_t);

pid_err set_output_limits(pid_controller_t, float min, float max);

pid_err healthy(pid_controller_t);
pid_err set_setpoint(pid_controller_t, float);
pid_err set_pid_params(pid_controller_t, pid_params);
pid_err set_cycle_time(pid_controller_t, pid_cycle_time_t);
pid_err get_pid_params(pid_controller_t, pid_params *);
pid_err get_cycle_time(pid_controller_t, pid_cycle_time_t *);
pid_err get_setpoint(pid_controller_t, float *);

pid_err update(pid_controller_t, float input, float *output);

#endif // _PID_H_