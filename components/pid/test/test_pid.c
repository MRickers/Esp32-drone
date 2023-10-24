#include "unity.h"
#include "pid.h"
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <esp_timer.h>

void tearDown()
{
    delete_pid(0);
    delete_pid(1);
    delete_pid(2);
    delete_pid(3);
}

TEST_CASE("Pid", "[delete]")
{
    pid_config config = {{1., 1., 1.}, 100};
    pid_controller_t pid = create_pid(config);

    TEST_ASSERT(pid != MAX_PID_CONTROLLERS);
    TEST_ASSERT(delete_pid(pid) == PID_OK);
    TEST_ASSERT(delete_pid(MAX_PID_CONTROLLERS) == PID_FAILED);
}

TEST_CASE("Pid", "[get/set params]")
{
    pid_config config = {{1., 2., 3.}, 100};
    pid_controller_t pid = create_pid(config);

    pid_params params;
    TEST_ASSERT(get_pid_params(pid, &params) == PID_OK);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.f, params.kp);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.f, params.ki);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.f, params.kd);
}

TEST_CASE("Pid", "[set setpoint]")
{
    pid_config config = {{1., 2., 3.}, 100};
    pid_controller_t pid = create_pid(config);

    TEST_ASSERT(set_setpoint(pid, 80) == PID_OK);
    float setpoint;
    TEST_ASSERT(get_setpoint(pid, &setpoint) == PID_OK);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 80.f, setpoint);
}

TEST_CASE("Pid", "[set cycletime]")
{
    pid_config config = {{1., 2., 3.}, 100};
    pid_controller_t pid = create_pid(config);

    TEST_ASSERT(set_cycle_time(pid, 100) == PID_OK);
    pid_cycle_time_t cycle_time;
    TEST_ASSERT(get_cycle_time(pid, &cycle_time) == PID_OK);
    TEST_ASSERT_EQUAL(100, cycle_time);
}

TEST_CASE("Pid", "[create/delete simple]")
{
    pid_config config = {{1., 1., 1.}, 100};
    pid_controller_t pid1 = create_pid(config);
    pid_controller_t pid2 = create_pid(config);
    pid_controller_t pid3 = create_pid(config);
    pid_controller_t pid4 = create_pid(config);
    TEST_ASSERT(pid1 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(pid2 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(pid3 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(pid4 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(create_pid(config) == MAX_PID_CONTROLLERS);
    TEST_ASSERT(delete_pid(pid1) == PID_OK);
    TEST_ASSERT(delete_pid(pid2) == PID_OK);
    TEST_ASSERT(delete_pid(pid3) == PID_OK);
    TEST_ASSERT(delete_pid(pid4) == PID_OK);
}

TEST_CASE("Pid", "[create/delete]")
{
    pid_config config = {{1., 1., 1.}, 100};
    pid_controller_t pid1 = create_pid(config);
    pid_controller_t pid2 = create_pid(config);
    pid_controller_t pid3 = create_pid(config);
    pid_controller_t pid4 = create_pid(config);
    TEST_ASSERT(pid1 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(pid2 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(pid3 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(pid4 != MAX_PID_CONTROLLERS);
    TEST_ASSERT(create_pid(config) == MAX_PID_CONTROLLERS);
    TEST_ASSERT(delete_pid(pid2) == PID_OK);
    TEST_ASSERT(delete_pid(pid4) == PID_OK);
    TEST_ASSERT(set_setpoint(pid2, 150.f) == PID_FAILED);
    pid2 = create_pid(config);
    TEST_ASSERT(set_setpoint(pid2, 150.f) == PID_OK);
}

TEST_CASE("Pid", "[healthy failed]")
{
    TEST_ASSERT(healthy(0) == PID_FAILED);
}

TEST_CASE("Pid", "[healthy unhealty]")
{
    pid_config config = {{1., 2., 3.}, 100};
    pid_controller_t pid = create_pid(config);

    vTaskDelay(pdMS_TO_TICKS(65));
    TEST_ASSERT(healthy(pid) == PID_UNHEALTY);
}

TEST_CASE("Pid", "[healthy healty]")
{
    pid_config config = {{1., 2., 3.}, 100};
    pid_controller_t pid = create_pid(config);
    vTaskDelay(pdMS_TO_TICKS(65));
    TEST_ASSERT(healthy(pid) == PID_UNHEALTY);
    float output;
    TEST_ASSERT(update(pid, 80, &output) == PID_OK);
    TEST_ASSERT(healthy(pid) == PID_OK);
}

// PT1 system parameters
const float tau = 4.0; // Time constant
const float Kp = 2.0;  // Gain

// Function to simulate the PT1 system
float pt1_system(float input, float dt, float *state)
{
    float delta = (Kp * (input - *state)) * dt / tau;

    // Update the state
    *state += delta;

    // Output of the system (PT1 response)
    return *state;
}

TEST_CASE("Pid", "[update PT1 Model]")
{
    float setpoint = 1.f;
    pid_cycle_time_t time = 0.f;
    float state = 0.f;
    float input = setpoint;

    pid_config config = {{20.f, 10.f, 0.2f}, setpoint};
    pid_controller_t pid = create_pid(config);

    pid_cycle_time_t cycle_time_ms;
    get_cycle_time(pid, &cycle_time_ms);

    int64_t runtime_ms = 2000;
    int64_t start = esp_timer_get_time();

    while (((esp_timer_get_time() - start) / 1000) < runtime_ms)
    {
        if (healthy(pid) == PID_UNHEALTY)
        {
            float output;
            update(pid, state, &output);
            float system_output = pt1_system(output, (float)cycle_time_ms / 1000, &state);
            printf("Time: %ld, Setpoint: %.3f, System Output: %.3f, Control Output: %.3f, Input: %.3f\n", time, setpoint, system_output, output, input);
            time += cycle_time_ms;
        }
    }
}