#include "unity.h"
#include "pid.h"

TEST_CASE("Pid", "[delete]")
{
    pid_config config = {{1., 1., 1.}, 100};
    pid_controller_t pid = create_pid(config);

    TEST_ASSERT(pid != MAX_PID_CONTROLLERS);
    TEST_ASSERT(delete_pid(pid) == 0);
    TEST_ASSERT(delete_pid(MAX_PID_CONTROLLERS) == -1);
    TEST_ASSERT(delete_pid(12345) == -1);
    TEST_ASSERT(delete_pid(-2345) == -1);
}

TEST_CASE("Pid", "[create]")
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
}
