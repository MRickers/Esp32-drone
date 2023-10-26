
if(${MPU6050_MONITOR_TEST})
message(STATUS "Building MPU6050 Monitor test!")
idf_component_register(SRCS "mpu6050_monitor.c"
                    INCLUDE_DIRS ""
                    REQUIRES "mpu6050")
else()
idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "")
endif()
