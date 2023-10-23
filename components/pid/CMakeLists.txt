set(srcs "pid.c"
)

idf_component_register(SRCS ${srcs}
INCLUDE_DIRS "include"
PRIV_REQUIRES esp_timer)