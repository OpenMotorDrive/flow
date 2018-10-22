#pragma once
#include <stdint.h>
#include <modules/pubsub/pubsub.h>

struct imu_sample_s {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
};

extern struct pubsub_topic_s invensense_raw_sample_topic;
