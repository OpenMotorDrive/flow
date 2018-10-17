#include <modules/worker_thread/worker_thread.h>
#include <hal.h>

#include <modules/uavcan_debug/uavcan_debug.h>

#include <modules/driver_vl53l1x/vl53l1_api.h>
#include <modules/driver_pmw3901mb/driver_pmw3901mb.h>

#include <com.hex.equipment.flow.Measurement.h>
#include <uavcan.equipment.range_sensor.Measurement.h>

#include <imu_integrator.h>

#define WT spi3_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

WORKER_THREAD_DECLARE_EXTERN(i2c_thread)

static const I2CConfig i2cconfig = {
  STM32_TIMINGR_PRESC(11U) |
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U)  | STM32_TIMINGR_SCLL(9U),
  0,
  0
};

static VL53L1_Dev_t vl53l1x;
static struct pmw3901mb_instance_s pmw3901mb;

static struct worker_thread_listener_task_s imu_delta_listener_task;
static void imu_deltas_handler(size_t msg_size, const void* buf, void* ctx);

static struct worker_thread_timer_task_s range_task;
static void range_task_func(struct worker_thread_timer_task_s* task);

RUN_AFTER(INIT_END) {
    
    // initialize VL53L1 lidar
    i2cStart(&I2CD2, &i2cconfig);
    vl53l1x.bus = &I2CD2;
    vl53l1x.i2c_address = 0x29;
    
    VL53L1_Error status;
    status = VL53L1_WaitDeviceBooted(&vl53l1x);
    status = VL53L1_DataInit(&vl53l1x);
    status = VL53L1_StaticInit(&vl53l1x);
    status = VL53L1_SetDistanceMode(&vl53l1x, VL53L1_DISTANCEMODE_SHORT);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x, 100000);
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&vl53l1x, 5);
    status = VL53L1_StartMeasurement(&vl53l1x);

    // initialize PMW3901MB optical flow
    pmw3901mb_init(&pmw3901mb, FLOW_SPI_BUS, BOARD_PAL_LINE_SPI_CS_FLOW, PMW3901MB_TYPE_V1);
    
    worker_thread_add_listener_task(&WT, &imu_delta_listener_task, &imu_deltas_topic, imu_deltas_handler, NULL);
    worker_thread_add_timer_task(&i2c_thread, &range_task, range_task_func, NULL, LL_MS2ST(200), true);

}

static void imu_deltas_handler(size_t msg_size, const void* buf, void* ctx) {
    {
        const struct imu_delta_s* deltas = (const struct imu_delta_s*)buf;
        
        struct pmw3901mb_motion_report_s motion_report;
        pmw3901mb_burst_read(&pmw3901mb, &motion_report);
        
        static struct com_hex_equipment_flow_Measurement_s msg;
        
        msg.quality = motion_report.squal;
        
        if (motion_report.shutter_upper == 0x1f || motion_report.squal < 0x19) {
            msg.quality = 0;
        }
        
        msg.integration_interval = deltas->dt;
        msg.rate_gyro_integral[0] = -deltas->delta_ang[1];
        msg.rate_gyro_integral[1] = deltas->delta_ang[0];
        msg.flow_integral[0] = motion_report.delta_x*1.8e-3;
        msg.flow_integral[1] = motion_report.delta_y*1.8e-3;
        
        uavcan_broadcast(0, &com_hex_equipment_flow_Measurement_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg);
    }
    
    {
        // get VL53L1 range measurement
        VL53L1_Error status;
        uint8_t measurement_available = 0;
        status = VL53L1_GetMeasurementDataReady(&vl53l1x, &measurement_available);
        if (measurement_available) {
            VL53L1_RangingMeasurementData_t meas_data;
            status = VL53L1_GetRangingMeasurementData(&vl53l1x, &meas_data);
            status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x);
        
            if (status == VL53L1_ERROR_NONE) {
                static struct uavcan_equipment_range_sensor_Measurement_s msg;
                memset(&msg, 0, sizeof(msg));
                
                msg.sensor_id = 0;
                msg.beam_orientation_in_body_frame.orientation_defined = false;
                
                msg.field_of_view = 0.4;
                
                if (meas_data.RangeStatus == 0 || meas_data.RangeStatus == 7) {
                    msg.range = meas_data.RangeMilliMeter*1e-3;
                    msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
                } else {
                    msg.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
                }
                uavcan_broadcast(0, &uavcan_equipment_range_sensor_Measurement_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM+1, &msg);
            }
        }
    }
}

static void range_task_func(struct worker_thread_timer_task_s* task) {

}
