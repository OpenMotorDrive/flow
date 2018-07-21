#include <modules/worker_thread/worker_thread.h>
#include <hal.h>

#include <modules/uavcan_debug/uavcan_debug.h>

#include <modules/driver_vl53l1x/vl53l1_api.h>
#include <modules/driver_pmw3901mb/driver_pmw3901mb.h>

#define WT hpwork_thread
WORKER_THREAD_DECLARE_EXTERN(WT)

static const I2CConfig i2cconfig = {
  STM32_TIMINGR_PRESC(11U) |
  STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |
  STM32_TIMINGR_SCLH(3U)  | STM32_TIMINGR_SCLL(9U),
  0,
  0
};

static VL53L1_Dev_t vl53l1x;
static struct pmw3901mb_instance_s pmw3901mb;

static struct worker_thread_timer_task_s can_flow_test_task;
static void can_flow_test_task_func(struct worker_thread_timer_task_s* task);


RUN_AFTER(INIT_END) {
    i2cStart(&I2CD2, &i2cconfig);
    
    vl53l1x.bus = &I2CD2;
    vl53l1x.i2c_address = 0x29;
    
    VL53L1_Error status;
    
    status = VL53L1_WaitDeviceBooted(&vl53l1x);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_DataInit(&vl53l1x);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_StaticInit(&vl53l1x);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_SetDistanceMode(&vl53l1x, VL53L1_DISTANCEMODE_SHORT);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x, 100000);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(&vl53l1x, 1);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    status = VL53L1_StartMeasurement(&vl53l1x);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);

    pmw3901mb_init(&pmw3901mb, FLOW_SPI_BUS, BOARD_PAL_LINE_SPI_CS_FLOW, PMW3901MB_TYPE_V1);
        
    worker_thread_add_timer_task(&WT, &can_flow_test_task, can_flow_test_task_func, NULL, MS2ST(100), true);
}

static void can_flow_test_task_func(struct worker_thread_timer_task_s* task) {
    
    uavcan_send_debug_keyvalue("task", 1);  // just so I know if I'm getting back here 10 hz

    // get PMW3901MB motion report
    bool motion = pmw3901mb_motion_detected(&pmw3901mb);
    uavcan_send_debug_keyvalue("motion", motion); 
    return; // just getting read of motion register for now!
    
    if (motion) {
        struct pmw3901mb_motion_report_s motion;
        motion.delta_x = pmw3901mb_read_dx(&pmw3901mb);
        motion.delta_y = pmw3901mb_read_dy(&pmw3901mb);
        motion.squal = pmw3901mb_read(&pmw3901mb, PMW3901MB_SQUAL);
        motion.shutter_upper = pmw3901mb_read(&pmw3901mb, PMW3901MB_SHUTTER_UPPER);
        if (motion.squal < 0x19 && motion.shutter_upper == 0x1f) {
            uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "false motion", "%u %u", __LINE__, 0);
            return;
        }
        
        uavcan_send_debug_keyvalue("dx", motion.delta_x);
        uavcan_send_debug_keyvalue("dy", motion.delta_y);
    }

    return;
        
    // get VL53L1 range measurement
    VL53L1_Error status;
    uint8_t measurement_available = 0;
    status = VL53L1_GetMeasurementDataReady(&vl53l1x, &measurement_available);
    //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    if (measurement_available) {
        VL53L1_RangingMeasurementData_t meas_data;
        status = VL53L1_GetRangingMeasurementData(&vl53l1x, &meas_data);
        //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
        status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x);
        //uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "", "%u %u", __LINE__, status);
    
        if (status == VL53L1_ERROR_NONE) {
            if (meas_data.RangeStatus == 0 || meas_data.RangeStatus == 7) {
                uavcan_send_debug_keyvalue("rng", meas_data.RangeMilliMeter*1e-3);
            } else {
                uavcan_send_debug_keyvalue("stat", meas_data.RangeStatus);
                uavcan_send_debug_keyvalue("rng", -1);
            }
        }
    }
}
