#include "AP_OpticalFlow_MAVLink.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include "OpticalFlow.h"
extern const AP_HAL::HAL& hal;

AP_OpticalFlow_MAVLink::AP_OpticalFlow_MAVLink(OpticalFlow &_frontend):
    OpticalFlow_backend(_frontend) 
{
    _last_read_ms = AP_HAL::millis();
}

void AP_OpticalFlow_MAVLink::init()
{
    
}

void AP_OpticalFlow_MAVLink::update()
{
    
}

void AP_OpticalFlow_MAVLink::handle_msg(mavlink_message_t *msg)
{
    mavlink_optical_flow_t packet;
    mavlink_msg_optical_flow_decode(msg, &packet);
    
    _last_read_ms = AP_HAL::millis();
    
    struct OpticalFlow::OpticalFlow_state state;
    state.device_id = packet.sensor_id;
    state.surface_quality = packet.quality; 
    const Vector3f &gyro = get_ahrs().get_gyro();
    
    if (state.surface_quality > 50) {
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

        state.flowRate = Vector2f(packet.flow_rate_x * flowScaleFactorX ,
                                  packet.flow_rate_y * flowScaleFactorY );

        // delta_time is in microseconds so multiply to get back to seconds
        state.bodyRate = Vector2f(gyro.x ,gyro.y);

        _applyYaw(state.flowRate);
    } else {
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    // copy results to front end
    _update_frontend(state);

//    hal.console->printf("X: %4.2f Y: %4.2f\n",state.flowRate.x ,state.flowRate.y);
//    hal.console->printf("FLOW_ONBOARD qual:%u FlowRateX:%4.2f Y:%4.2f"
//                        "BodyRateX:%4.2f Y:%4.2f\n",
//                        (unsigned)state.surface_quality,
//                        (double)state.flowRate.x,
//                        (double)state.flowRate.y,
//                        (double)state.bodyRate.x,
//                        (double)state.bodyRate.y);
//    hal.console->printf("get optical flow in mavlink\n");
}
