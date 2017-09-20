#ifndef AP_OPTICALFLOW_MAVLINK_H
#define AP_OPTICALFLOW_MAVLINK_H

#pragma once

#include "OpticalFlow.h"

class AP_OpticalFlow_MAVLink : public OpticalFlow_backend
{
public:
    AP_OpticalFlow_MAVLink(OpticalFlow &_frontend);
    void init(void);
    void update(void);
    void handle_msg(mavlink_message_t *msg) override;
private:
    uint32_t _last_read_ms;
};

#endif // AP_OPTICALFLOW_MAVLINK_H
