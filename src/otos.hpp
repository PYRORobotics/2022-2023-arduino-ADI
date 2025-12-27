#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "proto/messages.pb.h"
#pragma once

struct OTOSResult {
    Pose pose;
    int code;
  };

OTOSResult updateOTOS();

void processOTOSCommand(const Command& command);

// extern QwiicOTOS otos;

bool trySetupOTOS(bool calibrate = true);

void recoverOTOS();