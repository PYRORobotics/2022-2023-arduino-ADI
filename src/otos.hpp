#include "proto/messages.pb.h"

struct OTOSResult {
    Status status;
    int code;
  };

OTOSResult updateOTOS();

bool trySetupOTOS(bool calibrate = true);