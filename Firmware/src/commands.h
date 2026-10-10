#pragma once

#include "calibration.h"
#include "controller.h"
#include "safety.h"
#include "balance.h"

void commandsUpdate(Controller &ctrl, Calibration &calib, Safety &safety, Leveling &leveling);

class Print;

void commandsRunLine(const char *line, Print &reply, Controller &ctrl, Calibration &calib,
                     Safety &safety, Leveling &leveling);
