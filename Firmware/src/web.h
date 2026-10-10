#pragma once

#include "controller.h"
#include "calibration.h"
#include "safety.h"
#include "balance.h"
#include "web_status.h"

void webBegin();

void webUpdate(Controller &ctrl, Calibration &calib, Safety &safety, Leveling &leveling, const LoopStats &loop);
