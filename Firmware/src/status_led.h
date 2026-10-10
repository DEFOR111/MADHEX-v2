#pragma once

#include "controller.h"
#include "safety.h"

void statusLedBegin();

void statusLedUpdate(const Controller &ctrl, const Safety &safety);
