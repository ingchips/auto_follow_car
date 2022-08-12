#pragma once

#include "profile.h"

void vehicle_init(void);

void vehicle_update(target_pos_t *pos);

void vehicle_reset(void);

void vehicle_lock(void);
void vehicle_unlock(void);
