/******************************************************************************
  Copyright (c) 2015 Particle Industries, Inc.  All rights reserved.
  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, either
  version 3 of the License, or (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this program; if not, see <http://www.gnu.org/licenses/>.
 ******************************************************************************/
#ifndef CELL_LOCATE_H_
#define CELL_LOCATE_H_

#include "application.h"


struct MDM_CELL_LOCATE {
    int day;
    int month;
    int year;
    int hour;
    int minute;
    int second;
    char lat[14];
    char lng[14];
    int altitude;
    int uncertainty;
    int speed;
    int direction;
    int vertical_acc;
    int sensor_used;
    int sv_used;
    int antenna_status;
    int jamming_status;
    int count;
    bool ok;
    int size;

    MDM_CELL_LOCATE()
    {
        memset(this, 0, sizeof(*this));
        size = sizeof(*this);
    }
};

void cell_locate_timeout_set(uint32_t timeout_ms);

bool is_cell_locate_timeout();

void cell_locate_timeout_clear();

bool is_cell_locate_matched(MDM_CELL_LOCATE& loc);

bool is_cell_locate_accurate(MDM_CELL_LOCATE& loc, uint16_t accuracy);

int _cbLOCATE(int type, const char* buf, int len, MDM_CELL_LOCATE* data);

int cell_locate(MDM_CELL_LOCATE& loc, uint32_t timeout_ms, uint16_t accuracy);

bool cell_locate_in_progress(MDM_CELL_LOCATE& loc);

bool cell_locate_get_response(MDM_CELL_LOCATE& loc);

void cell_locate_display(MDM_CELL_LOCATE& loc);

#endif // CELL_LOCATE_H_
