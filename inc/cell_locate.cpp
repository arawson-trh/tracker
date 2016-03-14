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

#include "cell_locate.h"

volatile uint32_t cellTimeout;
volatile uint32_t cellTimeStart;

void cell_locate_timeout_set(uint32_t timeout_ms) {
    cellTimeout = timeout_ms;
    cellTimeStart = millis();
}

bool is_cell_locate_timeout() {
    return (cellTimeout && ((millis()-cellTimeStart) > cellTimeout));
}

void cell_locate_timeout_clear() {
    cellTimeout = 0;
}

bool is_cell_locate_matched(MDM_CELL_LOCATE& loc) {
    return loc.ok;
}

bool is_cell_locate_accurate(MDM_CELL_LOCATE& loc, uint16_t accuracy) {
    return loc.ok && loc.uncertainty <= accuracy;
}

/* Cell Locate Callback */
int _cbLOCATE(int type, const char* buf, int len, MDM_CELL_LOCATE* data)
{
    if ((type == TYPE_PLUS) && data) {
        // DEBUG CODE TO SEE EACH LINE PARSED
        // char line[256];
        // strncpy(line, buf, len);
        // line[len] = '\0';
        // Serial.printf("LINE: %s",line);

        // <response_type> = 1:
        //+UULOC: <date>,<time>,<lat>,<long>,<alt>,<uncertainty>,<speed>,<direction>,
        //        <vertical_acc>,<sensor_used>,<SV_used>,<antenna_status>,<jamming_status>
        //+UULOC: 25/09/2013,10:13:29.000,45.7140971,13.7409172,266,17,0,0,18,1,6,3,9
        int count = 0;
        //
        // TODO: %f was not working for float on LAT/LONG, so opted for capturing strings for now
        if ( (count = sscanf(buf, "\r\n+UULOC: %d/%d/%d,%d:%d:%d.%*d,%[^,],%[^,],%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            &data->day,
            &data->month,
            &data->year,
            &data->hour,
            &data->minute,
            &data->second,
            data->lat,
            data->lng,
            &data->altitude,
            &data->uncertainty,
            &data->speed,
            &data->direction,
            &data->vertical_acc,
            &data->sensor_used,
            &data->sv_used,
            &data->antenna_status,
            &data->jamming_status) ) > 0 ) {
            // UULOC Matched
            data->count = count;
            data->ok = true;
        }
    }
    return WAIT;
}

int cell_locate(MDM_CELL_LOCATE& loc, uint32_t timeout_ms, uint16_t accuracy) {
    loc.count = 0;
    loc.ok = false;
    if (RESP_OK == Cellular.command(5000, "AT+ULOCCELL=0\r\n")) {
        if (RESP_OK == Cellular.command(_cbLOCATE, &loc, timeout_ms, "AT+ULOC=2,2,1,%d,%d\r\n", timeout_ms/1000, accuracy)) {
            cell_locate_timeout_set(timeout_ms);
            if (loc.count > 0) {
                return loc.count;
            }
            return 0;
        }
        else {
            return -2;
            // Serial.println("Error! No Response from AT+LOC");
        }
    }
    // Serial.println("Error! No Response from AT+ULOCCELL");
    return -1;
}

bool cell_locate_in_progress(MDM_CELL_LOCATE& loc) {
    if (!is_cell_locate_matched(loc) && !is_cell_locate_timeout()) {
        return true;
    }
    else {
        cell_locate_timeout_clear();
        return false;
    }
}

bool cell_locate_get_response(MDM_CELL_LOCATE& loc) {
    // Send empty string to check for URCs that were slow
    Cellular.command(_cbLOCATE, &loc, 1000, "");
    if (loc.count > 0) {
        return true;
    }
    return false;
}

void cell_locate_display(MDM_CELL_LOCATE& loc) {
    /* The whole kit-n-kaboodle */
    Serial.printlnf("\r\n%d/%d/%d,%d:%d:%d,LAT:%s,LONG:%s,%d,UNCERTAINTY:%d,SPEED:%d,%d,%d,%d,%d,%d,%d,MATCHED_COUNT:%d",
        loc.month,
        loc.day,
        loc.year,
        loc.hour,
        loc.minute,
        loc.second,
        loc.lat,
        loc.lng,
        loc.altitude,
        loc.uncertainty,
        loc.speed,
        loc.direction,
        loc.vertical_acc,
        loc.sensor_used,
        loc.sv_used,
        loc.antenna_status,
        loc.jamming_status,
        loc.count);

    /* A nice map URL */
    Serial.printlnf("\r\nhttps://www.google.com/maps?q=%s,%s\r\n",loc.lat,loc.lng);
}
