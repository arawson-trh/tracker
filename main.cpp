#pragma SPARK_NO_PREPROCESSOR

#include <math.h>
#include "math.h"
#include <ctype.h>

// Particle application functions
#include "application.h"
#include "cellular_hal.h"

// GPS
#include "inc/Adafruit_GPS.h"
#include "inc/Adafruit_LIS3DH.h"
#include "inc/GPS_Math.h"

// Cellular Location
#include "inc/cell_locate.h"

#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);
Adafruit_LIS3DH accel = Adafruit_LIS3DH(A2, A5, A4, A3);
FuelGauge fuel;

SerialDebugOutput debugOutput;

#define PREFIX "t/"
#define CLICKTHRESHHOLD 20


SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

MDM_CELL_LOCATE _cell_locate;

unsigned long lastCellLocation = 0;
unsigned long lastMotion = 0;
unsigned long lastPublish = 0;
unsigned long lastMMessage = 0;

time_t lastIdleCheckin = 0;

bool GPS_ACTIVE   = false;
bool PUBLISH_MODE = true; // Publish by default

// publish after x seconds
unsigned int PUBLISH_DELAY = (120 * 1000);

// retrieve cell location after x seconds IF NO FIX
unsigned int CELL_LOCATION_DELAY = (60 * 1000);
unsigned int CELL_LOCATION_TIMEOUT = (10 * 1000);
unsigned int CELL_LOCATION_REQ_ACCURACY = 100;
unsigned int CELL_LOCATION_IGNORE_ACCURACY = 5000;

// if no motion for 3 minutes, sleep! (milliseconds)
unsigned int NO_MOTION_IDLE_SLEEP_DELAY = (3 * 60 * 1000);

// lets wakeup every 6 hours and check in (seconds)
unsigned int HOW_LONG_SHOULD_WE_SLEEP = (6 * 60 * 60);

// When manually asked to sleep
unsigned int SLEEP_TIME = 3600; // Sleep for an hour by default

// when we wakeup from deep-sleep not as a result of motion,
// how long should we wait before we publish our location?
// lets set this to less than our sleep time, so we always idle check in.
// (seconds)
int MAX_IDLE_CHECKIN_DELAY = (HOW_LONG_SHOULD_WE_SLEEP - 60);


void initAccel() {
    accel.begin(LIS3DH_DEFAULT_ADDRESS);

    // Default to 5kHz low-power sampling
    accel.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);

    // Default to 4 gravities range
    accel.setRange(LIS3DH_RANGE_4_G);

    // listen for single-tap events at the threshold
    // keep the pin high for 1s, wait 1s between clicks

    //uint8_t c, uint8_t clickthresh, uint8_t timelimit, uint8_t timelatency, uint8_t timewindow
    accel.setClick(1, CLICKTHRESHHOLD);//, 0, 100, 50);
}


bool gpsActivated() {
    return GPS_ACTIVE;
}


void deactivateGPS() { 
    Serial.println("GPS: Disabling power to GPS shield...");

    // Hey GPS, please stop using power, kthx.
    digitalWrite(D6, HIGH);
    GPS_ACTIVE = false;
}


void activateGPS() {
    Serial.println("GPS: Enabling power to GPS shield...");

    // electron asset tracker shield needs this to enable the power to the gps module.
    pinMode(D6, OUTPUT);
    digitalWrite(D6, LOW);

    Serial.println("GPS: Starting serial ports...");
    GPS.begin(9600);
    GPSSerial.begin(9600);

    Serial.println("GPS: Requesting hot restart...");
    //# request a HOT RESTART, in case we were in standby mode before.
    GPS.sendCommand("$PMTK101*32");
    delay(250);

    // request everything!
    Serial.println("GPS: Setting data format for ALLDATA...");
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
    delay(250);

    // turn off antenna updates
    Serial.println("GPS: Turning off antenna updates...");
    GPS.sendCommand(PGCMD_NOANTENNA);
    delay(250);

    GPS_ACTIVE = true;
}


int publishMode(String mode);
int forceSleep(String seconds);


void setup() {
    lastMotion = 0;
    lastMMessage = 0;
    lastPublish = 0;
    lastCellLocation = 0;

    initAccel();

    // for blinking.
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);

    Serial.begin(9600);

    Particle.function("publish", publishMode);
    Particle.function("sleep", forceSleep);
}


int publishMode(String mode) {
    PUBLISH_MODE = (mode == "enabled") ? true : false;
    return 1;
}


// Define sleep timer function
bool doSleep() {
    Serial.println("Sleeping...");
    System.sleep(SLEEP_MODE_DEEP, SLEEP_TIME);
}


int forceSleep(String seconds) {
    sscanf(seconds, "%u", SLEEP_TIME);

    Serial.printlnf("Sleeping in 10s for %u", SLEEP_TIME);
    Timer shutdown_timer(10000, doSleep, true);
    shutdown_timer.start();
    return 1;
}


void checkGPS() {
    if(gpsActivated()) {
        // process and dump everything from the module through the library.
        while (GPSSerial.available()) {
            char c = GPS.read();

            // lets echo the GPS output until we get a good clock reading, then lets calm things down.

            if (GPS.newNMEAreceived()) {
                GPS.parse(GPS.lastNMEA());
            }
        }
    } else {
        Serial.println("Attempted to check GPS but it isn't active!");
    }
}


/* Send hint commands to GPS module with cellular location fix
 * This should speed up time to fix in low signal strength situations.
 * Do not hint if we already have a lock (no need) */
void gpsHint(MDM_CELL_LOCATE& loc) {
    if(gpsActivated()) {
        if(is_cell_locate_accurate(loc,CELL_LOCATION_IGNORE_ACCURACY)) {
            String locationString = String::format("%s,%s,%d",
                loc.lat,
                loc.lng,
                loc.altitude
            );

            String timeString = String::format("%d,%d,%d,%d,%d,%d",
                loc.year,
                loc.month,
                loc.day,
                loc.hour,
                loc.minute,
                loc.second
            );

            String gpsTimeCmd = "PMTK740," + timeString;
            String locationTimeCmd = "PMTK741,"+locationString + "," + timeString;

            String cmd = String::format("$%s*%02x", gpsTimeCmd.c_str(), crc8(gpsTimeCmd));
            GPS.sendCommand(strdup(cmd.c_str()));
            cmd = String::format("$%s*%02x", locationTimeCmd.c_str(), crc8(locationTimeCmd));
            GPS.sendCommand(strdup(cmd.c_str()));
        }
    } else {
        Serial.println("gpsHint: Attempted to hint GPS but it isn't activated!");
    }
}


void publishLocation() {
    if(PUBLISH_MODE) {
        unsigned long now = millis();
        if (((now - lastPublish) > PUBLISH_DELAY) || (lastPublish == 0)) {
            lastPublish = now;

            unsigned int msSinceLastMotion = (now - lastMotion);
            int motionInTheLastMinute = (msSinceLastMotion < 60000);
            
            if(!GPS.fix && is_cell_locate_accurate(_cell_locate,CELL_LOCATION_IGNORE_ACCURACY)) {
                Serial.println("Publish: No GPS Fix, reporting Cellular location...");
                String loc_data =
                      "{\"lat\":"      + String(_cell_locate.lat)
                    + ",\"lon\":"      + String(_cell_locate.lng)
                    + ",\"a\":"        + String(_cell_locate.altitude)
                    + ",\"q\":"        + String(_cell_locate.uncertainty)
                    + ",\"t\":\"gsm\""
                    + ",\"spd\":"      + String(_cell_locate.speed) 
                    + ",\"mot\":"      + String(motionInTheLastMinute)
                    + ",\"s\": 0"
                    + ",\"vcc\":"      + String(fuel.getVCell())
                    + ",\"soc\":"      + String(fuel.getSoC())
                    + "}";
                Particle.publish(PREFIX + String("l"), loc_data, 60, PRIVATE);
            } else if (GPS.fix) {
                Serial.println("Publish: GPS Fix available, reporting...");
                String loc_data =
                      "{\"lat\":"      + String(convertDegMinToDecDeg(GPS.latitude))
                    + ",\"lon\":-"     + String(convertDegMinToDecDeg(GPS.longitude))
                    + ",\"a\":"        + String(GPS.altitude)
                    + ",\"q\":"        + String(GPS.fixquality)
                    + ",\"t\":\"gps\""
                    + ",\"spd\":"      + String(GPS.speed * 0.514444) 
                    + ",\"mot\":"      + String(motionInTheLastMinute)
                    + ",\"s\": "       + String(GPS.satellites)
                    + ",\"vcc\":"      + String(fuel.getVCell())
                    + ",\"soc\":"      + String(fuel.getSoC())
                    + "}";
                Particle.publish(PREFIX + String("l"), loc_data, 60, PRIVATE);
            } else {
                Particle.publish(PREFIX + String("s"), "no_fix", 60, PRIVATE);
            }
        }
    }
}


bool hasMotion() {
    bool motion = false;
    uint16_t cnt = 0;
    do {
        motion = digitalRead(WKP);
        digitalWrite(D7, (motion) ? HIGH : LOW);
        cnt++;
    }
    while(!motion && cnt < 500);
    
    return motion;
}


void checkMotion() {
    if(hasMotion()) {
        unsigned long now = millis();

        Serial.printlnf("checkMotion: BUMP - Setting lastMotion to %d", now);

        lastMotion = now;

        // Activate GPS module if inactive
        if(!gpsActivated()) {
            activateGPS();
        }

        if (Particle.connected() == false) {
            // Init GPS now, gives the module a little time to fix while particle connects
            Serial.println("checkMotion: Connecting...");
            Particle.connect();
            Particle.publish(PREFIX + String("s"), "motion_checkin", 1800, PRIVATE);
        }
    }
}


void idleCheckin() {
    // If we havent idle checked-in yet, set our initial timer.
    if (lastIdleCheckin == 0) {
        lastIdleCheckin = Time.now();
    }

    // use the real-time-clock here, instead of millis.
    if ((Time.now() - lastIdleCheckin) >= MAX_IDLE_CHECKIN_DELAY) {

        // Activate GPS module if inactive
        if(!gpsActivated()) {
            activateGPS();
        }

        // it's been too long!  Lets say hey!
        if (Particle.connected() == false) {
            Serial.println("idleCheckin: Connecting...");
            Particle.connect();
        }

        Particle.publish(PREFIX + String("s"), "idle_checkin", 1800, PRIVATE);
        lastIdleCheckin = Time.now();
    }
}


void checkCellLocation() {
    // Only request cell location when connected and with no GPS fix
    if (Particle.connected() == true) {
        if(!GPS.fix) {
            unsigned long now = millis();
            if ((now - lastCellLocation) >= CELL_LOCATION_DELAY || (lastCellLocation == 0)) {
                int ret = cell_locate(_cell_locate, CELL_LOCATION_TIMEOUT, CELL_LOCATION_REQ_ACCURACY);
                if (ret == 0) {
                    /* Handle non-instant return of URC */
                    while (cell_locate_in_progress(_cell_locate)) {
                        /* Since this loop might take a while, check for location between checking
                         * return of cell location 
                         */
                        checkMotion();

                        /* still waiting for URC */
                        cell_locate_get_response(_cell_locate);
                    }
                }
                else {
                    /* ret == -1 */
                    Serial.println("Cell Locate Error!");
                }
                lastCellLocation = millis();
                gpsHint(_cell_locate);
            }
        }
    }
}


void idleSleep(unsigned long now) {
    if ((now - lastMotion) > NO_MOTION_IDLE_SLEEP_DELAY) {
        Serial.printlnf("No motion in %d ms, sleeping...", (now - lastMotion));
        // hey, it's been longer than xx minutes and nothing is happening, lets go to sleep.
        // if the accel triggers an interrupt, we'll wakeup earlier than that.

        Particle.publish(PREFIX + String("s"), "sleeping", 1800, PRIVATE);

        lastPublish = 0;
        lastMotion = 0;
        lastCellLocation = 0;

        deactivateGPS();
        
        // lets give ourselves a chance to settle, deal with anything pending, achieve enlightenment...
        delay(10*1000);
        System.sleep(WKP, CHANGE, HOW_LONG_SHOULD_WE_SLEEP);

        // Set lastMotion to now on wake (only from non-deep sleep since deep will re-call setup()
        lastMotion = millis();
        //System.sleep(SLEEP_MODE_DEEP, HOW_LONG_SHOULD_WE_SLEEP);
    }
}


void loop() {

    unsigned long now = millis();

    /* If any of these timers are higher than now then something went weird
     * or timer rolled over ??? so... reset
     */ 
    if (lastMotion > now) { lastMotion = now; }
    if (lastPublish > now) { lastPublish = now; }
    if (lastCellLocation > now) { lastCellLocation = now; }

    /* Check to see if we've seen any motion
     * if so, enable GPS, connect, reset timers. This order gives time for 
     * GPS to start while we connect
     */
    checkMotion();

    /* If we havent idle checked in in a long time, do it now.
     * This enables GPS, connects, resets idle timer and sends
     * a location. 
     */
    idleCheckin();

    /* If GPS is activated, scan for new NMEA messages */
    checkGPS();

    /* If we don't have a GPS fix, scan for approximate
     * location using cellular location */
    checkCellLocation();

    /* Publish location if we haven't done so for at least
     * <interval> */
    publishLocation();

    if ((now - lastMMessage) > 10000) {
        Serial.printlnf("Sleeping in %ds due to no motion...", ((NO_MOTION_IDLE_SLEEP_DELAY - (now - lastMotion)) / 1000));
        lastMMessage = now;
    }

    // use "now" instead of millis...  If it takes us a REALLY long time to connect, we don't want to
    // accidentally idle out.
    idleSleep(now);

    delay(10);
}


