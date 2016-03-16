Tracker
===

Based on David Middlecamp's [fancy-asset-tracker](/dmiddlecamp/fancy-asset-tracker) demo for the Particle Electron with the Asset Tracker Shield.

Functionality
====

The basic implementation provides a combined GPS and Mobile network location system that will:

- Wake up when accelerometer detects motion
- If more motion is detected
  - Reset 'no motion' shutdown timer
  - Initialise GPS module
  - Attempt to connect to Particle cloud

- If no status has been sent in more than 6 hours
  - Send `idle_checkin` status
- Read new NMEA sentences from the GPS module
- If we connected to the cloud but do NOT have a GPS fix
  - Retrieve an approximate location using the UBlox CellLocate feature (ULOC)
  - Use the GSM based location to hint the GPS module in hopes of a quicker fix timer using the PMTK740 and PMTK741 commands

- Publish the most accurate location we have to the Particle cloud.
  - If we have a GPS fix then use that, otherwise submit the approximate GSM location

- If GPS module has not gained a fix in 10 minutes (accelerometer will keep the tracker running permanently if it is in motion)
  - Turn off the GPS module for 10 minutes to save battery (yay active antenna)
  
- TODO: If battery SoC is less than 20%
  - Disable accelerometer based interrupts that keep the device awake
  - Rely on idle checkin every 6 hours (or longer? configurable maybe) to report location

- If we've not detected motion in more than 3 minutes
  - Put the Electron into deep sleep mode


TODO
====
- Implement low-battery 'emergency' mode that only reports location every <x> hours
- Implement ability to deep sleep the tracker for <x> hours with no motion activated wakeup, e.g. when tracked device is being used by owner. Proximity based maybe?


