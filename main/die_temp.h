#pragma once

// The ESP32-S3's on-die temperature sensor. It measures the chip, not the panel
// or its power stage - those sit elsewhere on the board - so it answers "is the
// processor running hot" (a spinning task, a radio pinned in setup mode), not
// "is the charge pump cooking". Logged at boot and before every sleep, and
// reported by /api/system-info, so the question has numbers next time.
//
// Returns degrees Celsius, or NAN if the sensor could not be brought up.
float die_temp_read(void);
