AP_GPS::

init();

// typically 10 hz or more
update();

// the time we last processed a message in milliseconds. This is
// used to indicate that we have new GPS data to process
last_message_time_ms();

// the expected lag (in seconds) in the position and velocity readings from the gps
// return true if the GPS hardware configuration is known or the lag parameter has been set manually
get_lag();

// return a 3D vector defining the offset of the GPS antenna in meters relative to the body frame origin
Vector3f &get_antenna_offset();

// returns the desired gps update rate in milliseconds
// this does not provide any guarantee that the GPS is updating at the requested
// rate it is simply a helper for use in the backends for determining what rate
// they should be configuring the GPS to run at
uint16_t get_rate_ms(uint8_t instance) const;

// update one GPS instance. This should be called at 10Hz or greater
void AP_GPS::update_instance(uint8_t instance)

// return a 3D vector defining the offset of the GPS antenna in meters relative to the body frame origin
const Vector3f &AP_GPS::get_antenna_offset(uint8_t instance) const

// Acquire location
const Location &loc = gps.location();
