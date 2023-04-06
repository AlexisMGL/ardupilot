#include "AP_Parachute.h"

#if HAL_PARACHUTE_ENABLED

#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_Parachute, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:First Relay,1:Second Relay,2:Third Relay,3:Fourth Relay,10:Servo
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is not released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in meters above home
    // @Description: Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
    // @Range: 0 32000
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    // @Param: DELAY_MS
    // @DisplayName: Parachute release delay
    // @Description: Delay in millseconds between motor stop and chute release
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DELAY_MS", 5, AP_Parachute, _delay_ms, AP_PARACHUTE_RELEASE_DELAY_MS),
    
    // @Param: CRT_SINK
    // @DisplayName: Critical sink speed rate in m/s to trigger emergency parachute
    // @Description: Release parachute when critical sink rate is reached
    // @Range: 0 15
    // @Units: m/s
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRT_SINK", 6, AP_Parachute, _critical_sink, AP_PARACHUTE_CRITICAL_SINK_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Parachute options
    // @Description: Optional behaviour for parachute
    // @Bitmask: 0:hold open forever after release
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 7, AP_Parachute, _options, 0),

    // @Param: VTOL_SK
    // @DisplayName: Critical sink speed rate in vtol
    // @Description: release parachute when VTOL Critical sink rate is reached
    // @Range: 0 15
    // @Units: m/s
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("VTOL_SK", 8, AP_Parachute, _VTOL_critical_sink, 5.0f),

    // @Param: VTOL_SK_T
    // @DisplayName: Time needed under VTOL_CRT_SINK to release parachute
    // @Description: decision to release parachute is taken after this time
    // @Range: 0 1100
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("VTOL_SK_T", 9, AP_Parachute, _VTOL_sink_time, 425),

    // @Param: CRT_TBC
    // @DisplayName: critical time before crash
    // @Description: parachute is released when estimated time before crash is lower than critical time before crash during enough time
    // @Range: 0 22500
    // @Units: ms
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("CRT_TBC", 10, AP_Parachute, _critical_TBC, 11225),

    // @Param: LOOP_TMAX
    // @DisplayName: In cruise, time needed in sink state to release parachute at high altitude
    // @Description: parachute is released when estimated time before crash is lower than critical time before crash during loop_tmax ms
    // @Range: 1000 4250
    // @Units: ms
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("LOOP_TMAX", 11, AP_Parachute, _loop_Tmax, 2000),

    // @Param: LOOP_TMIN
    // @DisplayName: In cruise, time needed in sink state to release parachute at low altitude
    // @Description: parachute is released when estimated time before crash is lower than critical time before crash during loop_tmin ms
    // @Range: 0 2250
    // @Units: ms
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("LOOP_TMIN", 12, AP_Parachute, _loop_Tmin, 425),

    // @Param: ALT_TMAX
    // @DisplayName: In cruise, altitude where Tmax is reached
    // @Description: the time tolerance in sink state is higher at high altitude and reach LOOP_TMAX at ALT_TMAX
    // @Range: 60 800
    // @Units: m
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ALT_TMAX", 13, AP_Parachute, _alt_Tmax, 120),

    // @Param: ALT_TMIN
    // @DisplayName: In cruise, altitude where Tmin is reached
    // @Description: the time tolerance in sink state is lower at low altitude and reach LOOP_TMIN at ALT_TMIN
    // @Range: 0 100
    // @Units: m
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ALT_TMIN", 14, AP_Parachute, _alt_Tmin, 40),

    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled = on_off;

    // clear release_time
    _release_time = 0;

    AP::logger().Write_Event(_enabled ? LogEvent::PARACHUTE_ENABLED : LogEvent::PARACHUTE_DISABLED);
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Released");
    AP::logger().Write_Event(LogEvent::PARACHUTE_RELEASED);

    // set release time to current system time
    if (_release_time == 0) {
        _release_time = AP_HAL::millis();
    }

    _release_initiated = true;

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;
}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return;
    }

    // calc time since release
    uint32_t time_diff = AP_HAL::millis() - _release_time;
    uint32_t delay_ms = _delay_ms<=0 ? 0: (uint32_t)_delay_ms;

    bool hold_forever = (_options.get() & uint32_t(Options::HoldOpen)) != 0;

    // check if we should release parachute
    if ((_release_time != 0) && !_release_in_progress) {
        if (time_diff >= delay_ms) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_on_pwm);
            } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            }
            _release_in_progress = true;
            _released = true;
        }
    } else if ((_release_time == 0) ||
               (!hold_forever && time_diff >= delay_ms + AP_PARACHUTE_RELEASE_DURATION_MS)) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
        } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _release_in_progress = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}

// set_sink_rate - set vehicle sink rate
void AP_Parachute::set_sink_rate(float sink_rate)
{
    // reset sink time if critical sink rate check is disabled or vehicle is not flying
    if ((_critical_sink <= 0) || !_is_flying) {
        _sink_time_ms = 0;
        return;
    }

    // reset sink_time if vehicle is not sinking too fast
    if (sink_rate <= _critical_sink) {
        _sink_time_ms = 0;
        return;
    }

    // start time when sinking too fast
    if (_sink_time_ms == 0) {
        _sink_time_ms = AP_HAL::millis();
    }
}

void AP_Parachute::set_sink_rate_edit(float sink_rate,float relative_alt_parachute_m,bool in_vtol)
{
    // derive ETBC and avoids divions/0 issues
    if (sink_rate*sink_rate < 0.25){
        estimated_time_before_crash_ms = 1000.0f*relative_alt_parachute_m/0.5;
    }
    else{
        estimated_time_before_crash_ms = 1000.0f*relative_alt_parachute_m/sink_rate;
    } 

    
    if (_is_flying){
        float log_etbc = estimated_time_before_crash_ms;
        if (log_etbc < 0 || log_etbc > 40000){
            log_etbc = 40000.0f; //make the FPAR logs easier to read 
        }
        if (_sink_time_ms_edit != 0){
            AP::logger().Write("FPAR","TimeUS,ETBC_s,sink_time,loop_time,AGL,SR","Qfffff",AP_HAL::micros64(),log_etbc,(AP_HAL::millis() - _sink_time_ms_edit)*1.0f,loop_time_ms*1.0f,relative_alt_parachute_m,sink_rate);
        }
        else{
        AP::logger().Write("FPAR","TimeUS,ETBC_s,sink_time,loop_time,AGL,SR","Qfffff",AP_HAL::micros64(),log_etbc,0*1.0f,loop_time_ms*1.0f,relative_alt_parachute_m,sink_rate); 
        }
    }

    // reset sink time if critical sink rate check is disabled or vehicle is not flying
    if ((_critical_sink <= 0) || !_is_flying || relative_alt_parachute_m < 10) {
        _sink_time_ms_edit = 0;
        return;
    }

    
    // reset sink_time if vehicle is not sinking too fast
    if (in_vtol){
        if (sink_rate <= _VTOL_critical_sink){
            _sink_time_ms_edit = 0;
            return;
        }
    }

    else{
        if(estimated_time_before_crash_ms < 0 || estimated_time_before_crash_ms >= _critical_TBC || _critical_TBC <= 0){
            _sink_time_ms_edit = 0;
            return;
        }

    }

   // start time when sinking too fast
    if (_sink_time_ms_edit == 0) {
        _sink_time_ms_edit = AP_HAL::millis();
    }

    if(in_vtol){
        loop_time_ms =_VTOL_sink_time; 
    }
    else{
        uint32_t loop_tmin = _loop_Tmin;
        uint32_t loop_tmax = _loop_Tmax; //convert _loop_tmin and _loop_tmax to uint to be compared to uint
        uint32_t k_parachute_law = (_loop_Tmax-_loop_Tmin)/(_alt_Tmax-_alt_min); // k is in  ms / m
        uint32_t zero_h_parachute_law = _loop_Tmax - k_parachute_law*_alt_Tmax; // in ms
        loop_time_ms = k_parachute_law*relative_alt_parachute_m + zero_h_parachute_law;
        if (loop_time_ms < loop_tmin){
            loop_time_ms = _loop_Tmin;
        }
        if (loop_time_ms > loop_tmax){
            loop_time_ms = _loop_Tmax;
        }
    }
}

// trigger parachute release if sink_rate is below critical_sink_rate for 1sec
void AP_Parachute::check_sink_rate()
{
    // return immediately if parachute is being released or vehicle is not flying
    if (!_is_flying) {
        return;
    }

    if ((_sink_time_ms_edit > 0) && ((AP_HAL::millis() - _sink_time_ms_edit) > loop_time_ms)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Fictive Parachute released");
    }

    if (_release_initiated) {
        return;
    }    

    // if vehicle is sinking too fast for more than a second release parachute
    if ((_sink_time_ms > 0) && ((AP_HAL::millis() - _sink_time_ms) > 1000)) {
        release();
    }

}

// check settings are valid
bool AP_Parachute::arming_checks(size_t buflen, char *buffer) const
{
    if (_enabled > 0) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_parachute_release)) {
                hal.util->snprintf(buffer, buflen, "Chute has no channel");
                return false;
            }
        } else if (!_relay.enabled(_release_type)) {
            hal.util->snprintf(buffer, buflen, "Chute invalid relay %d", int(_release_type));
            return false;
        }
        if (_release_initiated) {
            hal.util->snprintf(buffer, buflen, "Chute is released");
            return false;
        }
    }
    return true;
}

// singleton instance
AP_Parachute *AP_Parachute::_singleton;

namespace AP {

AP_Parachute *parachute()
{
    return AP_Parachute::get_singleton();
}

}
#endif // HAL_PARACHUTE_ENABLED
