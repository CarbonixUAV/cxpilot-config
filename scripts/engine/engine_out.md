# Engine Failsafe Script

This script provides a best-effort return mechanism in case of a forward motor
loss on an aircraft. Upon detecting engine failure, characterized by drops in
RPM and vibration below certain thresholds, the script initiates a series of
actions to safely handle the situation.

When an engine failure is detected, the script will 
  - Send a warning to the ground station.
  - Prioritize airspeed and set an optimal target speed for glide.
  - Optionally, it will initiate an automatic landing at the the nearest rally
    point or to the home position.

The script also implements prearm checks to ensure the glide speed parameter is
configured before flight and that the engine is running before takeoff.

## Gliding

When the script configures the TECS for gliding, `THR_MAX` is set to 0. This is
better than setting the speed-weight, as the latter causes the TECS to set the
"underspeed" flag, which locks the airspeed to minimum instead of the optimal
glide.

This also has the side-effect of throttling down to idle instead of max,
which only makes a difference in a false-positive engine-out detection. In my
opinion, this is safer, as all subsequent actions taken by the script assume
zero thrust, and strange things could happen if the engine was stuck running at
full throttle.

Additionally, the following parameters are set:
  - `Q_ASSIST_SPEED` is enabled if not already enabled
  - `Q_ASSIST_ALT` is set to `Q_RTL_ALT`
  - `TERRAIN_FOLLOW` is enabled to allow `Q_ASSIST_ALT` to trigger on AGL
    instead of above-home altitude
  - `Q_OPTIONS`
    - Disable "level transition" if enables, to allow smooth banking turns
      during Q Assist
    - Set "disable approach". The approach phase in QRTL doesn't allow the
      target position to be overridden, which is needed by the script.
  - `AIRSPEED_MIN` is lowered down to match `ENGOUT_QAST_GSPD`. Lowering this
    prevents getting stuck permanently in Q Assist if an angle assist occurs.
  - `AIRSPEED_CRUISE` is set to `ENGOUT_GLIDE_SPD` to prioritize maintaining the
    optimal glide speed. The exception is that during Q Assist, the airspeed is
    `ENGOUT_QAST_GSPD` to facilitate airbraking in the final spiral of the
    landing.

## Failsafe Actions

If enabled with the `ENGOUT_FS_ENABLE` parameter, the script will initiate a
series of actions to attempt to glide to, and automatically land on, the nearest
pre-designated safe landing zones (marked by rally points). The actions are as
follows:
  - Parameters are reconfigured:
    - `RTL_AUTOLAND`, if enabled, is disabled so that the aircraft will begin to
      make its way to the nearest rally point or home position.
    - `Q_ASSIST_ALT` is set to match `Q_RTL_ALT`
    - `Q_ASSIST_SPEED`, if disabled, will be set to 0.1 to ensure that
      `Q_ASSIST_ALT` and `Q_ASSIST_ANGLE` will work.
    - `Q_TRANS_FAIL` is set to 0 to prevent triggering QLand after a long period
      of Q Assist.
  - The mode will change to RTL.
  - If the aircraft reaches the landing area with altitude to spare, it will
    orbit until it gets low enough to activate Q Assist.
  - Once Q Assist is engaged, either in an orbit or en route, it will coasts
    until it stops making enough progress, then switch to QLand or QRTL
    depending on distance to the landing point.
  - As the ground speed reduces, the script adjusts the loiter radius to spiral
    inward toward the landing point. This simultaneously reduces the time to
    reach the landing point, makes for a less jarring switch to QRTL, and also
    helps initiate the switch while pointing roughly into the wind.
  - If QRTL takes unexpectedly long, the script will switch to QLand.
  - If Q Assist is on for an unexpectedly long time, the script will switch to
    QRTL/QLand depending on the distance to the landing point. This a last-ditch attempt to handle any bugs or edge cases.

During the failsafe, if you decide that there is a better landing location, you
can override the target location by changing to Guided mode. The script will
switch back to RTL immediately, but will override the destination with the
location of the Guided command.

## Configuration

The script is pre-configured with default parameters that I believe are
appropriate for most medium-to-large fixed-wing VTOL aircraft with a single
internal combustion engine and a few minutes of endurance in hover. The only
parameter which *must* be set up manually prior to use is `ENGOUT_GLIDE_SPD`.

You may want to adjust a few others depending on your aircraft.

### `ENGOUT_GLIDE_SPD`
Set this to the airspeed at which your aircraft glides most efficiently.

### `ENGOUT_QAST_GSPD`
I recommend you set this to your Q_WP_SPEED parameter, or maybe a little higher,
like 150%. This is the groundspeed threshold where the script will switch over
to QLand or QRTL.

### `ENGOUT_VIB_THRSH`
This check is to prevent a false positive engine-out detection caused by an
issue with RPM measurement. Check a log to see what vibration level is normal
for your aircraft. Err on the high side for this parameter. If your vibration
isolation is good, and your RPM sensor is reliable, you might want to just
disable this by setting it to a very high value like 10000.

### `ENGOUT_AUX_FUNC`
If you are very distrustful of the engine-out detection, you can set this to one
of the available scripting aux function values, and configure a switch or a
button on your GCS to override the detection. Low position will tell the script
that you are certain the engine is out, and high position will tell the script
that you are certain the engine is running. Middle position will let the script
decide as normal.

### `ENGOUT_FS_ENABLE`
If you just want the script to warn about an engine failure, and reconfigure the
aircraft for a glide, but not to attempt an automatic landing, set this to 0. No
other behaviors are implemented when this is disabled. RTL and QRTL will behave
however they are configured normally for your aircraft.
