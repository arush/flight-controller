# Controls Writeup

## Motor controls
We use the following equation for the motor thrusts. We solve for F's by inverting the matrix.
![Equation 1](https://snipboard.io/V6wrQu.jpg)

The inverted matrix looks like:
![invert](https://snipboard.io/07eiaL.jpg)

so the equation for motor thrusts is

```
// [thrust_1]   [c_bar]           [ 1/4  1/4  1/4  1/4]
// [thrust_2] = [p_bar=Mx/l]   x  [ 1/4 -1/4  1/4 -1/4]
// [thrust_3]   [q_bar=My/l]      [ 1/4  1/4 -1/4 -1/4]
// [thrust_4]   [r_bar=Mz/kappa]  [ 1/4 -1/4 -1/4 -1/4]
```

apparently the system accepts thrust as sqrt(F/k_f) according to F = k_f * omega^2, so no need to calc actual F per rotor

```
cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f;
cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f;
cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) / 4.f;
cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar - r_bar) / 4.f;
```

I don't understand why this is the case, but it works.

### Body Rate Controller
Body rate controller is a basic P controller. Just get the error from commanded rate and actual rate, multiply by the proportional gain kPQR and the moments of inertia of each axis.


### Roll Pitch Controller
The rollpitch controller takes xy accelerations, attitude and thrust to generate commands for p, q 

Relationships between xy accel and thrust depend on b_x and b_y which are R13 and R23 in the rotation matrix

### Lateral Controller
The lateral controller will use a PD controller with feedforward to command target values for elements of the drone's rotation matrix. The drone generates lateral acceleration by changing the body orientation which results in non-zero thrust in the desired direction. This will translate into the commanded rotation matrix elements 𝑏𝑥𝑐 and 𝑏𝑦𝑐. The control equations have the following form:
![lateral control formula](https://snipboard.io/rzT14c.jpg)
for the 𝑦 direction the control equations will have the same form as above.

### Altitude controller
This is a PID controller. The QuadController has an instance variable integratedAltitudeError to easily keep track of the iTerm over time (position error * dt).
The formula we need is p_term + i_term + d_term where

```
p_term = kPosZ * zErr
d_term = kVelZ * velZErr (remember to constrain this for min max rates)
i_term = kiZ * integratedAltitudeError
```

Then to get thrust as adjusted for all the additional thrusts coming from the roll pitch:
![altitude control formula](https://snipboard.io/VzM2jI.jpg)
where b_z is R33 in the rotation matrix.

```
float bZ = R(2,2);

float posError = posZCmd - posZ;
integratedAltitudeError += posError * dt;
float velZError =  CONSTRAIN(velZCmd - velZ, -maxAscentRate, maxDescentRate);

float pTerm = kpPosZ * posError;
float dTerm = kpVelZ * velZError;
float iTerm = KiPosZ * integratedAltitudeError;
accelZCmd += pTerm + iTerm + dTerm;

// formula for vertical accel command is:
// accelZCmd = c * bZ + g
// c = (accelZCmd - g) / bZ

float thrustAcc = (CONST_GRAVITY - accelZCmd) / bZ;

// convert accel to thrust (N) by adjusting for mass
// F = ma
thrust = thrustAcc * mass; // invert accel for NED
```

![Equation 2](https://snipboard.io/KyNo6A.jpg)

### Scenario 1
![Scenario 1](https://snipboard.io/pStP2l.jpg)

### Scenario 2
![Scenario 2](https://snipboard.io/tVi1Gg.jpg)

### Scenario 3
![Scenario 3](https://snipboard.io/jDgVnN.jpg)

### Scenario 4
![Scenario 4](https://snipboard.io/IPnhjD.jpg)

