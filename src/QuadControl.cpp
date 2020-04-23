#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

const int SCENARIO = 2;

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    if (SCENARIO == 1) {
        cmd.desiredThrustsN[0] = CONST_GRAVITY * mass / 4.f;
        cmd.desiredThrustsN[1] = CONST_GRAVITY * mass / 4.f;
        cmd.desiredThrustsN[2] = CONST_GRAVITY * mass / 4.f;
        cmd.desiredThrustsN[3] = CONST_GRAVITY * mass / 4.f;

//        cmd.desiredThrustsN[0] = 0;
//        cmd.desiredThrustsN[1] = 4.f;
//        cmd.desiredThrustsN[2] = 4.f;
//        cmd.desiredThrustsN[3] = 0;
    }
    else {
        // Scenario 2
        
        // [ 1  1  1  1] // c
        // [ 1 -1  1 -1] // roll
        // [ 1  1 -1 -1] // pitch
        // [-1  1  1 -1] // yaw

        
        // [ 1  1  1  1][thrust_1]   [c_bar]
        // [ 1 -1  1 -1][thrust_2] = [p_bar=Mx/l]
        // [ 1  1 -1 -1][thrust_3]   [q_bar=My/l]
        // [-1  1  1 -1][thrust_4]   [r_bar=-Mz/kappa]
        
        float l = L * M_SQRT1_2;
        
        float c_bar = collThrustCmd;
        float p_bar = momentCmd.x / l;
        float q_bar = momentCmd.y / l;
        float r_bar = -momentCmd.z / kappa; // thrust up, momentCmd is down
        
        // now solve the equation for thrust by multiplying the inverse of the 4x4 matrix
        
        // [thrust_1]   [c_bar]           [ 1/4  1/4  1/4  1/4]
        // [thrust_2] = [p_bar=Mx/l]   x  [ 1/4 -1/4  1/4 -1/4]
        // [thrust_3]   [q_bar=My/l]      [ 1/4  1/4 -1/4 -1/4]
        // [thrust_4]   [r_bar=Mz/kappa]  [ 1/4 -1/4 -1/4 -1/4]
        
        // apparently the system accepts thrust as sqrt(F/k_f) according to F = k_f * omega^2, so no need to calc actual F per rotor
        cmd.desiredThrustsN[0] = (c_bar + p_bar + q_bar + r_bar) / 4.f;
        cmd.desiredThrustsN[1] = (c_bar - p_bar + q_bar - r_bar) / 4.f;
        cmd.desiredThrustsN[2] = (c_bar + p_bar - q_bar - r_bar) / 4.f;
        cmd.desiredThrustsN[3] = (c_bar - p_bar - q_bar - r_bar) / 4.f;
        
    }
    /////////////////////////////// END STUDENT CODE ////////////////////////////
    
  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;
     
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    V3F pqrError = pqrCmd - pqr;
    V3F inertia = V3F(Ixx, Iyy, Izz);
    
    momentCmd = kpPQR * pqrError * inertia;

    
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    float c  = -collThrustCmd / mass; // F = ma, or a = F/m, convert force to accel
    
    // The rollpitch controller takes xy accelerations, attitude and thrust to generate commands for p, q
    // relationships between xy accel and thrust depend on b_x and b_y which are R13 and R23 in the rotation matrix
    // x_dot_dot = c * b_x=R(0,2)
    // y_dot_dot = c * b_y=R(1,2)
    
    // because we have our accelCmd for xy we know our b_c lets call it bCmd
    V3F bCmd = accelCmd / c;
    float bX = R(0, 2);
    float bXError = CONSTRAIN(bCmd.x, -maxTiltAngle, maxTiltAngle) - bX;
    // now basic P controller applies gain to error
    float bXCDot = kpBank * bXError;
    
    // do the same for y
    float bY = R(1, 2);
    float bYError = CONSTRAIN(bCmd.y, -maxTiltAngle, maxTiltAngle) - bY;
    float bYCDot = kpBank * bYError;
    
    // now use the rotation matrix to transform local accelerations to body rates
    
    pqrCmd.x = (R(1, 0) * bXCDot - R(0, 0) * bYCDot) / R(2, 2);
    pqrCmd.y = (R(1, 1) * bXCDot - R(0, 1) * bYCDot) / R(2, 2);
    pqrCmd.z = 0.f;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    float bZ = R(2,2);
    
    float posError = posZCmd - posZ;
    integratedAltitudeError += posError * dt;
    float velZError =  velZCmd - velZ;

    float pTerm = kpPosZ * posError;
    float dTerm = CONSTRAIN(kpVelZ * velZError, -maxAscentRate, maxDescentRate);
    float iTerm = KiPosZ * integratedAltitudeError;
    accelZCmd += pTerm + iTerm + dTerm;

    
    
    // formula for vertical accel command is:
    // accelZCmd = c * bZ + g
    // c = (accelZCmd - g) / bZ

    float thrustAcc = (CONST_GRAVITY - accelZCmd) / bZ;
    
    // convert accel to thrust (N) by adjusting for mass
    // F = ma
    thrust = thrustAcc * mass; // invert accel for NED

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    // We're using a standard PD controller with feedforward (accelCmd).
    // kd and kp are kpPosXY and kpVelXY
    
//    accelCmd.x += kpPosXY * (posCmd.x - pos.x) + kpVelXY * (CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY) - vel.x);
//    accelCmd.y += kpPosXY * (posCmd.y - pos.y) + kpVelXY * (CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY) - vel.y);
//
//    accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
//    accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
    velCmd.constrain(-maxSpeedXY,maxSpeedXY);
    accelCmd = kpPosXY * (posCmd - pos) + kpVelXY *(velCmd - vel);
    accelCmd.constrain(-maxAccelXY,maxAccelXY);
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}