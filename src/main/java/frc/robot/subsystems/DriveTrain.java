// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

//import java.beans.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS; //(ASK later)
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C.Port;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;


public class DriveTrain extends SubsystemBase {
//check what we should add in constants and what in subsystems

private CANSparkMax leftMotor = new CANSparkMax(MOTOR_LEFT_CAN_ID, MotorType.kBrushless);
private CANSparkMax rightMotor = new CANSparkMax(MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);
private final static int MOTOR_LEFT_CAN_ID = 0;
private final static int MOTOR_RIGHT_CAN_ID = 0;

// public PIDController turnSpeedController;
public double turnOutput;

DifferentialDrive robotDrive;
DifferentialDriveOdometry odometry;

Encoder leftEncoder; //= new Encoder(LEFT_ENCODER_PORTS);//(LOOK AT THIS!!)
Encoder rightEncoder; //= new Encoder(RIGHT_ENCODER_PORTS);
//private final static int[] LEFT_ENCODER_PORTS = new int[]{0, 1};
//private final static int[] RIGHT_ENCODER_PORTS = new int[]{2,3};

AHRS m_navX; //(ASK LATER!!!)

double limitedThrottle;

/*SIMULATION(CHECK IF WE NEED IT)
public DifferentialDrivetrainSim drivetrainSimulator;
private EncoderSim leftEncoderSim;
private EncoderSim rightEncoderSim;
// The Field2d class simulates the field in the sim GUI. Note that we can have only one
// instance!
private Field2d fieldSim;
private SimDouble gyroAngleSim;
*/

private int prevBallLocation = 0;
private int prevStartLocation = 10; // ASK WHAT THIS IS FOR!!!

//PID
private SparkMaxPIDController m_leftController;
private SparkMaxPIDController m_rightController;

private static double K_P = 1.0;
private static double K_I = 0.0;
private static double K_D = 0.0; 
private static double K_FF = 1.0;

private static final double DISTANCE_PER_PULSE = 0.0; //0.00155852448 

public DriveTrain() {
  m_leftController = leftMotor.getPIDController();
  m_leftController.setP(K_P);
  m_leftController.setI(K_I);
  m_leftController.setD(K_D);
  m_leftController.setFF(K_FF);
  m_rightController = rightMotor.getPIDController();
  m_rightController.setP(K_P);
  m_rightController.setI(K_I);
  m_rightController.setD(K_D);
  m_rightController.setFF(K_FF);

  
  robotDrive = new DifferentialDrive(leftMotor, rightMotor);
  robotDrive.setSafetyEnabled(false);

  //navX = new AHRSSimWrapper(Port.kMXP, (byte) 200); //(ASK!!!!)

  // Set current limiting on drve train to prevent brown outs
  Arrays.asList(leftMotor, rightMotor) //(ASK!!)
          .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

  // Set motors to brake when idle. We don't want the drive train to coast.
  Arrays.asList(leftMotor, rightMotor)
          .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

  //TODO determine real numbers to use here
        //rightLeader.setOpenLoopRampRate(0.0065);
        //leftLeader.setOpenLoopRampRate(0.0065);

  
  ////////////////////////////ODOMETRY SET UP//////////////////////////////////
  //(ASK!!!)

  leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
  rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
  

  odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), DISTANCE_PER_PULSE, DISTANCE_PER_PULSE);

  // turnSpeedController = new PIDController(0.015, 0.0001, 0.0, 0, navX, output -> this.turnOutput = output);
  
  /* 
  if (RobotBase.isSimulation()) {
    // If our robot is simulated
    // This class simulates our drivetrain's motion around the field.
    drivetrainSimulator = new DifferentialDrivetrainSim(
          Constants.kDrivetrainPlant,
          Constants.kDriveGearbox,
          Constants.kDriveGearing,
          Constants.kTrackwidth,
          Constants.kWheelDiameterMeters / 2.0);

    // The encoder and gyro angle sims let us set simulated sensor readings
    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    
    // get the angle simulation variable
    // SimDevice is found by name and index, like "name[index]"
    gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value + "]").getDouble("Angle");

    // the Field2d class lets us visualize our robot in the simulation GUI.
    fieldSim = new Field2d();

    SmartDashboard.putNumber("moveAroundField/startPos", prevStartLocation);
    SmartDashboard.putNumber("moveAroundField/ballPos", prevBallLocation);
  } */
}

public Pose2d getPose () {
  return odometry.getPoseMeters();
}


public void setPose(Pose2d pose) {
  /*if (RobotBase.isSimulation()) {
      // This is a bit hokey, but if the Robot jumps on the field, we need
      //   to reset the internal state of the DriveTrainSimulator.
      //   No method to do it, but we can reset the state variables.
      //   NOTE: this assumes the robot is not moving, since we are not resetting
      //   the rate variables.
      drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));

      // reset the GyroSim to match the driveTrainSim
      // do it early so that "real" odometry matches this value
      gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
      fieldSim.setRobotPose(pose);
  }*/
  // The left and right encoders MUST be reset when odometry is reset
  leftEncoder.reset();
  rightEncoder.reset();
  odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), 0.0, 0.0, pose);  //(ASK!!)
}

public void tankDriveVolts (double leftVolts, double rightVolts) {
  leftMotor.setVoltage(-leftVolts);
  rightMotor.setVoltage(rightVolts);// make sure right is negative becuase sides are opposite
  robotDrive.feed();
}

public double getAverageEncoderDistance() {
  return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
}

public double getLeftEncoderDistance() {
  return leftEncoder.getDistance();
}

public double getRightEncoderDistance() {
  return rightEncoder.getDistance();
}

public double getHeading() {
  return odometry.getPoseMeters().getRotation().getDegrees();
}

private double getGyroAngle() {
  return Math.IEEEremainder(m_navX.getAngle(), 360) * -1; // -1 wa put here for unknown reason look in documatation
}

public void resetOdometry (Pose2d pose) { // Same as setPose() but left here for compatibility
  setPose(pose);
  // old code as in master branch. leave here until tested
  // resetEncoders();
  // //resetHeading();
  // odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyroAngle()));

  // if (RobotBase.isSimulation()) {
  //     fieldSim.setRobotPose(pose);
  // }
}

public DifferentialDriveWheelSpeeds getWheelSpeeds () {
  return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
}

@Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getGyroAngle()), leftEncoder.getDistance(), rightEncoder.getDistance());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putString("Pose", getPose().toString());
    }

/*@Override
    public void simulationPeriodic() {
      // To update our simulation, we set motor voltage inputs, update the simulation,
      // and write the simulated positions and velocities to our simulated encoder and gyro.
      // We negate the right side so that positive voltages make the right side
      // move forward.
      drivetrainSimulator.setInputs(-leftMotors.get() * RobotController.getBatteryVoltage(),
                                    rightMotors.get() * RobotController.getBatteryVoltage());
      drivetrainSimulator.update(0.020);

      leftEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftPosition));
      leftEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kLeftVelocity));
  
      rightEncoderSim.setDistance(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightPosition));
      rightEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim.State.kRightVelocity));
  
      gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
  
      fieldSim.setRobotPose(getPose());
    }*/

    public void allDrive(double throttle, double rotate, boolean squaredInputs) {
      // TODO: We should look into using the deadband settings in DifferentialDrive
      if (squaredInputs) {
          if (Math.abs(throttle) < 0.1)
              throttle = 0;
          if (Math.abs(rotate) < 0.1) 
              rotate = 0;
      }
      robotDrive.arcadeDrive(throttle, -rotate, squaredInputs);
    }

    public int getLeftEncoderTicks () {
      return leftEncoder.get();
  }

  public int getRightEncoderTicks () {
      return rightEncoder.get();
  }

  public double turnSpeedCalc(double angleError) {
      if (Math.abs(angleError) > 60) {
          return 0.8 * Math.signum(angleError);
      }
      else if (Math.abs(angleError) > 30) {
          return 0.4 * Math.signum(angleError);
      }
      else if (Math.abs(angleError) > 10) {
          return 0.13 * Math.signum(angleError);
      }
      else if (Math.abs(angleError) > 5) {
          return 0.07 * Math.signum(angleError);
      }
      else {
          return 0.065 * Math.signum(angleError);
      }
  }
   
  public double getPitch() {
    return m_navX.getPitch();    //(ASK!!)
  }

public void setIdleMode(IdleMode idleMode) {
    if (Robot.isReal()) {
        Arrays.asList(leftMotor, rightMotor)
            .forEach((CANSparkMax spark) -> spark.setIdleMode(idleMode));
    }
}

/*public void moveAroundField() {
  // only applies for simulation
  if (RobotBase.isReal()) return;

  int startPos = (int)SmartDashboard.getNumber("moveAroundField/startPos", 10);
  int ballPos = (int)SmartDashboard.getNumber("moveAroundField/ballPos", 0);

  // Use either start of ball to set robot pose.
  // 10 is the dummy default value for start.
  if (startPos != prevStartLocation && startPos >= 0 && startPos < FieldMap.startPosition.length) {
      // The start value has changed and is valid. Use it to position the robot.
      fieldSim.setRobotPose(FieldMap.startPosition[startPos]);
  } else if (ballPos != prevBallLocation && ballPos >= 0 && ballPos < FieldMap.ballPosition.length) {
      // start value is invalid, so use the ball position with 0 rotation angle
      fieldSim.setRobotPose(new Pose2d(FieldMap.ballPosition[ballPos], new Rotation2d(0.0)));
  }

  prevBallLocation = ballPos;
  prevStartLocation = startPos;

  // On every call, output the Pose info to SmartDashboard for debugging convenience
  Pose2d pose = fieldSim.getRobotPose();
  SmartDashboard.putNumber("moveAroundField/robotX", pose.getX() / Constants.inchToMetersConversionFactor);
  SmartDashboard.putNumber("moveAroundField/robotY", pose.getY() / Constants.inchToMetersConversionFactor);
  SmartDashboard.putNumber("moveAroundField/robotAngle", pose.getRotation().getDegrees());
}
*/

/*public void setRobotFromFieldPose() {
  // only applies for simulation
  if (RobotBase.isSimulation())
      setPose(fieldSim.getRobotPose());
}*/

}








































  /*@Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLeftSpeed(double speedMetersPerSecond){
    m_leftController.setReference(limitSpeed(speedMetersPerSecond), ControlType.kVelocity);
  }

  public void setRightSpeed(double speedMetersPerSecond){
    m_rightController.setReference(limitSpeed(speedMetersPerSecond), ControlType.kVelocity);
  }
  
  private double limitSpeed(double speed){
    return MathUtil.clamp(speed, -MAX_SPEED_METERS_PER_SECOND, MAX_SPEED_METERS_PER_SECOND);
  }

  /* 
  public double getLeftSpeed(){
     return m_leftMotor.get();
  }
  public double getRightSpeed(){
    return -m_rightMotor.get();
  }
  */


