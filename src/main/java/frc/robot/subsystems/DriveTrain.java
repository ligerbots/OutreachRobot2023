// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

// import com.revrobotics.CANSparkBase.IdleMode;

//import java.beans.Encoder;
// import com.revrobotics.CANSparkMaxLowLevel;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import edu.wpi.first.hal.SimDouble;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.proto.Wpimath;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.I2C.Port;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.EncoderSim;
// import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
import com.kauailabs.navx.frc.AHRS; //(ASK later)
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.util.Units;


public class DriveTrain extends SubsystemBase {
  // check what we should add in constants and what in subsystems
  // CANSparkMax(int, CANSparkLowLevel.MotorType)
  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
  

  // public PIDController turnSpeedController;
  private double m_turnOutput;

  private DifferentialDrive m_robotDrive;
  private DifferentialDriveOdometry m_odometry;

  private RelativeEncoder m_leftEncoder; // = new Encoder(LEFT_ENCODER_PORTS);//(LOOK AT THIS!!)
  private RelativeEncoder m_rightEncoder; // = new Encoder(RIGHT_ENCODER_PORTS);
  // private final static int[] LEFT_ENCODER_PORTS = new int[]{0, 1};
  // private final static int[] RIGHT_ENCODER_PORTS = new int[]{2,3};

  AHRS m_navX; 

  double m_limitedThrottle;

  /*
   * SIMULATION(CHECK IF WE NEED IT)
   * public DifferentialDrivetrainSim drivetrainSimulator;
   * private EncoderSim leftEncoderSim;
   * private EncoderSim rightEncoderSim;
   * // The Field2d class simulates the field in the sim GUI. Note that we can
   * have only one
   * // instance!
   * private Field2d fieldSim;
   * private SimDouble gyroAngleSim;
   */


  // PID
  private SparkPIDController m_leftController;
  private SparkPIDController m_rightController;

  private static double K_P = 1.0;
  private static double K_I = 0.0;
  private static double K_D = 0.0;
  private static double K_FF = 1.0;


  private static final double DRIVE_GEAR_REDUCTION = 1/19.527; //TODO: Double check number
  private static final double METER_PER_REVOLUTION = (Units.inchesToMeters(8)*Math.PI) * DRIVE_GEAR_REDUCTION;

  public DriveTrain() {
    m_leftController = m_leftMotor.getPIDController();
    m_leftController.setP(K_P);
    m_leftController.setI(K_I);
    m_leftController.setD(K_D);
    m_leftController.setFF(K_FF);
    m_rightController = m_rightMotor.getPIDController();
    m_rightController.setP(K_P);
    m_rightController.setI(K_I);
    m_rightController.setD(K_D);
    m_rightController.setFF(K_FF);

    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_robotDrive.setSafetyEnabled(false);


    // Set current limiting on drve train to prevent brown outs
    //Arrays.asList(m_leftMotor, m_rightMotor) 
        //.forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));
    
    m_leftMotor.setSmartCurrentLimit(35);
    m_rightMotor.setSmartCurrentLimit(35);
    

    // Set motors to brake when idle. We don't want the drive train to coast.
    //Arrays.asList(m_leftMotor, m_rightMotor)
        //.forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

    m_leftMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
    m_rightMotor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);

    // TODO determine real numbers to use here
    // rightLeader.setOpenLoopRampRate(0.0065);
    // leftLeader.setOpenLoopRampRate(0.0065);


    m_leftEncoder.setPositionConversionFactor(METER_PER_REVOLUTION);
    m_rightEncoder.setPositionConversionFactor(METER_PER_REVOLUTION);

    //FIXME: Not sure what the second two arguments do, from looking at the docs they seem unnessary
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0), 0, 0);

    // turnSpeedController = new PIDController(0.015, 0.0001, 0.0, 0, navX, output
    // -> this.m_turnOutput = output);

    /*
     * if (RobotBase.isSimulation()) {
     * // If our robot is simulated
     * // This class simulates our drivetrain's motion around the field.
     * drivetrainSimulator = new DifferentialDrivetrainSim(
     * Constants.kDrivetrainPlant,
     * Constants.kDriveGearbox,
     * Constants.kDriveGearing,
     * Constants.kTrackwidth,
     * Constants.kWheelDiameterMeters / 2.0);
     * 
     * // The encoder and gyro angle sims let us set simulated sensor readings
     * leftEncoderSim = new EncoderSim(m_leftEncoder);
     * rightEncoderSim = new EncoderSim(m_rightEncoder);
     * 
     * // get the angle simulation variable
     * // SimDevice is found by name and index, like "name[index]"
     * gyroAngleSim = new SimDeviceSim("AHRS[" + SPI.Port.kMXP.value +
     * "]").getDouble("Angle");
     * 
     * // the Field2d class lets us visualize our robot in the simulation GUI.
     * fieldSim = new Field2d();
     * 
     * SmartDashboard.putNumber("moveAroundField/startPos", prevStartLocation);
     * SmartDashboard.putNumber("moveAroundField/ballPos", prevBallLocation);
     * }
     */
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    /*
     * if (RobotBase.isSimulation()) {
     * // This is a bit hokey, but if the Robot jumps on the field, we need
     * // to reset the internal state of the DriveTrainSimulator.
     * // No method to do it, but we can reset the state variables.
     * // NOTE: this assumes the robot is not moving, since we are not resetting
     * // the rate variables.
     * drivetrainSimulator.setState(new Matrix<>(Nat.N7(), Nat.N1()));
     * 
     * // reset the GyroSim to match the driveTrainSim
     * // do it early so that "real" odometry matches this value
     * gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
     * fieldSim.setRobotPose(pose);
     * }
     */
    // The left and right encoders MUST be reset when odometry is reset
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    m_odometry.resetPosition(Rotation2d.fromDegrees(getGyroAngle()), 0.0, 0.0, pose); 
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(-leftVolts);
    m_rightMotor.setVoltage(rightVolts);// make sure right is negative becuase sides are opposite
    m_robotDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  public double getLeftEncoderDistance() {
    return m_leftEncoder.getPosition();
  }

  public double getRightEncoderDistance() {
    return m_rightEncoder.getPosition();
  }

  public double getHeading() {
    return m_odometry.getPoseMeters().getRotation().getDegrees();
  }

  private double getGyroAngle() {
    return Math.IEEEremainder(m_navX.getAngle(), 360) * -1; // -1 wa put here for unknown reason look in documatation
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getGyroAngle()), getLeftEncoderDistance(), getRightEncoderDistance());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putString("Pose", getPose().toString());
  }

  /*
   * @Override
   * public void simulationPeriodic() {
   * // To update our simulation, we set motor voltage inputs, update the
   * simulation,
   * // and write the simulated positions and velocities to our simulated encoder
   * and gyro.
   * // We negate the right side so that positive voltages make the right side
   * // move forward.
   * drivetrainSimulator.setInputs(-m_leftMotor.get() *
   * RobotController.getBatteryVoltage(),
   * m_rightMotors.get() * RobotController.getBatteryVoltage());
   * drivetrainSimulator.update(0.020);
   * 
   * leftEncoderSim.setDistance(drivetrainSimulator.getState(
   * DifferentialDrivetrainSim.State.kLeftPosition));
   * leftEncoderSim.setRate(drivetrainSimulator.getState(DifferentialDrivetrainSim
   * .State.kLeftVelocity));
   * 
   * rightEncoderSim.setDistance(drivetrainSimulator.getState(
   * DifferentialDrivetrainSim.State.kRightPosition));
   * rightEncoderSim.setRate(drivetrainSimulator.getState(
   * DifferentialDrivetrainSim.State.kRightVelocity));
   * 
   * gyroAngleSim.set(-drivetrainSimulator.getHeading().getDegrees());
   * 
   * fieldSim.setRobotPose(getPose());
   * }
   */

  public void allDrive(double throttle, double rotate, boolean squaredInputs) {
    // TODO: We should look into using the deadband settings in DifferentialDrive
    if (squaredInputs) {
      if (Math.abs(throttle) < 0.1)
        throttle = 0;
      if (Math.abs(rotate) < 0.1)
        rotate = 0;
    }
    m_robotDrive.arcadeDrive(throttle, -rotate, squaredInputs);
  }

  // public int getLeftEncoderTicks() {
  //   return m_leftEncoder.get();
  // }

  // public int getRightEncoderTicks() {
  //   return m_rightEncoder.get();
  // }

  public double turnSpeedCalc(double angleError) {
    if (Math.abs(angleError) > 60) {
      return 0.8 * Math.signum(angleError);
    } else if (Math.abs(angleError) > 30) {
      return 0.4 * Math.signum(angleError);
    } else if (Math.abs(angleError) > 10) {
      return 0.13 * Math.signum(angleError);
    } else if (Math.abs(angleError) > 5) {
      return 0.07 * Math.signum(angleError);
    } else {
      return 0.065 * Math.signum(angleError);
    }
  }

  public double getPitch() {
    return m_navX.getPitch(); 
  }

  public void setIdleMode(IdleMode idleMode) {
    if (Robot.isReal()) {
      Arrays.asList(m_leftMotor, m_rightMotor)
          .forEach((CANSparkMax spark) -> spark.setIdleMode(idleMode));
    }
  }

  /*
   * public void moveAroundField() {
   * // only applies for simulation
   * if (RobotBase.isReal()) return;
   * 
   * int startPos = (int)SmartDashboard.getNumber("moveAroundField/startPos", 10);
   * int ballPos = (int)SmartDashboard.getNumber("moveAroundField/ballPos", 0);
   * 
   * // Use either start of ball to set robot pose.
   * // 10 is the dummy default value for start.
   * if (startPos != prevStartLocation && startPos >= 0 && startPos <
   * FieldMap.startPosition.length) {
   * // The start value has changed and is valid. Use it to position the robot.
   * fieldSim.setRobotPose(FieldMap.startPosition[startPos]);
   * } else if (ballPos != prevBallLocation && ballPos >= 0 && ballPos <
   * FieldMap.ballPosition.length) {
   * // start value is invalid, so use the ball position with 0 rotation angle
   * fieldSim.setRobotPose(new Pose2d(FieldMap.ballPosition[ballPos], new
   * Rotation2d(0.0)));
   * }
   * 
   * prevBallLocation = ballPos;
   * prevStartLocation = startPos;
   * 
   * // On every call, output the Pose info to SmartDashboard for debugging
   * convenience
   * Pose2d pose = fieldSim.getRobotPose();
   * SmartDashboard.putNumber("moveAroundField/robotX", pose.getX() /
   * Constants.inchToMetersConversionFactor);
   * SmartDashboard.putNumber("moveAroundField/robotY", pose.getY() /
   * Constants.inchToMetersConversionFactor);
   * SmartDashboard.putNumber("moveAroundField/robotAngle",
   * pose.getRotation().getDegrees());
   * }
   */

  /*
   * public void setRobotFromFieldPose() {
   * // only applies for simulation
   * if (RobotBase.isSimulation())
   * setPose(fieldSim.getRobotPose());
   * }
   */

}

/*
 * @Override
 * public void periodic() {
 * // This method will be called once per scheduler run
 * }
 * 
 * public void setLeftSpeed(double speedMetersPerSecond){
 * m_leftController.setReference(limitSpeed(speedMetersPerSecond),
 * ControlType.kVelocity);
 * }
 * 
 * public void setRightSpeed(double speedMetersPerSecond){
 * m_rightController.setReference(limitSpeed(speedMetersPerSecond),
 * ControlType.kVelocity);
 * }
 * 
 * private double limitSpeed(double speed){
 * return MathUtil.clamp(speed, -MAX_SPEED_METERS_PER_SECOND,
 * MAX_SPEED_METERS_PER_SECOND);
 * }
 * 
 * /*
 * public double getLeftSpeed(){
 * return m_leftMotor.get();
 * }
 * public double getRightSpeed(){
 * return -m_rightMotor.get();
 * }
 */
