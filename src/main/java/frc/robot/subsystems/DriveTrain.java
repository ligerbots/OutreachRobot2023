// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import java.beans.Encoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

//check what we should add in constants and what in subsystems

  private CANSparkMax m_leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
  private CANSparkMax m_rightMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
 //Do we need leader, follower?

  private DifferentialDrive m_differentialDrive;
  private DifferentialDriveOdometry m_odometry;

  private ADXRS450_Gyro m_gyro;
  //(Maria?, private AHRS m_navX;)

  //Sim collection??(ex.TalonFXSimCollection)
  //fieldSim?? gyroAngleSim??
  //Encoders
  //private Encoder m_leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0],Constants.LEFT_ENCODER_PORTS[1])
  //private Encoder m_rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0],Constants.RIGHT_ENCODER_PORTS[1])

  //PID
  private SparkMaxPIDController m_leftController;
  private SparkMaxPIDController m_rightController;

  private static double K_P = 0.0; // change the (tune it)
  private static double K_I = 0.0;
  private static double K_D = 0.0;
  private static double K_FF = 0.0;

  private final double MAX_SPEED_METERS_PER_SECOND = 10.0;
  /** Creates a new DriveTrain. */


  public DriveTrain() {

    m_leftMotor = new CANSparkMax(Constants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(Constants.RIGHT_MOTOR_CAN_ID,MotorType.kBrushless);

    //PID
    m_leftController.setP(K_P);
    m_leftController.setP(K_I);
    m_leftController.setP(K_D);
    m_leftController.setP(K_FF);

    m_rightController.setP(K_P);
    m_rightController.setP(K_I);
    m_rightController.setP(K_D);
    m_rightController.setP(K_FF);
  }

  @Override
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

}
