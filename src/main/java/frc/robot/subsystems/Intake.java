// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ExampleCommand;

enum IntakePivotState {
  RETRACTED,
  DEPLOYED
}

public class Intake extends SubsystemBase {
  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_pivotMotor;
  private final double INTAKE_VOLTAGE = 0.25;
  private double speed;
  private final double PIVOT_GEAR_REDUCTION = 1.0 / 33.0;
  private final double RADIANS_PER_REVOLUTION = 2 * Math.PI * PIVOT_GEAR_REDUCTION;
  private final RelativeEncoder m_pivotEncoder;
  private final double RETRACTED_ANGLE_RAD = 0.0; //TODO: find right values
  private final double DEPLOYED_ANGLE_RAD = 0.0; //TODO: find right values
  private IntakePivotState pivotState = IntakePivotState.RETRACTED; //TODO: verify that starting in retracted is reasonable

  //PID Stuff
  private SparkMaxPIDController m_pidController;
  private static double K_P = 1.0; //TODO: Tune
  private static double K_I = 0.0;
  private static double K_D = 0.0;
  private static double K_FF = 1.0;
  /** Creates a new Intake. */
  
  public Intake() {
    m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    
    //Pivot motor setup:
    m_pivotMotor = new CANSparkMax(Constants.INTAKE_PIVOT_MOTOR_CAN_ID, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder.setPositionConversionFactor(RADIANS_PER_REVOLUTION); //All future angle refrences will be based off this
    m_pidController = m_pivotMotor.getPIDController(); //Use `m_pidController.setReference(<Angle>, ControlType.kPosition);`
    m_pidController.setP(K_P);
    m_pidController.setI(K_I);
    m_pidController.setD(K_D);
    m_pidController.setFF(K_FF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void run(double speed){
    m_intakeMotor.set(-speed);
    // function to run the motor
  }

  public void IntakeBalls(){
    run(INTAKE_VOLTAGE);
    speed = INTAKE_VOLTAGE;
    // experiment with the numbers
    // showing speed of motor when intaking balls
  }

  public void OutputBalls(){
    run(INTAKE_VOLTAGE * -1); 
    speed = INTAKE_VOLTAGE * -1;
    // experiment with the numbers
    // showing the speed of motor when outputting balls
  }

  public void StopIntake(){
    run(0);
    speed = 0.0;
  }

  public double getSpeed() {
    return speed;
  }

  private void setPivotAngle(double angle) {
    m_pidController.setReference(angle, ControlType.kPosition);
  }

  public void deployIntake() {
    setPivotAngle(DEPLOYED_ANGLE_RAD);
    pivotState = IntakePivotState.DEPLOYED;
  }

  public void retractIntake() {
    setPivotAngle(RETRACTED_ANGLE_RAD);
    pivotState = IntakePivotState.RETRACTED;
  }

  public IntakePivotState getPivotState() {
    return pivotState;
  }

  public Intake onTrue(ExampleCommand exampleCommand) {
    return null;
  }
}
