// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private IntakeRollers m_intakeRollers;
  private IntakePivot m_intakePivot;
  
  /** Creates a new Intake. */
  public Intake() {
    m_intakeRollers = new IntakeRollers();
    m_intakePivot = new IntakePivot();
  }

  //IntakePivot methods
  public void deployIntake() {
    m_intakePivot.deployIntake();
  }

  public void retractIntake() {
    m_intakePivot.retractIntake();
  }

  public IntakePivotState getPivotState() {
    return m_intakePivot.getPivotState();
  }

  //IntakeRollers methods
  public void runIntakeRollers(double speed){
    m_intakeRollers.run(speed);
  }

  public void intake(){
    m_intakeRollers.intake();
  }

  public void outtake(){
    m_intakeRollers.outtake();
  }

  public void stopIntakeRollers(){
    m_intakeRollers.intake();
  }

  public double getIntakeRollersSpeed() {
    return m_intakeRollers.getSpeed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
