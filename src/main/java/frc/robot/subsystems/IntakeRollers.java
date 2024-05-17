// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {
  private CANSparkMax m_intakeMotor;
  private final double INTAKE_VOLTAGE = 0.25; //TODO: find working voltage
  private double speed;

  /** Creates a new IntakeRollers. */
  public IntakeRollers() {
    m_intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
  }

  public void run(double speed){
    m_intakeMotor.set(-speed);
    // function to run the motor
  }

  public void intake(){
    run(INTAKE_VOLTAGE);
    speed = INTAKE_VOLTAGE;
    // experiment with the numbers
    // showing speed of motor when intaking balls
  }

  public void outtake(){
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
