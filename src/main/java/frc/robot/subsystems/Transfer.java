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

public class Transfer extends SubsystemBase {
  private CANSparkMax m_transferMotor;
  private final double m_speed = 1.00; //This is a voltage
  
  /** Creates a new Intake. */
  public Transfer() {
    m_transferMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    m_transferMotor.setIdleMode(IdleMode.kBrake);
  }

  //Transfer methods
  public void run() {
    m_transferMotor.set(m_speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Transfer onTrue(ExampleCommand exampleCommand) {
    return null;
  }
}