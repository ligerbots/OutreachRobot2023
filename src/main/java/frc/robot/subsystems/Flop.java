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

public class Flop extends SubsystemBase {
  private CANSparkMax m_flopMotor;
  private final double m_speed = 1.00; //TODO: find working voltage values
  
  /** Creates a new Intake. */
  public Flop() {
    m_flopMotor = new CANSparkMax(Constants.FLOP_MOTOR_CAN_ID, MotorType.kBrushless);
    m_flopMotor.setIdleMode(IdleMode.kBrake);
  }

  //Transfer methods
  public void run() {
    m_flopMotor.set(m_speed);
  }

  public void stop() {
    m_flopMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Transfer onTrue(ExampleCommand exampleCommand) {
    return null;
  }
}