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

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  public boolean backwards;
  public CANSparkMax m_turretMotor;
  private final RelativeEncoder m_turretMotorEncoder;
  private final double TURRET_GEAR_REDUCTION = 1.0 / 162.5;
  private final double RADIANS_PER_REVOLUTION = 2 * Math.PI * TURRET_GEAR_REDUCTION;

  //PID Stuff
  private SparkMaxPIDController m_pidController;
  private static double K_P = 1.0; //TODO: Tune
  private static double K_I = 0.0;
  private static double K_D = 0.0;
  private static double K_FF = 1.0;

  /** Creates a new Turret. */
  public Turret() {
    //super(null); //TODO: put constraints in here if we're changing this to trapezoidal
    m_turretMotor = new CANSparkMax(Constants.TURRET_MOTOR_CAN_ID, MotorType.kBrushless);
    m_turretMotor.restoreFactoryDefaults();
    m_turretMotorEncoder = m_turretMotor.getEncoder();
    m_turretMotorEncoder.setPositionConversionFactor(RADIANS_PER_REVOLUTION); //All future angle refrences will be based off this
    m_pidController = m_turretMotor.getPIDController(); //Use `m_pidController.setReference(<Angle>, ControlType.kPosition);`
    m_pidController.setP(K_P);
    m_pidController.setI(K_I);
    m_pidController.setD(K_D);
    m_pidController.setFF(K_FF);
  }

  //protected void useState(TrapezoidProfile.State setPoint) {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setTurretAngle(double angle) {
    m_pidController.setReference(angle, ControlType.kPosition);
  }

  public double getPosition() {
    return m_turretMotorEncoder.getPosition();
  }

  public void spin(double d) {}

  public void resetBallCount() {}
}
