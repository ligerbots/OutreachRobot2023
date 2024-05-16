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

enum IntakePivotState {
  RETRACTED,
  DEPLOYED
}

public class IntakePivot extends TrapezoidProfileSubsystem {
  private CANSparkMax m_pivotMotor;
  private final RelativeEncoder m_pivotEncoder;
  private final double PIVOT_GEAR_REDUCTION = 1.0 / 33.0;
  private final double RADIANS_PER_REVOLUTION = 2 * Math.PI * PIVOT_GEAR_REDUCTION;
  private final double RETRACTED_ANGLE_RAD = 0.0; //TODO: find working values
  private final double DEPLOYED_ANGLE_RAD = 0.0; //TODO: find working values
  private IntakePivotState pivotState = IntakePivotState.RETRACTED; //TODO: verify that starting in retracted is reasonable

  //PID Stuff
  private SparkMaxPIDController m_pidController;
  private static double K_P = 1.0; //TODO: Tune
  private static double K_I = 0.0;
  private static double K_D = 0.0;
  private static double K_FF = 1.0;

  /** Creates a new IntakePivot. */
  public IntakePivot() {
    super(null); //TODO: put constraints in here
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

  protected void useState(TrapezoidProfile.State setPoint) {}

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
