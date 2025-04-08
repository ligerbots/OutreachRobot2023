// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transfer extends SubsystemBase {
    private CANSparkMax m_transferMotor;
    private static final double SPEED = 0.30;

    /** Creates a new Intake. */
    public Transfer() {
        m_transferMotor = new CANSparkMax(Constants.TRANSFER_MOTOR_CAN_ID, MotorType.kBrushless);
        m_transferMotor.setIdleMode(IdleMode.kBrake);
        m_transferMotor.setInverted(true);
    }
    
    // Transfer methods
    public void intake() {
        m_transferMotor.set(SPEED);
    }
    
    public void outtake() {
        m_transferMotor.set(-SPEED);
    }

    public void stop() {
        m_transferMotor.set(0);
    }
    
    // @Override
    // public void periodic() {
    //     // This method will be called once per scheduler run
    // }
}