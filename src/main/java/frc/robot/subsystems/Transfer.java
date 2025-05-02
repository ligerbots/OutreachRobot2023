// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transfer extends SubsystemBase {
    private SparkMax m_transferMotor;
    private static final double SPEED = 0.30;

    /** Creates a new Intake. */
    public Transfer() {
        m_transferMotor = new SparkMax(Constants.TRANSFER_MOTOR_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        // always set a current limit
        // config.smartCurrentLimit(CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);

        m_transferMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
}