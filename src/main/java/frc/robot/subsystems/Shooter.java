// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    TalonFX m_shooter;
    // the thing that actually does the shooting
    SparkMax m_flup;

    //TODO: Replace with motor
    // Servo m_hoodServo;

    TreeMap<Double, Double[]> m_distanceLookUp = new TreeMap<Double, Double[]>() {
    }; // set up lookup table for ranges
    TreeMap<Double, Double> m_turretAngleLookup = new TreeMap<Double, Double>() {
    };
    DutyCycleEncoder m_dutyEncoder;

    private static final double SHOOTER_STATOR_CURRENT_LIMIT = 40.0;

    // PID Constants for the shooter PID controller
    private static final double K_P = 0.15;
    private static final double K_FF = 0.0113;

    // constructor
    public Shooter() {
        m_shooter = new TalonFX(Constants.SHOOTER_CAN_ID);

        // PID Configs
        TalonFXConfiguration config = new TalonFXConfiguration();
        // set slot 0 gains
        Slot0Configs slot0configs = config.Slot0;
        slot0configs.kP = K_P;  // start small!!!
        slot0configs.kI = 0.0; // no output for integrated error
        slot0configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
        slot0configs.kV = K_FF; // feed forward gain

        m_shooter.getConfigurator().apply(config, 0.1);

        // Supply current limiting
        CurrentLimitsConfigs shooterCurrentLimitConfigs = new CurrentLimitsConfigs();
        shooterCurrentLimitConfigs.StatorCurrentLimit = SHOOTER_STATOR_CURRENT_LIMIT;
        shooterCurrentLimitConfigs.StatorCurrentLimitEnable = true;

        m_shooter.getConfigurator().apply(shooterCurrentLimitConfigs);

        m_flup = new SparkMax(Constants.SHOOTER_FLUP_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig configSparkMax = new SparkMaxConfig();
        configSparkMax.inverted(false);
        // always set a current limit
        // config.smartCurrentLimit(CURRENT_LIMIT);
        configSparkMax.idleMode(IdleMode.kBrake);

        m_flup.configure(configSparkMax, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putNumber("shooter/setPoint", 0);

    }

    // called a lot
    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/RPM", getSpeed());
        SmartDashboard.putNumber("shooter/current", m_shooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooter/Output_Voltage", m_shooter.getMotorVoltage().getValueAsDouble());
    }

    public double getVoltage() {
        return m_shooter.getSupplyVoltage().getValueAsDouble();
    }

    // public void setHood(double angle) {
    //     System.out.println("hood angle SET!!!!");
    //     if (angle < 40) {
    //         angle = 40;
    //     }
    //     if (angle > 160) {
    //         angle = 160;
    //     }
    //     m_hoodServo.setAngle(angle);
    // }

    public double getSpeed() {
        return m_shooter.getVelocity().getValueAsDouble() * 60;
    }

    public void prepareShooter(double distance) {
        // Set the shooter and hood based on the distance
        setShooterRpm(calculateShooterSpeed(distance));
        // setHood(calculateShooterHood(distance));
    }

    // SHOOT!!!!!!!!!!
    public void shoot() {
        m_flup.set(0.5);
    }

    // does literally nothing
    public void setShooterRpm(double rpm) {
        m_shooter.setControl(new VelocityDutyCycle(rpm/60));
        SmartDashboard.putNumber("shooter/setPoint", rpm);
    }

    public double calculateShooterSpeed(double distance) {
        Entry<Double, Double[]> floorEntry = m_distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = m_distanceLookUp.higherEntry(distance);
        if (floorEntry != null && ceilingEntry != null) {
            // Charles' calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double rpmAdjustment = floorEntry.getValue()[0]
                    + ratio * (ceilingEntry.getValue()[0] - floorEntry.getValue()[0]);

            System.out.format("Shooter: ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, RPM %4.1f",
                    ratio, floorEntry.getKey(), distance, ceilingEntry.getKey(), rpmAdjustment);
            return rpmAdjustment;
        } else {
            System.out.println("Shooter: floorEntry or ceilingEntry was null");
            // Typical speed. Not sure this will work for much, but it won't break anything.
            return 4000;
        }
    }

    public double calculateShooterHood(double distance) {
        Entry<Double, Double[]> floorEntry = m_distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = m_distanceLookUp.higherEntry(distance);

        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double hoodAdjustment = floorEntry.getValue()[1]
                    + ratio * (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]);
            System.out.format(" hood %3.0f%n", hoodAdjustment);

            return hoodAdjustment;
        } else {
            return 60;
        }
    }

    public boolean speedOnTarget(final double targetVelocity, final double percentAllowedError) {
        return Math.abs(getSpeed() - targetVelocity) < percentAllowedError*targetVelocity/100.0;
    }

    // public boolean hoodOnTarget(final double targetAngle) {
    //     System.out.println("ServoPosition: " + m_hoodServo.getPosition());
    //     return m_hoodServo.getAngle() > targetAngle - 0.5 && m_hoodServo.getAngle() < targetAngle + 0.5;
    // }

    public void stopAll() {
        m_shooter.set(0);
        m_flup.set(0);
        // setHood(160);
        SmartDashboard.putString("shooter/Status", "Idle");
        SmartDashboard.putNumber("shooter/setPoint", 0);
    }
}
