// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map.Entry;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
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

    TreeMap<Double, Double[]> m_distanceLookUp = new TreeMap<Double, Double[]>() {
    }; // set up lookup table for ranges
    TreeMap<Double, Double> m_turretAngleLookup = new TreeMap<Double, Double>() {
    };

    private static final double SHOOTER_STATOR_CURRENT_LIMIT = 40.0;

    // Constants for the shooter PID controller
    private static final double K_P = 0.15;
    private static final double K_FF = 0.0113;

    private static final double FLUP_SPEED = 0.5;

    // constructor
    public Shooter() {
        m_shooter = new TalonFX(Constants.SHOOTER_CAN_ID);

        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID Config, using Slot0
        Slot0Configs slot0configs = config.Slot0;
        slot0configs.kP = K_P;  // start small!!!
        slot0configs.kI = 0.0; // no output for integrated error
        slot0configs.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output
        slot0configs.kV = K_FF; // feed forward gain

        // Supply current limiting
        CurrentLimitsConfigs shooterCurrentLimitConfigs = config.CurrentLimits;
        shooterCurrentLimitConfigs.StatorCurrentLimit = SHOOTER_STATOR_CURRENT_LIMIT;
        shooterCurrentLimitConfigs.StatorCurrentLimitEnable = true;
        
        m_shooter.getConfigurator().apply(config, 0.1);

        m_flup = new SparkMax(Constants.SHOOTER_FLUP_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig configSparkMax = new SparkMaxConfig();
        configSparkMax.inverted(false);
        // always set a current limit
        // config.smartCurrentLimit(CURRENT_LIMIT);
        configSparkMax.idleMode(IdleMode.kBrake);

        m_flup.configure(configSparkMax, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.putNumber("shooter/setPoint", 0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/RPM", getSpeed());
        SmartDashboard.putNumber("shooter/current", m_shooter.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shooter/outputVoltage", m_shooter.getMotorVoltage().getValueAsDouble());
    }

    public double getVoltage() {
        return m_shooter.getSupplyVoltage().getValueAsDouble();
    }

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
        m_flup.set(FLUP_SPEED);
    }

    public void setShooterRpm(double rpm) {
        // speed expected in RPS, so convert
        m_shooter.setControl(new VelocityDutyCycle(rpm / 60.0));
        SmartDashboard.putNumber("shooter/setPoint", rpm);
    }

    public double calculateShooterSpeed(double distance) {
        Entry<Double, Double[]> floorEntry = m_distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = m_distanceLookUp.higherEntry(distance);

        if (floorEntry == null || ceilingEntry == null) {
            DriverStation.reportError("Shooter: floorEntry or ceilingEntry was null", false);
            // Typical speed. Not sure this will work for much, but it won't break anything.
            return 3000.0;
        }

        double ratio = MathUtil.inverseInterpolate(floorEntry.getKey(), ceilingEntry.getKey(), distance);
        double rpmTarget = MathUtil.interpolate(floorEntry.getValue()[0], ceilingEntry.getValue()[0], ratio);

        System.out.format("Shooter: ratio %3.2f, floor %4.1f, dist %4.1f, ceiling %4.1f, RPM %4.1f",
                ratio, floorEntry.getKey(), distance, ceilingEntry.getKey(), rpmTarget);
        return rpmTarget;
    }

    public boolean speedOnTarget(final double targetVelocity, final double percentAllowedError) {
        return Math.abs(getSpeed() - targetVelocity) < percentAllowedError * targetVelocity / 100.0;
    }

    public void stopAll() {
        m_shooter.set(0);
        m_flup.set(0);
        SmartDashboard.putNumber("shooter/setPoint", 0);
    }
}
