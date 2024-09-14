// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;

    /** Creates a new Intake. */

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
        // intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {
        // WHY negative? better to invert the motor and have the speed match what we want
        intakeMotor.set(-speed);
        // function to run the motor
    }

    public void IntakeBalls() {
        run(0.25);
        // experiment with the numbers
        // showing speed of motor when intaking balls
    }

    public void OutputBalls() {
        run(-0.25);
        // experiment with the numbers
        // showing the speed of motor when outputting balls
    }
}
