// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final Intake m_intake = new Intake();
    private final DriveTrain m_driveTrain = new DriveTrain();
    private final Transfer m_transfer = new Transfer();
    private final Shooter m_shooter = new Shooter();
    private final Hood m_hood = new Hood();
    private final Turret m_turret = new Turret();

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Configure the trigger bindings
        configureBindings();
        
        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    private void configureBindings() {
        m_driverController.rightBumper().whileTrue(new StartEndCommand(m_transfer::intake, m_transfer::stop, m_transfer));
        m_driverController.leftBumper().whileTrue(new StartEndCommand(m_transfer::outtake, m_transfer::stop, m_transfer));
        m_driverController.rightTrigger().whileTrue(new StartEndCommand(m_transfer::intake, m_transfer::stop, m_transfer));
        m_driverController.leftTrigger().whileTrue(new StartEndCommand(m_transfer::outtake, m_transfer::stop, m_transfer));

        // m_driverController.rightTrigger().onTrue(new Shoot(m_shooter));
        m_driverController.y().onTrue(new Shoot(m_shooter, m_hood, m_transfer, 4500, Rotation2d.fromDegrees(60))); // VERY FAR SHOT
        m_driverController.x().onTrue(new Shoot(m_shooter, m_hood, m_transfer, 3500, Rotation2d.fromDegrees(55))); // Normal far lob shot
        m_driverController.b().onTrue(new Shoot(m_shooter, m_hood, m_transfer, 3500, Rotation2d.fromDegrees(55))); // Normal far lob shot
        m_driverController.a().onTrue(new Shoot(m_shooter, m_hood, m_transfer, 1750, Rotation2d.fromDegrees(45))); // Close shot

        m_driverController.povLeft().onTrue(new RotateTurret(m_turret, Rotation2d.fromDegrees(22.5)));
        m_driverController.povRight().onTrue(new RotateTurret(m_turret, Rotation2d.fromDegrees(-22.5)));
    }
    
    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Right stick X axis -> rotation
        return new Drive(
                m_driveTrain,
                () -> -modifyAxis(m_driverController.getLeftY()),
                () -> -modifyAxis(m_driverController.getLeftX()));
    }
    
    private static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, 0.05);
        
        // Square the axis, but preserve the sign
        value = value * Math.abs(value);
        
        return value;
    }
}
