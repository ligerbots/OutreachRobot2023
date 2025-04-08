// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
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

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        
        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    private void configureBindings() {
        m_driverController.rightBumper().whileTrue(new StartEndCommand(m_transfer::intake, m_transfer::stop, m_transfer));
        m_driverController.leftBumper().whileTrue(new StartEndCommand(m_transfer::outtake, m_transfer::stop, m_transfer));

        m_driverController.rightTrigger().onTrue(new Shoot(m_shooter));
    }
    
    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Right stick X axis -> rotation
        // note: "rightBumper()"" is a Trigger which is a BooleanSupplier
        return new Drive(
                m_driveTrain,
                () -> -modifyAxis(m_driverController.getLeftY()),
                () -> -modifyAxis(m_driverController.getRightX()));
    }
    
    private static double modifyAxis(double value) {
        // Deadband
        value = MathUtil.applyDeadband(value, 0.05);
        
        // Square the axis, but preserve the sign
        value = value * Math.abs(value);
        
        return value;
    }
}
