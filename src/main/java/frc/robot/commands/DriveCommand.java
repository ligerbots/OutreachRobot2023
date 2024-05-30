// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */

  DriveTrain driveTrain;
  DoubleSupplier throttle;
  DoubleSupplier turn;

  public DriveCommand(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn){
    this.driveTrain = driveTrain;
    this.throttle = throttle;
    this.turn = turn;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("left encoder", driveTrain.getLeftEncoderTicks());
    SmartDashboard.putNumber("right encoder", driveTrain.getRightEncoderTicks());
    driveTrain.allDrive(throttle.getAsDouble(), turn.getAsDouble(), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
