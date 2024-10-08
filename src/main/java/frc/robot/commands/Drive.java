// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class Drive extends Command {
  /** Creates a new DriveCommand. */

  DriveTrain m_driveTrain;
  DoubleSupplier m_throttle;
  DoubleSupplier m_turn;

  public Drive(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier turn){
    m_driveTrain = driveTrain;
    m_throttle = throttle;
    m_turn = turn;

    addRequirements(m_driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.allDrive(m_throttle.getAsDouble(), m_turn.getAsDouble(), false);
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
