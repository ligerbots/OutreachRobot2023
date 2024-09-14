// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretCommand extends Command {
  private Turret m_turret;
  private double m_angle;
  /** Creates a new CarouselCommand. */
  public TurretCommand(Turret turret, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    m_angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setTurretAngle(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
