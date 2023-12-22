// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret;

public class TurnTurret extends CommandBase {
  /** Creates a new turretCommand. */
  private turret m_turret;
  private DoubleSupplier m_turretAngle;
  public TurnTurret(turret turretInstance, DoubleSupplier turretAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turretInstance;
    m_turretAngle = turretAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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
