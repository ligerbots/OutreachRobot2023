// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateTurret extends InstantCommand {
  private Rotation2d m_rotationDiff;
  private final Turret m_turret;

  public RotateTurret(Turret turret, Rotation2d angleDiff) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
    m_turret = turret;
    m_rotationDiff = angleDiff;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turret.setAngle(m_turret.getAngle().plus(m_rotationDiff));
  }
}
