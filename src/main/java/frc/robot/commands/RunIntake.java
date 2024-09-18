// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command { //TODO: find a more suitable name for this
  Intake m_intake;
  boolean m_runForwards; // true = run forwards
  
  /** Creates a new RunIntake. */
  public RunIntake(Intake intake, boolean runForwards) {
    m_intake = intake;
    m_runForwards = runForwards;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_runForwards) {
      m_intake.intake();
    } else {
      m_intake.outtake();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntakeRollers();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}