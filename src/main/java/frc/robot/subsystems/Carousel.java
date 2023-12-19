// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carousel extends SubsystemBase {
  public boolean backwards;

/** Creates a new Carousel. */
  public Carousel() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(int i) {
  }

public int getTicks() {
    return 0;
}

public void spin(double d) {
}

public void resetBallCount() {
}
}
