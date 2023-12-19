package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    public double getDistance() {
        return 0;
    }

    public void setMode(VisionMode goalfinder) {}

    public enum VisionMode {
        INTAKE,                 // driver view through Intake camera
        SHOOTER,                // driver view through Shooter camera
        GOALFINDER,
        BALLFINDER,
        HOPPERFINDER,
        GALACTIC_SEARCH_PATH_CHOOSER,
    }

    public double getRobotAngle() {
        return 0;
    }
}
