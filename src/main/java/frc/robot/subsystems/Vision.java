package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public AprilTagFieldLayout aprilTag_FieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
}
