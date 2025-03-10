package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
    
    private static final Vision m_Vision = new Vision();
    public boolean tempDisable = false;
    private boolean timestampToReEnable = 0;
    private String _limelightName = Constants.VisionConstants.limeLightName;

    public static Vision getInstance() {
        return m_Vision;
    }

    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace(_limelightName, 0.0275, -0.29845, 0.36195, 0, 0, 0);
    }

    @Override
    public void periodic() {
        if (timestampToReEnable < Utils.getCurrentTimeSeconds()){
            tempDisable = false;
        }
        SmartDashboard.putBoolean("tempDisable", tempDisable);
    }

    public Alliance MyAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        } else {
            return null;
        }
    }

    // public boolean AllianceTargetAquired() {
        // boolean targetAquired = LimelightHelpers.getTV(_limelightName);
        // if (targetAquired) {
        //     int targetID = (int) LimelightHelpers.getFiducialID(_limelightName);
        //     if ((targetID >= 0) && (targetID <= 16))
        //         return (MyAlliance() == _tagApproches.TagAlliance(targetID));
        //     else
        //         return false;
        // }
        // return false;
    // }

    public void tempDisable(double seconds) {
        tempDisable = true;
        double currentTime = Utils.getCurrentTimeSeconds();
        timestampToReEnable = currentTime + seconds;
    }
}