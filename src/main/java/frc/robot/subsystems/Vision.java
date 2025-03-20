package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    
    private static final Vision m_Vision = new Vision();
    public boolean tempDisable = false;
    private double timestampToReEnable;
    private String _limelightName = Constants.VisionConstants.limeLightName;
    private Pose2d autoStartPose = new Pose2d();
 //   public Field2d autoPlacement = new Field2d();
    public int lastTargetFront;
    public int lastTargetBack;

    public static Vision getInstance() {
        return m_Vision;
    }

    public Vision() {
        LimelightHelpers.setCameraPose_RobotSpace(_limelightName, 0, -0.27845, 0.29, 0, 0, 0);
    }

    @Override
    public void periodic() {

        updateTargetData(Constants.VisionConstants.limeLightName);
        updateTargetData(Constants.VisionConstants.limeLightName2);

        SmartDashboard.putNumber("lasttargetfront", lastTargetFront);
        SmartDashboard.putNumber("lasttargetback", lastTargetBack);
        if (timestampToReEnable < Utils.getCurrentTimeSeconds() && tempDisable == true){
            tempDisable = false;
        }

        // SmartDashboard.putBoolean("tempDisable", tempDisable);
        //SmartDashboard.putData("Auton Placement Field", autoPlacement);
        //   autoPlacement.setRobotPose(autoStartPose);
        //System.out.println("placementPosition: " + autoStartPose);
        SmartDashboard.putString("placementPosition: " , autoStartPose.toString());
        SmartDashboard.putString("currentPosition: ", Robot.getInstance().drivetrain.getState().Pose.toString());

        if (DriverStation.isAutonomous() && !DriverStation.isEnabled()) {
            // For auto set-up
            if (!autoStartPose.equals(new Pose2d())) {
                Translation2d currentT2D = Robot.getInstance().drivetrain.getState().Pose.getTranslation();
                double distance = autoStartPose.getTranslation().getDistance(currentT2D);
                // difference between the goal angle and current angle arccos(cos(a-b))
                double rot_distance = Math.acos(autoStartPose.getRotation().getCos() * 
                     Robot.getInstance().drivetrain.getState().Pose.getRotation().getCos() + 
                     autoStartPose.getRotation().getSin() * 
                     Robot.getInstance().drivetrain.getState().Pose.getRotation().getSin());

                SmartDashboard.putNumber("Auto config distance", distance);
                SmartDashboard.putNumber("Auto config rotation distance", rot_distance);
                if (distance < 0.1 && (Units.radiansToDegrees(rot_distance) < 12)) {
                    LimelightHelpers.setLEDMode_ForceOn(_limelightName);
                } else {
                    LimelightHelpers.setLEDMode_ForceOff(_limelightName);
                }
            } else {
                LimelightHelpers.setLEDMode_ForceOff(_limelightName);
            }
        }

    }

    public Alliance MyAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red ? Alliance.Red : Alliance.Blue;
        } else {
            return null;
        }
    }

    public void updateTargetData(String llName) {
        if (LimelightHelpers.getTV(llName)) {
            if (llName == Constants.VisionConstants.limeLightName) {
                lastTargetFront = ((int)LimelightHelpers.getFiducialID(llName));
            }
            if (llName == Constants.VisionConstants.limeLightName2) {
                lastTargetBack = ((int)LimelightHelpers.getFiducialID(llName));
            }
        }
    }

    public void tempDisable(double seconds) {
        tempDisable = true;
        double currentTime = Utils.getCurrentTimeSeconds();
        timestampToReEnable = currentTime + seconds;
    }

    public void updateAutoStartPosition(String autoName) {
        // Instant Command is the name of the "None" Auto
        if (!autoName.equals("InstantCommand")) {
            try {
                autoStartPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getStartingDifferentialPose(); 
            } catch (Exception e){
                System.out.println(e.getMessage());
                autoStartPose = new Pose2d();

            }
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                autoStartPose = FlippingUtil.flipFieldPose(autoStartPose);
            }
        } else {
            autoStartPose = new Pose2d();
        } 
    }
}