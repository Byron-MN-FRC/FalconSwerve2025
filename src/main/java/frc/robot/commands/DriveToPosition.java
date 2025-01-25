
package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.TagApproaches;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
/**
 *
 */
public class DriveToPosition extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private String _limelightName = Constants.limeLightName;
/*old up */
    private final CommandSwerveDrivetrain drivetrain;
    /* old down */
    private Pose2d goalPose;

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0, .1, OMEGA_CONSTRAINTS);

    private int lastTarget;
/* old up */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    // private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    
    public DriveToPosition(CommandSwerveDrivetrain subsystem) {
        drivetrain = subsystem;
        /* old down */
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        /* old up */
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // // Set position of camera based on target seen 
        lastTarget = 0;
        // Verify that see a valid target for aliance and set current robot pose based on it.
          if (LimelightHelpers.getTV("")) {
            int fidID = (int) LimelightHelpers.getFiducialID(_limelightName);
            if ((fidID >= 0) && (fidID <= 22)) {
                lastTarget = fidID;
                
                // goalPose = TagApproaches.getInstance().DesiredRobotPos(lastTarget);
                goalPose = TagApproaches.getInstance().DesiredRobotPosN(22);

                SmartDashboard.putString("goal pose", goalPose.toString());
            }
        }

        omegaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
        yController.reset(drivetrain.getState().Pose.getY());
        xController.reset(drivetrain.getState().Pose.getX());

    }

    // // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (lastTarget == 0) {
            // drivetrain.stop();
        } else {

            // Drive
            xController.setGoal(goalPose.getX());
            yController.setGoal(goalPose.getY());
            omegaController.setGoal(goalPose.getRotation().getRadians());

            // Drive to the target
            var xSpeed = xController.calculate(drivetrain.getState().Pose.getX());
            if (xController.atGoal()) {
                xSpeed = 0;
            }

            var ySpeed = yController.calculate(drivetrain.getState().Pose.getY());
            if (yController.atGoal()) {
                ySpeed = 0;
            }

            var omegaSpeed = omegaController.calculate(drivetrain.getState().Pose.getRotation().getRadians());
            if (omegaController.atGoal()) {
                omegaSpeed = 0;
            }
 
    /*break old ^ */
        drivetrain.setControl(
            Robot.getInstance().drive.withVelocityX(xSpeed * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(ySpeed * MaxSpeed)
                .withRotationalRate(omegaSpeed * MaxAngularRate)
        );
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
//cancerl current trajectory
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}