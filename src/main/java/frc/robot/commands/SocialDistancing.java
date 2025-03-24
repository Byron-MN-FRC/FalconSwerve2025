package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlignmentSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SocialDistancing extends Command {
    
    private CommandSwerveDrivetrain m_Drivetrain;
    private AlignmentSubsystem m_Range;

    private static final TrapezoidProfile.Constraints FORWARD_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints SIDE_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController forwardController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController sideController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0, 0, OMEGA_CONSTRAINTS);
    
    private Pose2d goalPose;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    public AlignRotationallyWithWall (CommandSwerveDrivetrain drivetrain, AlignmentSubsystem alignmentSubsystem, Pose2d acquiredTarget) {
         xController.setTolerance(0.0);
        yController.setTolerance(0.0);
        omegaController.setTolerance(Units.degreesToRadians(0.1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        m_Drivetrain = drivetrain;
        m_groupOfCANRanges = alignmentSubsystem;
        goalPose = acquiredTarget;
        addRequirements(drivetrain, alignmentSubsystem);
    }

    
    @Override
    public void initialize() {
        forwardController.reset(m_Range.getDistance().in(Units.inches));
        sideController.reset(0);
        omegaController.reset(m_drivetrain.getState().Pose.getRotation().getRadians());
    }

    @Override
    public void execute() {
                                    
        // Drive
        forwardController.setGoal(12);
        sideController.setGoal(0);
        omegaController.setGoal(goalPose.getRotation().getRadians());

        // Drive to the target
        double forwardSpeed = forwardController.calculate(m_Range.getDistance().in(Units.inches));
        if (forwardController.atGoal()) {
            forwardSpeed = 0;
        }

        double sideSpeed = sideController.calculate(0);
        if (sideController.atGoal()) {
            sideSpeed = 0;
        }

        double omegaSpeed = omegaController.calculate(m_drivetrain.getState().Pose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        Optional<Alliance> ally = DriverStation.getAlliance();

            if (ally.get() == Alliance.Blue) {
                m_Drivetrain.setControl(
                driveRobotCentric
                .withVelocityX(forwardSpeed * MaxAngularRate)
                .withVelocityY(0)
                .withRotationalRate(omegaSpeed * MaxAngularRate)
                );
            } else {
                m_Drivetrain.setControl(
                driveRobotCentric
                .withVelocityX(forwardSpeed * MaxAngularRate)
                .withVelocityY(0)
                .withRotationalRate(omegaSpeed * MaxAngularRate)
                );
            }        
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
    
}
