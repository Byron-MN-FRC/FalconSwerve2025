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

public class AlignRotationallyWithWall extends Command {
    
    private CommandSwerveDrivetrain m_Drivetrain;
    private AlignmentSubsystem m_groupOfCANRanges;

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    private final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(2.5, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(3, 0, 0, OMEGA_CONSTRAINTS);
    
    private Translation2d goalTranslation;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    public AlignRotationallyWithWall (CommandSwerveDrivetrain drivetrain, AlignmentSubsystem alignmentSubsystem) {
         xController.setTolerance(0.0);
        yController.setTolerance(0.0);
        omegaController.setTolerance(Units.degreesToRadians(0.1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        m_Drivetrain = drivetrain;
        m_groupOfCANRanges = alignmentSubsystem;
        addRequirements(drivetrain, alignmentSubsystem);
    }

    
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("aligning", true);

        goalTranslation = Robot.getInstance().drivetrain.getState().Pose.getTranslation();

        omegaController.reset(m_groupOfCANRanges.getRelativeAngle().in(Radians));
        yController.reset(m_Drivetrain.getState().Pose.getY());
        xController.reset(m_Drivetrain.getState().Pose.getX());
        
    }

    @Override
    public void execute() {
                                    
        // Drive
        xController.setGoal(goalTranslation.getX());
        yController.setGoal(goalTranslation.getY());
        omegaController.setGoal(0);

        // Drive to the target
        double xSpeed = xController.calculate(m_Drivetrain.getState().Pose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }

        double ySpeed = yController.calculate(m_Drivetrain.getState().Pose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }

        double omegaSpeed = omegaController.calculate(m_groupOfCANRanges.getRelativeAngle().in(Radians));
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        // SmartDashboard.putNumber("ySpeed", ySpeed);
        // SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("omegaTurnySpeed", omegaSpeed);

        Optional<Alliance> ally = DriverStation.getAlliance();

            if (ally.get() == Alliance.Blue) {
                m_Drivetrain.setControl(
                Robot.getInstance().drive
                // .withVelocityX(xSpeed * MaxSpeed)
                // .withVelocityY(ySpeed * MaxSpeed)
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-omegaSpeed * MaxAngularRate)
                );
            } else {
                m_Drivetrain.setControl(
                Robot.getInstance().drive
                // .withVelocityX(-xSpeed * MaxSpeed)
                // .withVelocityY(-ySpeed * MaxSpeed)
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-omegaSpeed * MaxAngularRate)
                );
            }        

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("aligning", false);
    }
    
    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
    }
    
}