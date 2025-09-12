// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonDropCoral;
import frc.robot.commands.DropCoral;
import frc.robot.commands.UndropCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {
    // Subsystems
        public final Claw m_claw = new Claw();
        public final Vision m_Vision = new Vision();
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        //  private CANdi wristAndClawCandi;
        //  private CANdi shoulderAndTopCandi;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        // 3/4 of a rotation per second max angular velocity
    private double percentSlow = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);
    // private final CommandXboxController characterizationJoystick = new CommandXboxController(2);


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* Fields */
    public Field2d field = new Field2d();
    public Field2d targetPoseField = new Field2d();

    //TODO reassign
    public int globalCurrNumSelected = 1;
    
    public RobotContainer() {
        
        NamedCommands.registerCommand("AutonDropCoral", new AutonDropCoral(m_claw));

        autoChooser = AutoBuilder.buildAutoChooser("L1 Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Field Widgets
        SmartDashboard.putData("Current Robot Position", field);
        SmartDashboard.putData("Target Robot Position", targetPoseField);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            field.setRobotPose(pose);
        });
        
        PathPlannerLogging.setLogTargetPoseCallback((pose) ->  {
            field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("path").setPoses(poses);
        });

        configureBindings();
    }

    private void configureBindings() {    
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed * percentSlow)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * percentSlow)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentSlow)
        ));
        // generated buttons that drivers will probably never use
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // Characterization buttons
        // Note that each routine should be run exactly once in a single log.
        // characterizationJoystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // characterizationJoystick.a().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // characterizationJoystick.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // characterizationJoystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Operator buttons
        joystick.y().onTrue(new InstantCommand(() -> slow()));
        joystick.start().onTrue(new InstantCommand(() -> m_Vision.tempDisable(0.5)).andThen(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())));
        joystick.leftTrigger().whileTrue(new DropCoral(m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        joystick.rightTrigger().whileTrue(new UndropCoral(m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));



        //Op Test Buttons TODO Reassign
        // joystick.b().whileTrue(
        //     new DriveToPosition(drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        // );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void slow() {
        if (percentSlow == 1) {
            percentSlow = Constants.SwerveConstants.percentSlow;
        } else {
            percentSlow = 1;
        }
    }

}
