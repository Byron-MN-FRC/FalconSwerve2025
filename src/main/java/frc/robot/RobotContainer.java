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

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveToPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double percentSlow = 1;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // private final CommandXboxController characterizationJoystick = new CommandXboxController(1);

    public int globalCurrNumSelected = 1;
    public double GLOBALOFFSET = 0.0;



    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Vision m_Vision = new Vision();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    public Field2d field = new Field2d();
    public Field2d field2= new Field2d();

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Autonomous Command");
        SmartDashboard.putData("Auto Mode", autoChooser);

        field = new Field2d();
        SmartDashboard.putData("field", field);
        field2 = new Field2d();
        SmartDashboard.putData("field2", field2);
        SmartDashboard.putNumber("offsetTest", 0);
        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        //     field.setRobotPose(pose);
        // });
        
        // PathPlannerLogging.setLogTargetPoseCallback((pose) ->  {
        //     field.getObject("target pose").setPose(pose);
        // });

        // PathPlannerLogging.setLogActivePathCallback((poses) -> {
        //     field.getObject("path").setPoses(poses);
        // });
        
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * percentSlow) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * percentSlow) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentSlow) // Drive counterclockwise with negative X (left)
            )
        );

        /* Driver Buttons */

        // generated buttons that drivers will probably never use
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines.
        // Note that each routine should be run exactly once in a single log.
        // characterizationJoystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // characterizationJoystick.a().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // characterizationJoystick.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // characterizationJoystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.back().toggleOnTrue(new InstantCommand(() -> drivetrain.rotateOpPerspective()));
        drivetrain.registerTelemetry(logger::telemeterize);

        // slow mode
        joystick.back().onTrue(new InstantCommand(() -> slow()));

        //drive to position
        joystick.b().whileTrue(
            new DriveToPosition(drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
    
        joystick.leftBumper().onTrue(new InstantCommand(() -> minus()));
        joystick.rightBumper().onTrue(new InstantCommand(() -> plus()));
        joystick.x().onTrue(new InstantCommand(()-> toggleReefOffset()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void slow() {
        if (percentSlow == 1) {
            percentSlow = Constants.percentSlow;
        } else {
            percentSlow = 1;
        }
    }

    private void plus() {
        globalCurrNumSelected++;
    }

    private void minus() {
        if (globalCurrNumSelected > 1) {
            globalCurrNumSelected--;
        }
    }

    private void toggleReefOffset() {
        if (GLOBALOFFSET == 0) GLOBALOFFSET = 0.327/2.0;
        else if (GLOBALOFFSET == 0.327/2.0) GLOBALOFFSET = -0.327/2.0;
        else if (GLOBALOFFSET == -0.327/2.0) GLOBALOFFSET = 0.0;  
    }

}
