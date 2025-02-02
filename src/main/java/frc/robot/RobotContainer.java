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

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Climb;
import frc.robot.commands.GrabAlgae;
import frc.robot.commands.GrabCoral;
import frc.robot.commands.PlaceAlgae;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.SelectPlacement;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    public static RobotContainer m_robotContainer = new RobotContainer();
    // Subsystems
        public final Shoulder m_shoulder = new Shoulder();
        public final Wrist m_wrist = new Wrist();
        public final Elevator m_elevator = new Elevator();
        public final Claw m_claw = new Claw();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double percentSlow;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    // private final CommandXboxController characterizationJoystick = new CommandXboxController(2);
    private final XboxController accessory = new XboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Autonomous Command");
        SmartDashboard.putData("Auto Mode", autoChooser);

        SmartDashboard.putBoolean("0-0", true);
        SmartDashboard.putBoolean("1-0", false);
        SmartDashboard.putBoolean("2-0", false);
        SmartDashboard.putBoolean("3-0", false);
        SmartDashboard.putBoolean("0-1", false);
        SmartDashboard.putBoolean("1-1", false);
        SmartDashboard.putBoolean("2-1", false);
        SmartDashboard.putBoolean("3-1", false);
        Constants.Selector.PlacementSelector.initializeTab();

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * percentSlow) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed * percentSlow) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentSlow) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));
        // generated buttons that drivers will probably never use
        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
        // -joystick.getLeftX()))
        // ));

        // public final Wrist m_wrist = new Wrist();

        // Characterization buttons
        // Note that each routine should be run exactly once in a single log.
        // characterizationJoystick.y().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // characterizationJoystick.a().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // characterizationJoystick.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // characterizationJoystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Operator buttons
        joystick.rightTrigger(.5).onTrue(new PlaceCoral(m_shoulder, m_elevator, m_wrist, m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // whileTrue because it won't be with tags -- drivers will need to hold to stop selector from overriding poses
        joystick.rightBumper().whileTrue(new GrabCoral(m_shoulder, m_elevator, m_wrist, m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        joystick.leftTrigger(.5).whileTrue(new PlaceAlgae(m_shoulder, m_elevator, m_wrist, m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        joystick.leftBumper().onTrue(new GrabAlgae(m_shoulder, m_elevator, m_wrist, m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // reset the field-centric heading
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // Accessory buttons
        final POVButton pOVButtonLeft = new POVButton(accessory, 270, 0);
        pOVButtonLeft.onTrue(new SelectPlacement(270).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final POVButton pOVButtonRight = new POVButton(accessory, 90, 0);
        pOVButtonRight.onTrue(new SelectPlacement(90).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final POVButton pOVButtonDown = new POVButton(accessory, 180, 0);
        pOVButtonDown.onTrue(new SelectPlacement(180).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final POVButton pOVButtonUp = new POVButton(accessory, 0, 0);
        pOVButtonUp.onTrue(new SelectPlacement(0).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
        final JoystickButton btnClimb = new JoystickButton(accessory, XboxController.Button.kStart.value);        
        btnClimb.onTrue(new Climb().withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void slow() {
        if (percentSlow == 1) {
            percentSlow = Constants.Swerve.percentSlow;
        } else {
            percentSlow = 1;
        }
    }
}
