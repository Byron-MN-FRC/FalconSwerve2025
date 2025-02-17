// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Climb;
import frc.robot.commands.DriveToPosition;
import frc.robot.commands.GrabCoralHigh;
import frc.robot.commands.GrabCoralLow;
import frc.robot.commands.PlaceCoral;
import frc.robot.commands.SelectPlacement;
import frc.robot.commands.Store;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
    // Subsystems
        public final Claw m_claw = new Claw();
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        public final Shoulder m_shoulder = new Shoulder();
        public final Wrist m_wrist = new Wrist();
        public final Elevator m_elevator = new Elevator();
        public final Vision m_Vision = new Vision();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double percentSlow = 1;

    public String goalArrangement = "blank";
    public String currentArrangement = "blank";

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController accessory = new XboxController(1);
    // private final CommandXboxController characterizationJoystick = new CommandXboxController(2);


    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* Fields */
    public Field2d field = new Field2d();
    public Field2d targetPoseField = new Field2d();

    //TODO reassign
    public int globalCurrNumSelected = 1;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Autonomous Command");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Field Widgets
        SmartDashboard.putData("Current Robot Position", field);
        SmartDashboard.putData("Target Robot Position", targetPoseField);

        // selector spots
        Constants.Selector.PlacementSelector.initializeTab();
        SmartDashboard.putString("current setting", currentArrangement);
        SmartDashboard.putString("goal Setting", goalArrangement);

        // sliders for tuning positions? would need to set the motors to these speeds
        // Shuffleboard.getTab("REEFSCAPE").add("shoulder", shouldermotor).withWidget(BuiltInWidgets.kNumberSlider);
        // Shuffleboard.getTab("REEFSCAPE").add("stage 1", elevatorStage1).withWidget(BuiltInWidgets.kNumberSlider);
        // Shuffleboard.getTab("REEFSCAPE").add("stage 2", elevatorStage2).withWidget(BuiltInWidgets.kNumberSlider);

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

            SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = drivetrain.getState().Pose;
      
      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(2.0, 0.0)), new Rotation2d());

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
        waypoints, 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ),
        null, // Ideal starting state can be null for on-the-fly paths
        new GoalEndState(0.0, currentPose.getRotation())
      );

      // Prevent this path from being flipped on the red alliance, since the given positions are already correct
      path.preventFlipping = true;

      AutoBuilder.followPath(path).schedule();
    }));
    
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically

            drivetrain.applyRequest(() -> drive
                .withVelocityX(-joystick.getLeftY() * MaxSpeed * percentSlow)
                    // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * percentSlow)
                    // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * percentSlow)
                    // Drive counterclockwise with negative X (left)
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
        joystick.y().onTrue(new PlaceCoral(m_shoulder, m_elevator, m_wrist, m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        joystick.b().whileTrue(new GrabCoralHigh(m_shoulder, m_elevator, m_wrist, m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        joystick.a().whileTrue(new GrabCoralLow(m_shoulder, m_elevator, m_wrist, m_claw).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        joystick.back().onTrue(new InstantCommand(() -> slow()));
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Op Test Buttons TODO Reassign
        joystick.b().whileTrue(
            new DriveToPosition(drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );
        joystick.leftBumper().onTrue(new InstantCommand(() -> minus()));
        joystick.rightBumper().onTrue(new InstantCommand(() -> plus()));


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
        btnClimb.onTrue(new Climb(m_elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        final JoystickButton btnStore = new JoystickButton(accessory, XboxController.Button.kBack.value);        
        btnStore.onTrue(new Store(m_shoulder, m_elevator, m_wrist).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

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

    //TODO NAMES
    private void plus() {
        globalCurrNumSelected++;
    }

    private void minus() {
        if (globalCurrNumSelected > 1) {
            globalCurrNumSelected--;
        }
    }
    public Boolean getWristTripped() {
        return false;
       //return !shoulderAndTopCandi.getS2Closed().getValue();
      }
      public Boolean getTopStage2() {
        return false;
       // return shoulderAndTopCandi.getS1Closed().getValue();
      }
      public Boolean getCoralDetect() {
        return false;
        //return !wristAndClawCandi.getS1Closed().getValue();
      }
}
