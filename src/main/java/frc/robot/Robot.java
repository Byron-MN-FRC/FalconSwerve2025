// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private static final RobotContainer m_robotContainer = new RobotContainer();

  private boolean kUseLimelight = true;

  public Robot() {
    // m_robotContainer = new RobotContainer();
    HttpCamera frontCam = new HttpCamera("FrontCam", "http://10.48.59.11:5800");
    CameraServer.addCamera(frontCam);
    HttpCamera backCam = new HttpCamera("BackCam", "http://10.48.59.12:5800");
    CameraServer.addCamera(backCam);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Stage 2", m_robotContainer.getTopStage2());
    SmartDashboard.putBoolean("Coral", m_robotContainer.getCoralDetect());

    /*
     * This example of adding Limelight is very simple and may not be sufficient for
     * on-field use.
     * Users typically need to provide a standard deviation that scales with the
     * distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible,
     * though exact implementation
     * of how to use vision should be tuned per-robot and to the team's
     * specification.
     */
    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.limeLightName, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.VisionConstants.limeLightName);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0 && Robot.getInstance().m_Vision.tempDisable == false) {
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
            Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds));
      }
    }

    SmartDashboard.putNumber("tagselected", Robot.getInstance().globalCurrNumSelected);

  }

  @Override
  public void disabledInit() {
    // Robot.getInstance().m_elevator.stopBothMotors();
    // Robot.getInstance().m_shoulder.stopShoulder();
    // Robot.getInstance().m_claw.zero();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
