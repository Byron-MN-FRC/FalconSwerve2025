package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TagApproaches;

public class Testing extends Command {  
  public Testing() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TagApproaches.getInstance().addTagCentricOffset(
      new Pose2d(0, 0, new Rotation2d(Math.PI)), 
      new Pose2d(0.0, 1, new Rotation2d(0)));

      TagApproaches.getInstance().addTagCentricOffset(
        new Pose2d(0.0, 0.0, new Rotation2d(3* Math.PI/4)), 
        new Pose2d(0, 1, new Rotation2d(Math.PI)));

  }


  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    // TODO Auto-generated method stub
    return true;
  }

}
