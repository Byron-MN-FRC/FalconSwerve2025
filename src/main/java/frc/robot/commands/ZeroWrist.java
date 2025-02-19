package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class ZeroWrist extends Command {

  private final Wrist m_wrist;
  
  public ZeroWrist(Wrist subsystem) {
    m_wrist = subsystem;
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("zeroing wrist");
  }

  @Override
  public void execute() {
    // m_wrist.runInTheZeroWay();
  }

  @Override
  public void end(boolean interrupted) {
    // m_wrist.stopMotor();
  }

  @Override
  public boolean isFinished() {
    // return m_wrist.switchValue();
    return false;
  }
}
