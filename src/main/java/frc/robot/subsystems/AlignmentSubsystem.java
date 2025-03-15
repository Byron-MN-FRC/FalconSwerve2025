package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;

import static frc.robot.Constants.AlignmentConstants.LEFT_CANRANGE_DISTANCE_FROM_CENTER;
import static frc.robot.Constants.AlignmentConstants.RIGHT_CANRANGE_DISTANCE_FROM_CENTER;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem with sensors for aligning to a wall
 */
@Logged
public class AlignmentSubsystem extends SubsystemBase {

  private final CANrange leftCanRange = new CANrange(40, "CANivore1");
  private final CANrange rightCanRange = new CANrange(41, "CANivore1");

  private final StatusSignal<Distance> leftDistanceSignal = leftCanRange.getDistance();
  private final StatusSignal<Distance> rightDistanceSignal = rightCanRange.getDistance();

  /**
   * Constructs a new AlignmentSubsystem
   */
  public AlignmentSubsystem() {
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.withUpdateMode(UpdateModeValue.LongRangeUserFreq);
    canRangeConfig.FovParams.withFOVRangeX(6.75);
    canRangeConfig.FovParams.withFOVRangeY(6.75);
    leftCanRange.getConfigurator().apply(canRangeConfig);
    rightCanRange.getConfigurator().apply(canRangeConfig);
  }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Left Range", getLeftDistance().in(Inches));
      SmartDashboard.putNumber("Right Range", getRightDistance().in(Inches));
      SmartDashboard.putNumber("Relative Angle", getRelativeAngle().in(Degrees));
    }

  /**
   * Gets the distance detected by the front sensor
   * 
   * @return distance detected by the sensor
   */
  public Distance getLeftDistance() {
    return leftDistanceSignal.refresh().getValue();
  }

  /**
   * Gets the distance detected by the back sensor
   * 
   * @return distance detected by the sensor
  //  */
  public Distance getRightDistance() {
    return rightDistanceSignal.refresh().getValue();
  }

  /**
   * Gets the relative angle of the drivetrain to the reef. Positive angle means the robot needs to turn clockwise.
   * 
   * @return relative angle of the drivetrain to the reef
   */
  public Angle getRelativeAngle() {
    Distance leftDistance = getLeftDistance();
    Distance rightDistance = getRightDistance();

    return Radians.of(
        Math.atan2(
            rightDistance.minus(leftDistance).in(Inches),
              Math.abs(LEFT_CANRANGE_DISTANCE_FROM_CENTER.minus(RIGHT_CANRANGE_DISTANCE_FROM_CENTER).in(Inches))));
  }

  /**
   * Gets the distance from the center of the front robot on the plane of the CANRanges to the reef perpendicular to the
   * reef.
   * 
   * @return Distance to the reef perpendicular to the reef
   */
  public Distance getDistance() {
    Distance rightDistance = getRightDistance();
    Angle relativeAngle = getRelativeAngle();
    Distance normalizedRightDistance = rightDistance.times(Math.cos(relativeAngle.in(Radians)));
    return normalizedRightDistance.plus(RIGHT_CANRANGE_DISTANCE_FROM_CENTER.times(Math.sin(relativeAngle.in(Radians))));
  }

}