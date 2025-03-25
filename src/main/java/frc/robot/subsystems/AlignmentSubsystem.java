package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Millimeter;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem with sensors for aligning to a wall
 */
public class AlignmentSubsystem extends SubsystemBase {

  private Rev2mDistanceSensor distSensor = new Rev2mDistanceSensor(Port.kMXP);
  CANrange d = new CANrange(1);

  /**
   * Constructs a new AlignmentSubsystem
   */
  public AlignmentSubsystem() {
    distSensor.setAutomaticMode(true);
    distSensor.setDistanceUnits(Unit.kMillimeters);
    distSensor.setRangeProfile(RangeProfile.kDefault);
  }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Range", getDistance());
    }

    public double getDistance() {
      return distSensor.GetRange() / 1000;
    }

  // /**
  //  * Gets the relative angle of the drivetrain to the reef. Positive angle means the robot needs to turn clockwise.
  //  * 
  //  * @return relative angle of the drivetrain to the reef
  //  */
  // public Angle getRelativeAngle() {
  //   Distance leftDistance = getLeftDistance();
  //   Distance rightDistance = getRightDistance();

  //   return Radians.of(
  //       Math.atan2(
  //           rightDistance.minus(leftDistance).in(Inches),
  //             Math.abs(LEFT_CANRANGE_DISTANCE_FROM_CENTER.minus(RIGHT_CANRANGE_DISTANCE_FROM_CENTER).in(Inches))));
  // }

  // /**
  //  * Gets the distance from the center of the front robot on the plane of the CANRanges to the reef perpendicular to the
  //  * reef.
  //  * 
  //  * @return Distance to the reef perpendicular to the reef
  //  */
  // public Distance getDistance() {
  //   Distance rightDistance = getRightDistance();
  //   Angle relativeAngle = getRelativeAngle();
  //   Distance normalizedRightDistance = rightDistance.times(Math.cos(relativeAngle.in(Radians)));
  //   return normalizedRightDistance.plus(RIGHT_CANRANGE_DISTANCE_FROM_CENTER.times(Math.sin(relativeAngle.in(Radians))));
  // }

}