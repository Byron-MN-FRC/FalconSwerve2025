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
public class AlignmentSubsystem extends SubsystemBase {

  private final CANrange rearCanRange = new CANrange(40, "rio");
  private final StatusSignal<Distance> rearDistanceSignal = rearCanRange.getDistance();

  /**
   * Constructs a new AlignmentSubsystem
   */
  public AlignmentSubsystem() {
    var canRangeConfig = new CANrangeConfiguration();
    canRangeConfig.ToFParams.withUpdateMode(UpdateModeValue.LongRangeUserFreq);
    canRangeConfig.FovParams.withFOVRangeX(6.75);
    canRangeConfig.FovParams.withFOVRangeY(6.75);
    rearCanRange.getConfigurator().apply(canRangeConfig);
  }

    @Override
    public void periodic() {
      SmartDashboard.putNumber("Rear Range", getRearDistance().in(Inches));
    }

  /**
   * Gets the distance detected by the sensor
   * 
   * @return distance detected by the sensor
   */
  public Distance getRearDistance() {
    return rearDistanceSignal.refresh().getValue();
  }
}
