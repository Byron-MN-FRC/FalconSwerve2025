package frc.robot;

import java.util.TreeMap;

public class PoseSetter {

	public static final String Feeder = "Feeder";
	public static final String Zero = "Zero";
	public static final String Stored = "Stored";
	public static final String Ground = "Ground";
	public static final String Climb = "Climb";

	public static final TreeMap<String, double[]> positionsMap = new TreeMap<String, double[]>() {
		private static final long serialVersionUID = 1L;
		// TODO tune values
		{ // place to go to stage 1, stage 2, shoulder, wrist
			put("L1", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					1.5,
					10,
					Constants.WristConstants.rotationHorizontalAlligned
			});

			put("L2", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2UpperLimit,
					10,
					Constants.WristConstants.rotationVerticalAlligned
			});

			put("L3", new double[] {
					1.25,
					Constants.ElevatorConstants.stage2UpperLimit,
					10,
					Constants.WristConstants.rotationVerticalAlligned
			});

			put("L4", new double[] {
					Constants.ElevatorConstants.stage1UpperLimit,
					Constants.ElevatorConstants.stage2UpperLimit,
					6,
					Constants.WristConstants.rotationVerticalAlligned

			});

			put(Feeder, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					1.5,
					3,
					Constants.WristConstants.rotationVerticalAlligned
			});

			put(Zero, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2UpperLimit,
					Constants.ShoulderConstants.shoulderLowerLimit,
					Constants.WristConstants.rotationHorizontalAlligned

			});

			put(Stored, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					1.5,
					1,
					Constants.WristConstants.rotationHorizontalAlligned

			});

			put(Ground, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2UpperLimit,
					Constants.ShoulderConstants.shoulderUpperLimit,
					Constants.WristConstants.rotationHorizontalAlligned

			});

			put(Climb, new double[] {
					-1.25,
					Constants.ElevatorConstants.stage2LowerLimit,
					Constants.ShoulderConstants.shoulderUpperLimit,
					Constants.WristConstants.rotationHorizontalAlligned
			});

		}
	};

}