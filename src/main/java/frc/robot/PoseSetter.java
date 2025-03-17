package frc.robot;

import java.util.TreeMap;

public class PoseSetter {

	public static final String Feeder = "Feeder";
	public static final String Zero = "Zero";
	public static final String Stored = "Stored";
	public static final String Ground = "Ground";
	public static final String Climb = "Climb";
	public static final String PreClimb = "PreClimb";
	public static final String PreZero = "PreZero";
	public static final String AlgaeGrab = "AlgaeRemove";
	public static final String Processor = "Processor";
	public static final String LowAlgaeGrab = "LowAlgaeGrab";

	public static final TreeMap<String, double[]> positionsMap = new TreeMap<String, double[]>() {
		private static final long serialVersionUID = 1L;
		{ // place to go to stage 1, stage 2, shoulder

			put("L1", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2LowerLimit,
					99
			});

			put("L2", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2LowerLimit,
					99,
			});

			put("L3", new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					1.73,
					99,
			});

			put("L4", new double[] {
					Constants.ElevatorConstants.stage1UpperLimit,
					Constants.ElevatorConstants.stage2UpperLimit,
					103,

			});

			put(Feeder, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					1.61,
					Constants.ShoulderConstants.shoulderUpperLimit,
			});//tune

			put(Zero, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					Constants.ElevatorConstants.stage2UpperLimit,
					Constants.ShoulderConstants.shoulderLowerLimit,
			});

			put(Stored, new double[] {
					Constants.ElevatorConstants.stage1LowerLimit,
					0.74,
	 				120,

			});//tune

			put(Climb, new double[] {
					-1.25,
					Constants.ElevatorConstants.stage2LowerLimit,
					Constants.ShoulderConstants.shoulderUpperLimit,
			});//tune

			put(PreClimb, new double[] {
					1.0f,
					Constants.ElevatorConstants.stage2UpperLimit,
					Constants.ShoulderConstants.shoulderUpperLimit,

			});//tune

			put(PreZero, new double[] {
					Constants.ElevatorConstants.stage1UpperLimit / 4,
					Constants.ElevatorConstants.stage2UpperLimit,
					Constants.ShoulderConstants.shoulderLowerLimit + 8,
			});

			put(AlgaeGrab + "L1", new double[] {
				Constants.ElevatorConstants.stage1LowerLimit,
				Constants.ElevatorConstants.stage2UpperLimit,
				19,
			});

			put(AlgaeGrab + "L2", new double[] {
				Constants.ElevatorConstants.stage1LowerLimit,
				Constants.ElevatorConstants.stage2UpperLimit,
				19,
			});

			put(AlgaeGrab + "L3", new double[] {
				4.6,
				Constants.ElevatorConstants.stage2UpperLimit,
				22,
		    });

			put(AlgaeGrab + "L4", new double[] {
				4.6,
				Constants.ElevatorConstants.stage2UpperLimit,
				22,
		    });

			put(Processor, new double[]{
				Constants.ElevatorConstants.stage1LowerLimit,
				0.74,
				19
			}); //tune

			put(LowAlgaeGrab, new double[] {
				Constants.ElevatorConstants.stage1LowerLimit,
				Constants.ElevatorConstants.stage2LowerLimit,
				130,
		    }); //tune

		}
	};

}