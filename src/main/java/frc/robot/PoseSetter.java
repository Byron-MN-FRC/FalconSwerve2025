package frc.robot;

import java.util.TreeMap;

public class PoseSetter {
    	public static final TreeMap<String, double[]> positionsMap = new TreeMap<String, double[]>() {
		private static final long serialVersionUID = 1L;
		{   // place to go to                                  stage 1, stage 2, shoulder, wrist
			put( Constants.Selector.PlacementSelector.L1, new double[] { 0, 5, 20, 0 });
			put( Constants.Selector.PlacementSelector.L2, new double[] { 0, 10, 20, 90 });
			put( Constants.Selector.PlacementSelector.L3, new double[] { 5, 10, 20, 90 });
			put( Constants.Selector.PlacementSelector.L4, new double[] { 10, 10, 100, 90 });
			// put( 70, new double[] { 30, 3200 }); 
			// put( 25, new double[] { -5000, 2500 });
			// put( 0, new double[] { -4300, 3200 });

		}
	};

}
