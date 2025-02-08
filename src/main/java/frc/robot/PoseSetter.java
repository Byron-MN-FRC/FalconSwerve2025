package frc.robot;

import java.util.TreeMap;

public class PoseSetter {


    	public static final TreeMap<String, double[]> positionsMap = new TreeMap<String, double[]>() {
		private static final long serialVersionUID = 1L;
			// TODO tune values
		{   // place to go to   stage 1, stage 2, shoulder, wrist
			put( "L1", new double[] { 0, 5, 20, 0 });
			put( "L2", new double[] { 0, 10, 20, 90 });
			put( "L3", new double[] { 5, 10, 20, 90 });
			put( "L4", new double[] { 10, 10, 100, 90 });
			put( "Feeder", new double[] { 0, 5, 80, 90});
			put( "Pre Match Stored", new double[] { 0, 10, 0, 0});
			put( "Stored", new double[] { 0, 5, 0, 0});
			put( "Ground", new double[] { 0, 10, 20, 0});
		}
	};


}

