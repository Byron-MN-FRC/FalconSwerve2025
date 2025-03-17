// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final class Selector {
        public static final class DPAD {
            public static final int kUp = 0;
            public static final int kRight = 90;
            public static final int kDown = 180;
            public static final int kLeft = 270;
        }

        public static class PlacementSelector {

            private static boolean[][] array = new boolean[4][2];
            private static int currentRow = 0;
            private static int currentCol = 0;
            public static String level = "blank";
            public static String left = "left";
            public static String right = "right";

            public PlacementSelector() {

                // Initially set the first element to true
                array[currentRow][currentCol] = true;
            }

            public static void move(int direction) {
                // Set the current true element to false
                array[currentRow][currentCol] = false;
                SmartDashboard.putBoolean(currentRow + "-" + currentCol, false);

                if (direction == Constants.Selector.DPAD.kDown && currentRow > 0) {
                    currentRow--;
                } else if (direction == Constants.Selector.DPAD.kUp && currentRow < array.length - 1) {
                    currentRow++;
                } else if (direction == Constants.Selector.DPAD.kLeft && currentCol > 0) {
                    currentCol--;
                } else if (direction == Constants.Selector.DPAD.kRight && currentCol < array[0].length - 1) {
                    currentCol++;
                }

                // Set the new position to true
                array[currentRow][currentCol] = true;
                SmartDashboard.putBoolean(currentRow + "-" + currentCol, true);
            }

            public static int getCurrentCol() {
                return currentCol;
            }

            public static int getCurrentRow() {
                return currentRow;
            }

            public static void initializeTab() {
                for (int i = 0; i < array.length; i++) {
                    for (int j = 0; j < array[i].length; j++) {
                        array[i][j] = false;
                        SmartDashboard.putBoolean(i+"-"+j, false);
                    }
                }
                array[currentRow][currentCol] = true;
                SmartDashboard.putBoolean((currentRow+"-"+currentCol), true);

            }

            public static void printArray() {
                for (boolean[] row : array) {
                    for (boolean element : row) {
                        System.out.print(element + " ");
                    }
                    System.out.println();
                }
            }
        

            public static String getLevel(){
                if (DriverStation.isAutonomous()) level = "L4";
                else if (getCurrentRow() == 0) level = "L1";
                else if (getCurrentRow() == 1) level = "L2";
                else if (getCurrentRow() == 2) level = "L3";
                else if (getCurrentRow() == 3) level = "L4";
                return level;
            }

            public static String getScoringPose(){
                String side = "blank";
                if (getCurrentCol() == 0)side = left;
                else if (getCurrentCol() == 1)side = right;
                return side;
            }

        }
    }
        
    

    public static final class SwerveConstants {
        public static final double percentSlow = 0.35;
    }

    public static final class ElevatorConstants {
        public static final class PID {
            //
            public static final double P = 2.4f;
            public static final int I = 0;
            public static final double D = 0.1f;
            //
            public static final int P2 = 60;
            public static final int I2 = 0;
            public static final int D2 = 6;
            //
        }

        public static final double stage2UpperLimit = 3;
        public static final double stage2LowerLimit = 0;
        public static final double stage1UpperLimit = 2.5;
        public static final double stage1LowerLimit = 0;

    }

    public static final class ShoulderConstants {
        //
        public static final double P = 2.4f;
        public static final int I = 0;
        public static final double D = 0.1f;

        public static final double shoulderUpperLimit = 12;
        public static final double shoulderLowerLimit = 0;
        //
    }

    public static final class WristConstants {
            public static final double wristMotorGearRatio = 251.52; //X input rotations for each output rotation
        
            // public static final double rotationVerticalAlligned = wristMotorGearRatio * 0.25; // quarter of a rotation on the output shaft
            public static final double rotationVerticalAlligned = 54;
            public static final double rotationHorizontalAlligned = 0;
            public static final double tolerance = wristMotorGearRatio / (360.0 * 2.0); // 1/2 degree on the output mechanism

    }

    public static final class ClawConstants {
        public static final double VOLTS_TO_DIST = 2.55;
    }

    public static final class VisionConstants {
        public static final String limeLightName = "limelight-front";
        public static final String limeLightName2 = "limelight-back";
        public static final int aprilPipe = 0;
        // public static final int Pipe2 = 1;

        public static final Pose2d ReefTagOffset = new Pose2d(0, Units.inchesToMeters(24), new Rotation2d(Math.PI));
    }

    public static final class AlignmentConstants{
        public static final Distance RIGHT_CANRANGE_DISTANCE_FROM_CENTER = Inches.of(9); //enter as inches
        public static final Distance LEFT_CANRANGE_DISTANCE_FROM_CENTER = Inches.of(-9); //enter as inches
    }
}
