package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TagApproach {
    enum gameTarget {
        Reef,
        Barge,
        CoralStation, 
        Processor
    }
    
    private int _fiduciaryNumber;
    private Alliance _alliance;
    private gameTarget _targetType;
    private Pose2d _desiredPose;
    private String _descriptor;
    
    public TagApproach(int id, Alliance alliance, gameTarget targetType, 
        Pose2d desiredPose) {
        _fiduciaryNumber = id;
        _descriptor = "";
        _alliance = alliance;
        _targetType = targetType;
        _desiredPose = desiredPose;
    }

    // unused test method to construct an approach that is not associated with a tag.
    public TagApproach(String descriptor, Alliance alliance, gameTarget targetType, Pose2d desiredPose){
        _fiduciaryNumber = 0;
        _descriptor = descriptor;
        _alliance = alliance;
        _targetType = targetType;
        _desiredPose = desiredPose;
    }

    public int FiduciaryNumber(){
        return _fiduciaryNumber;
    }

    public Alliance TagAlliance(){
        return _alliance;
    }

    public gameTarget GameTarget(){
        return _targetType;
    }
    
    public Pose2d DesiredPos(){
        return _desiredPose;
    }

    public String Descriptor() {
        return _descriptor;
    }

}
