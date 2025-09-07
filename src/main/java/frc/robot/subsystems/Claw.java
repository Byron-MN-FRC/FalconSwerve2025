// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  
  private WPI_TalonSRX releaseMotor;
  
  /** Creates a new Claw. */
  public Claw() {
    
    releaseMotor = new WPI_TalonSRX(42);

  }

  public void startMotor() {
    releaseMotor.set(.5);
  }

  public void reverseMotor() {
    releaseMotor.set(-0.5);
  }
  
  public void stopMotor() {
    releaseMotor.set(0);
  }

}
