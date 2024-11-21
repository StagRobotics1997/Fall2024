// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;

public class AngleSubsystem extends SubsystemBase {
 
  private SparkMax mMotor1;


  public AngleSubsystem() {
    
    mMotor1 = new SparkMax(AngleConstants.motor1CanId, MotorType.kBrushless);
  }
 
  public void startMotors() {
    System.out.println("starting Angle");
    mMotor1.set(-.3);
  
  }

  public void stopMotors(boolean interrupted) {
    System.out.print("stopping Angle");
    mMotor1.set(0);
   
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle Motor 1 RPM", mMotor1.getEncoder().getVelocity());
      }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
