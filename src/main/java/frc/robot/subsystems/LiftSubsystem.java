// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
// import com.revrobotics.s
import com.revrobotics.spark.*;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {
  private SparkMax mMotor1;

  public LiftSubsystem() {
    if (!Constants.PracticeBot) {
      mMotor1 = new SparkMax(LiftConstants.motor1CanId, MotorType.kBrushed);
    }
  }

  public void startMotors() {
    System.out.println("starting Lift");
    if (!Constants.PracticeBot) {
      mMotor1.set(.5);
    }
  }

  public void stopMotors(boolean interrupted) {
    System.out.print("stopping Lift");
    if (!Constants.PracticeBot) {
      mMotor1.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
