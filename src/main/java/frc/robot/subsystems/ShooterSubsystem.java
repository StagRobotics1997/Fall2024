// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private DigitalInput mProxSensor;
  private SparkMax mMotor1;
  private SparkMax mMotor2;

  public ShooterSubsystem() {
    if (!Constants.PracticeBot) {
      mProxSensor = new DigitalInput(ShooterConstants.proximitySensorDIOPort);
      mMotor1 = new SparkMax(ShooterConstants.motor1CanId, MotorType.kBrushless);
      mMotor2 = new SparkMax(ShooterConstants.motor2CanId, MotorType.kBrushless);
    }
  }

  public Boolean getSensor() {
    if (Constants.PracticeBot) {
      return false;
    }
    return !mProxSensor.get();
  }

    public void startMotors() {
  startMotors(0.6);
  }
  public void startMotors(double speed) {
    System.out.println("starting shooter");
    if (!Constants.PracticeBot) {
      mMotor1.set(-speed);
      mMotor2.set(speed);
    }
  }

  public void startAmp() {
    System.out.println("starting shooterAmp");
    if (!Constants.PracticeBot) {
      mMotor1.set(-0.15);
      // mMotor2.set(0.05);
    }
  }

  public void startMotorsReverse() {
    System.out.println("starting shooter reverse");
    if (!Constants.PracticeBot) {
      mMotor1.set(0.2);
      mMotor2.set(-0.2);
    }
  }

  public void stopMotors(boolean interrupted) {
    System.out.print("stopping shooter");
    if (!Constants.PracticeBot) {
      mMotor1.set(0);
      mMotor2.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!Constants.PracticeBot) {
      SmartDashboard.putNumber("Shooter Motor 1 RPM", mMotor1.getEncoder().getVelocity());
      SmartDashboard.putNumber("Shooter Motor 2 RPM", mMotor2.getEncoder().getVelocity());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
