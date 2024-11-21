// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private DigitalInput mProxSensor;
  private SparkMax mMotor1;
  private SparkMax mMotor2;

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  public IntakeSubsystem() {
    if (!Constants.PracticeBot) {
      mMotor1 = new SparkMax(IntakeConstants.motor1CanId, MotorType.kBrushless);
      mMotor2 = new SparkMax(IntakeConstants.motor2CanId, MotorType.kBrushless);
      mProxSensor = new DigitalInput(IntakeConstants.proximitySensorDIOPort);
    }
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(131);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the buffer data for LEDs
    m_led.setData(m_ledBuffer);
    m_led.start();
    setColor("", 0, m_ledBuffer.getLength());
  }

  public Boolean noteDetected() {
    if (Constants.PracticeBot) {
      return true;
    }
    return !mProxSensor.get();
  }

  public void startMotors(double speed) {
    System.out.println("starting intake");
    if (!Constants.PracticeBot) {
      mMotor1.set(speed);
      mMotor2.set(speed);
    }
  }

  public void startMotorsReverse() {
    System.out.println("starting intake Reverse");
    if (!Constants.PracticeBot) {
      mMotor1.set(.15);
      mMotor2.set(.15);
    }
  }

  public void stopMotors(boolean interrupted) {
    System.out.print("stopping intake");
    if (!Constants.PracticeBot) {
      mMotor1.set(0);
      mMotor2.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!Constants.PracticeBot) {
      SmartDashboard.putNumber("Intake Motor 1 RPM", mMotor1.getEncoder().getVelocity());
      SmartDashboard.putNumber("Intake Motor 2 RPM", mMotor2.getEncoder().getVelocity());
      SmartDashboard.putBoolean("Note Detected", noteDetected());
      if (noteDetected()) {
        setColor("green", 0, 131);
      } else {
        setColor("red", 0, 131);
      }
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setColor(String color, Integer starting, Integer ledCount) {
    for (var i = starting; i < ledCount + starting; i++) {
      switch (color) {
        case "red":
          m_ledBuffer.setRGB(i, 255, 0, 0);
          break;
        case "blue":
          m_ledBuffer.setRGB(i, 0, 0, 255);
          break;
        case "purple":
          m_ledBuffer.setRGB(i, 255, 0, 255);
          break;
        case "green":
          m_ledBuffer.setRGB(i, 0, 255, 0);
          break;
        case "yellow":
          m_ledBuffer.setRGB(i, 255, 255, 0);
          break;
        default:
          m_ledBuffer.setRGB(i, 255, 255, 255);
      }
    }
    // Set the LEDs
    try {
      m_led.setData(m_ledBuffer);
    } catch (Exception e) {
      System.out.println(e);
    }
  }
}
