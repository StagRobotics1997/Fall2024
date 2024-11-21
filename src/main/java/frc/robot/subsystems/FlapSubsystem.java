// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FlapConstants;
import frc.robot.Constants;

public class FlapSubsystem extends SubsystemBase {

  private Relay mRelay;

  /** Creates a new ExampleSubsystem. */
  public FlapSubsystem() {
    if (!Constants.PracticeBot) {
      mRelay = new Relay(FlapConstants.RELAY_PORT);
    }
  }

  public Command InCommand() {
    if (Constants.PracticeBot) {
      return new PrintCommand("Practice Bot - no InCommand");
    }
    return new PrintCommand("in. current value=" + mRelay.get())
        .andThen(runOnce(() -> {
          mRelay.setDirection(Relay.Direction.kReverse);
        }))
        .andThen(runOnce(() -> {
          mRelay.set(Relay.Value.kOn);
        }))
        .andThen(new PrintCommand("current value=" + mRelay.get()))
        .andThen(new WaitCommand(7))
        .andThen(new PrintCommand("current value=" + mRelay.get()))
        .andThen(runOnce(() -> mRelay.set(Relay.Value.kOff)));
  }

  public Command OutCommand() {
    if (Constants.PracticeBot) {
      return new PrintCommand("Practice Bot - no OutCommand");
    }
    return new PrintCommand("out. current value=" + mRelay.get())
        .andThen(runOnce(() -> {
          mRelay.setDirection(Relay.Direction.kForward);
        }))
        .andThen(runOnce(() -> {
          mRelay.set(Relay.Value.kOn);
        }))
        .andThen(new PrintCommand("current value=" + mRelay.get()))
        .andThen(new WaitCommand(7))
        .andThen(new PrintCommand("current value=" + mRelay.get()))
        .andThen(runOnce(() -> mRelay.set(Relay.Value.kOff)));
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void flapIn() {
    if (Constants.PracticeBot) {
      System.out.println("Practice Bot - no flapIn");
      return;
    }
    SmartDashboard.putString("Flap", "In");
    mRelay.set(Value.kForward);
  }
}
