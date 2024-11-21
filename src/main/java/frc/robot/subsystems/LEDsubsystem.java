package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class LEDsubsystem extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private final double[][] targetDestinations = {
            { 1.0, 15.29, 0.80, 2.11 },
            // { 7.0, 1.33, 5.42, 0 },
            // { 8.0, 1.33, 5.42, 0 },
            { 7.0, 0.83, 4.34, -1.016 },
            { 8.0, 0.83, 4.34, -1.016 },
            { 3.0, 15.60, 4.42, -2.234 },
            { 4.0, 15.60, 4.42, -2.234 },
            { 10.0, 1.21, 0.78, 1.047 },
            { 9.0, 1.21, 0.78, 1.047 },
            { 2.0, 15.25, 0.91, 2.11 },

    };
    private boolean inrainbowmode;
    private int m_rainbowFirstPixelHue;

    public LEDsubsystem() {
        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(131);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
        setColor("", 0, m_ledBuffer.getLength());
    }

    public Command UpdateTargetInfo() {
        return this.runOnce(() -> {
            boolean found = false;
            // NetworkTableEntry xEntry = m_targetingTable.getEntry("tx");
            // NetworkTableEntry tidEntry = m_targetingTable.getEntry("tid");
            if (inrainbowmode) {
                rainbow();
            // } else {
            //     if (LimelightHelpers.getTV("limelight")) {
            //         double targetId = LimelightHelpers.getFiducialID("limelight");
            //         for (double[] targetDestination : targetDestinations)
            //             if (targetDestination[0] == targetId) {
            //                 found = true;
            //                 break;
            //             }
            //     }
             }
            if (found) {
                setColor("green", 0, 131);

            } else {
                setColor("red", 0, 131);

            }
        });

    }

    // public Command UpdateAllianceColor() {
    // return this.runOnce(() -> {
    // DriverStation.Alliance color = DriverStation.getAlliance();
    // if (color == DriverStation.Alliance.Blue) {
    // setColor("blue", 10, 10);
    // setColor("blue", 40, 10);
    // } else {
    // setColor("red", 10, 10);
    // setColor("red", 40, 10);
    // }
    // });
    // }

    public Command UpdateBatteryStatus() {
        return this.runOnce(() -> {
            double percentage = RobotController.getBatteryVoltage() / 15 * 100;
            setColor("green", 20, (int) percentage / 10);
            setColor("green", 50, (int) percentage / 10);
            setColor("red", (int) percentage / 10 + 20, 10 - (int) percentage / 10);
            setColor("red", (int) percentage / 10 + 50, 10 - (int) percentage / 10);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 /
                    m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
            // m_ledBuffer.setRGB(i, 0, 0, 1);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    public void rainbowOn() {
        inrainbowmode = true;
    }

    public void rainbowOff() {
        inrainbowmode = false;
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
        }
        catch(Exception e) {
            System.out.println(e);
        }
    }
}
