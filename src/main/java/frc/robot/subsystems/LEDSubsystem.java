package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation; // Make sure to import this!

public class LEDSubsystem extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    
    private int rainbowFirstPixelHue = 0; 

    public LEDSubsystem() {
        led = new AddressableLED(4); // PWM port 0
        buffer = new AddressableLEDBuffer(60); // set your LED count

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        // Automatically check the robot's state every 20ms
        if (DriverStation.isDisabled()) {
            runRainbowPattern();
        } else {
            setColorHSV(60, 255, 255);       
        }
    }

    // Renamed and updated to take HSV arguments!
    public void setColorHSV(int h, int s, int v) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, h, s, v); 
        }
        led.setData(buffer);
    }

    private void runRainbowPattern() {
        for (int i = 0; i < buffer.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / buffer.getLength())) % 180;
            buffer.setHSV(i, hue, 255, 128); // HSV: (Hue, Saturation, Value)
        }
        
        rainbowFirstPixelHue += 3; // Change speed of rainbow shift
        rainbowFirstPixelHue %= 180;
        
        led.setData(buffer); 
    }
}