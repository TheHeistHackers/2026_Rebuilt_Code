package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;

public class LEDSubsystem extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer buffer;
    
    // Variables for the slow rainbow
    private double rainbowFirstPixelHue = 0;

    // Variables for the blinking effect
    private int blinkCounter = 0;
    private boolean isBlinkOn = true;

    public LEDSubsystem() {
        led = new AddressableLED(4); // PWM port 4
        buffer = new AddressableLEDBuffer(150); 

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            // Slow rainbow pulse when the robot is sitting in the pits/staged
            // runSolidRainbow();
        } else {
            // Blinks Green. Change the numbers to blink a different color!
            // runBlinkRGB(0, 255, 0); 
        }
    }

    /**
     * Blinks the entire strip between the given RGB color and off.
     */
    public void runBlinkRGB(int r, int g, int b) {
        // periodic() runs every 20ms. 25 loops = 500ms (half a second)
        blinkCounter++;
        
        if (blinkCounter >= 10) { 
            isBlinkOn = !isBlinkOn; // Flip the state from ON to OFF (or vice versa)
            blinkCounter = 0;       // Reset the counter to start timing again
        }

        if (isBlinkOn) {
            setSolidRGB(0, 255, 255); // Turn ON with your chosen color
        } else {
            setSolidRGB(0, 0, 0); // Turn OFF (Black means off for LEDs)
        }
    }

    /**
     * Sets the entire strip to a specific RGB color.
     */
    public void setSolidRGB(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }

    /**
     * The slow rainbow logic from before.
     */
    private void runSolidRainbow() {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setHSV(i, (int) rainbowFirstPixelHue, 255, 255); 
        }
        
        rainbowFirstPixelHue += 0.5; 
        if (rainbowFirstPixelHue >= 180) {
            rainbowFirstPixelHue = 0;
        }
        led.setData(buffer); 
    }
}