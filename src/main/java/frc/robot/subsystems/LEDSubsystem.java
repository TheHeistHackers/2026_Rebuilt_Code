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
        runSolidRainbow();
    } else {
        // 1. Get our current alliance color
        var alliance = DriverStation.getAlliance();
        
        int r = 0, g = 0, b = 255; // Default to Blue
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            r = 0; g = 255; b = 0; // Switch to Red
        }

        // 2. Check if our Alliance's Hub is currently active
        if (isOurHubActive()) {
            // Hub is active! Blink our alliance color.
            runBlinkRGB(r, g, b);
        } else {
            // Hub is inactive. Run solid alliance color.
            setSolidRGB(r, g, b);
        }
    }
}

/**
 * Determines if your alliance's Hub is currently active based on the FRC 2026 REBUILT game manual.
 */
private boolean isOurHubActive() {
    // Both Hubs are active during Autonomous
    if (DriverStation.isAutonomous()) {
        return true;
    }

    // Get current match time in Teleop (counts down from 135 to 0)
    double matchTime = DriverStation.getMatchTime();
    
    // Both Hubs are active during the Transition Shift (first 5s of Teleop) 
    // and End Game (last 30s)
    if (matchTime >= 130.0 || matchTime <= 30.0) {
        return true;
    }

    // Parse Game Data to see who won Autonomous
    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) {
        return false; // FMS data hasn't arrived yet
    }

    var myAlliance = DriverStation.getAlliance();
    if (myAlliance.isEmpty()) {
        return false; 
    }

    // The first character ('R' or 'B') tells us who won Auto
    boolean weWonAuto = (gameData.charAt(0) == 'R' && myAlliance.get() == DriverStation.Alliance.Red) ||
                        (gameData.charAt(0) == 'B' && myAlliance.get() == DriverStation.Alliance.Blue);

    // During the 100 seconds of Alliance Shifts (130s to 30s), Hubs alternate being active.
    // There are 4 shifts of 25 seconds each.
    // Shift 1: 130 - 105 | Shift 2: 105 - 80 | Shift 3: 80 - 55 | Shift 4: 55 - 30
    boolean isShift1Or3 = (matchTime < 130.0 && matchTime >= 105.0) || 
                          (matchTime < 80.0 && matchTime >= 55.0);

    /* * NOTE: You should double-check the current game manual to verify if the 
     * Auto Winner's Hub is ACTIVE or INACTIVE during Shift 1. 
     * The logic below assumes the Auto Winner is ACTIVE for Shifts 1 and 3.
     * If the manual says they are inactive first, simply swap the return statements!
     */
    if (weWonAuto) {
        return isShift1Or3;     // We won, so we are active in shifts 1 and 3
    } else {
        return !isShift1Or3;    // We lost, so we are active in shifts 2 and 4
    }
}

    /**
     * Blinks the entire strip between the given RGB color and off.
     */
    public void runBlinkRGB(int r, int g, int b) {
        // periodic() runs every 20ms. 25 loops = 500ms (half a second)
        blinkCounter++;
        
        if (blinkCounter >= 40) { 
            isBlinkOn = !isBlinkOn; // Flip the state from ON to OFF (or vice versa)
            blinkCounter = 0;       // Reset the counter to start timing again
        }

        if (isBlinkOn) {
            setSolidRGB(r, g, b); // Turn ON with your chosen color
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