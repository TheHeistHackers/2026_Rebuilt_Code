package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSubsystem extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    public LEDSubsystem() {
        led = new AddressableLED(0); // PWM port 0
        buffer = new AddressableLEDBuffer(60); // set your LED count

        led.setLength(buffer.getLength());

        led.setData(buffer);
        led.start();
    }

    // simple test: set entire strip to a color
    public void setColor(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
        led.setData(buffer);
    }
}