package frc.robot.subsystems.led;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class LED extends SubsystemBase {
    private final AddressableLED addressableLED;
    private final AddressableLEDBuffer buffer;

    // all hues at maximum saturation and half brightness
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);

    // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
    // of 1 meter per second.
    private final LEDPattern m_scrollingRainbow =
            m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.2), kLedSpacing);

    public LED() {
        addressableLED = new AddressableLED(LEDConstants.LED_PORT);
        buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        addressableLED.setLength(buffer.getLength());
        addressableLED.setData(buffer);
        addressableLED.start();
    }

    public Command setColor(Color color) {
        return runOnce(() -> {
            LEDPattern patternColor = LEDPattern.solid(color);
            patternColor.applyTo(buffer);
            addressableLED.setData(buffer);
        });
    }

    @Override
    public void periodic() {
        m_scrollingRainbow.applyTo(buffer);
        addressableLED.setData(buffer);
    }

}
