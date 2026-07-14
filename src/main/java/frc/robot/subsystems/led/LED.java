package frc.robot.subsystems.led;

import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.util.Color;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED addressableLED;
    private final AddressableLEDBuffer buffer;

    public LED() {
        addressableLED = new AddressableLED(LEDConstants.LED_PORT);
        buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
        addressableLED.setLength(buffer.getLength());
        addressableLED.setData(buffer);
    }

    public Command setColor(Color color) {
        return runOnce(() -> {
            LEDPattern patternColor = LEDPattern.solid(color);
            patternColor.applyTo(buffer);
            addressableLED.setData(buffer);
        });
    }
}
