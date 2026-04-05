package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED addressableLED;
    private final AddressableLEDBuffer buffer;

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
}
