package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private AddressableLED addressableLED;
    private AddressableLEDBuffer buffer;

    public LED() {
        addressableLED = new AddressableLED(LEDConstants.port);
        buffer = new AddressableLEDBuffer(LEDConstants.count);
        addressableLED.setLength(buffer.getLength());
        addressableLED.setData(buffer);
        addressableLED.start();
    }

    public Command setColor(Color color) {
        return runOnce(() -> {
            LEDPattern red = LEDPattern.solid(color);
            red.applyTo(buffer);
            addressableLED.setData(buffer);
        });
    }
}
