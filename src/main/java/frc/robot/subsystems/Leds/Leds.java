package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
    private SerialPort serial1 = new SerialPort(9600, SerialPort.Port.kUSB1);

    public enum EffectCode {
        OFF(0),
        FIRE(1),
        BLINK(2),
        BREATHE(3),
        SOLID(4),
        RAINBOW(5);
    
        private final int code;
    
        EffectCode(int code) {
            this.code = code;
        }
    
        public int getCode() {
            return code;
        }
    }
    
    public void setEffect(EffectCode effect) {
        byte[] byteArray = new byte[]{(byte) effect.getCode()};
        serial1.write(byteArray, 1);
    }
    
}
