package frc.util;
import java.util.concurrent.TimeUnit;


import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class SIMColourSensor{
    private SerialPort serial;


    public SIMColourSensor() {
        this.serial = new SerialPort(115200, Port.kMXP);
    }


    public void close() {
		serial.close();
	}

    public int receive(byte[] buffer, int length, Checksum cs) {
		int i, j, c;
		if (cs != null)
			cs.reset();
		for (i = 0; i < length; i++) {
			// wait for byte, timeout after 2ms
			// note for a baudrate of 19.2K, each byte takes about 500us
			for (j = 0; true; j++) {
				if (j == 200)
					return -1;
				c = serial.read(1)[0];
				if (c >= 0)
					break;
				try {
					TimeUnit.MICROSECONDS.sleep(10);
				} catch (InterruptedException e) {
				}
			}
			buffer[i] = (byte) c;
			if (cs != null) {
				byte b = buffer[i];
				int csb = b & 0xff;
				cs.updateChecksum(csb);
			}
		}
		return length;
	}

    public int receive(byte[] buffer, int length) {
		return receive(buffer, length, null);
	}





public class Checksum{

    int cs = 0;

    public void updateChecksum(int b) {
        cs += b;
    }

    public int getChecksum() {
        return cs;
    }

    public void reset() {
        cs = 0;
    }
    }
}