package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

public class Lidar {
    private Counter mCounter;
    
    public Lidar(DigitalSource port) {
        mCounter = new Counter(port);
        mCounter.setMaxPeriod(1.0);
        mCounter.setSemiPeriodMode(true);
        mCounter.reset();
    }

    public double getDistanceCm() {
        return mCounter.getPeriod() * 1000000.0 / 10.0;
    }
    public double getDistanceIn() {
        return getDistanceCm() * .393700787;
    }
}
