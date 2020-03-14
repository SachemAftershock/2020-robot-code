package frc.lib;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

/**
* Class to Instantiate LIDAR Lite v3 Objects
*/
public class Lidar {
    
    private Counter mCounter;
    
    /**
     * Creates a LIDAR Lite v3 Object
     * @param port DigitalInput Object
     */
    public Lidar(DigitalSource port) {
        mCounter = new Counter(port);
        mCounter.setMaxPeriod(1.0);
        mCounter.setSemiPeriodMode(true);
        mCounter.reset();
    }

    /**
     * Gets the distance in Centimeters
     * @return Measured Distance in Centimeters
     */
    public double getDistanceCm() {
        return mCounter.getPeriod() * 1000000.0 / 10.0;
    }
    
    /**
     * Gets the distance in Inches
     * @return Measures distance in Inches
     */
    public double getDistanceIn() {
        return getDistanceCm() * .393700787;
    }
}
