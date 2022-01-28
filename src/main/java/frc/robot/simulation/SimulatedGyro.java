package frc.robot.simulation;

import edu.wpi.first.wpilibj.interfaces.Gyro;

public class SimulatedGyro implements Gyro {
    private double m_value;

    @Override
    public void close() throws Exception {
        // do nothing
    }

    @Override
    public void calibrate() {
        // do nothing
    }

    @Override
    public void reset() {
        m_value = 0;        
    }

    @Override
    public double getAngle() {
        return m_value;
    }

    @Override
    public double getRate() {
        throw new UnsupportedOperationException();
    }

    /**
     * Sets the simulated gyro angle
     * 
     * @param newAngle New angle for the gyro
     */
    public void setAngle(double newAngle) {
        m_value = newAngle;
    }
}
