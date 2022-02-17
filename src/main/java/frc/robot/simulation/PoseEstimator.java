package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.DriveTrain;

/**
 * Estimates robot's x, y, and rotational pose on the field from encoder and gyro values
 */
public class PoseEstimator {
    private double m_lastLeftEncoderValue;
    private double m_lastRightEncoderValue;
    private DriveTrain m_driveTrain;
    private Pose2d m_pose = new Pose2d(new Translation2d(8.922,5.522), new Rotation2d(1));
 
    
    /**
     * Constructor for pose estimator
     * @param driveTrain Hardware on which to estimate pose
     */
    public PoseEstimator(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        //m_lastLeftEncoderValue = m_driveTrain.getLeftEncoderValue();
        m_lastRightEncoderValue = m_driveTrain.getRightEncoderValue();
    }

    /**
     * Update the pose state based on the current encoder values and gyro
     */
    public void periodic() {
        /* Very simplified physics model used below:
         * We assume forward motion is just average of motion of both wheels.
         */ 
        var leftCount = m_driveTrain.getLeftEncoderValue() - m_lastLeftEncoderValue;
        var rightCount = m_driveTrain.getRightEncoderValue() - m_lastRightEncoderValue;

        m_lastLeftEncoderValue = m_driveTrain.getLeftEncoderValue();
        m_lastRightEncoderValue = m_driveTrain.getRightEncoderValue();
        
        double linear = ((double)(leftCount + rightCount)) / 2.0;
        // note flip in angle: gyro follows left-hand rule
        double rotationRadians = -Math.toRadians(m_driveTrain.getGyroAngleDegrees());

        /* Now figure out how far the robot went this blip of time.
         * Linear is the distance, but the x- and y-axis distance is figured out by
         * trigonometry on the current heading.
         */
        double newX = Units.inchesToMeters(
            m_driveTrain.encoderTicksToInches(linear) * Math.cos(rotationRadians)) + m_pose.getX();
        double newY = Units.inchesToMeters(
            m_driveTrain.encoderTicksToInches(linear)  * Math.sin(rotationRadians)) + m_pose.getY();

        /* Create a new pose with thesee updated values 
         * Note: angle is flipped from right-hand rule; gets more negative as
         * chassis rotates counter-clockwise
         */
         newX=Math.max( 0, newX);
         newX=Math.min(16.4846, newX);
         newY=Math.max( 0, newY);
         newY=Math.min(8.1026, newY);
        
         m_pose = new Pose2d(
             new Translation2d(newX, newY), 
             Rotation2d.fromDegrees(-m_driveTrain.getGyroAngleDegrees()));        
    }

    /**
     * Get most-recently-calculated pose value
     * @return pose value
     */
    public Pose2d getPose() {
        return m_pose;
    }
}
