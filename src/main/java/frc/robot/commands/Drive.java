package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.ShootParamaters;

public class Drive extends CommandBase {

    private DriveTrain m_driveTrain;
    public ShootParamaters M_Shoot;

    // Addition of the distances to group them together
    private double m_desiredDistance;

    public Drive(DriveTrain driveTrain, ShootParamaters Shootparamaters, double distanceInches) {
        M_Shoot = Shootparamaters;
        m_driveTrain = driveTrain;
        m_desiredDistance = distanceInches;
    }

    @Override
    public void initialize() {
        m_driveTrain.resetEncoders();

        System.out.println("Running");

    }

    @Override
    public void execute() {

        System.out.println("Driving Forwards");

        if (m_desiredDistance > 0) {
            m_driveTrain.SetAutonomousIntake();
            m_driveTrain.tankDrive(.5, .5);
        } else {
            m_driveTrain.SetAutonomousIntake();
            m_driveTrain.tankDrive(-.5, -.5);
        }


        SmartDashboard.putNumber("DistanceTraveled", m_driveTrain.getEncoderInches());

        // System.out.println(M_DriveTrain.getLeftEncoderValue());

    }

    @Override
    public void end(boolean interrupt) {

        m_driveTrain.tankDrive(0, 0);
        m_driveTrain.StopIntake();

    }

    @Override
    public boolean isFinished() {

        // Sets the distance to zerox
        double distanceInches = m_driveTrain.getEncoderInches();

        // While our desired distance is greater than our current distance keep running
        // the drive motors
        if (m_desiredDistance < 0) {

            return (distanceInches <= m_desiredDistance);
        }

        return (distanceInches >= m_desiredDistance);

    }
}
