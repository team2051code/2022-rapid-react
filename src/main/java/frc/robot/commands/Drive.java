package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class Drive extends CommandBase {

    private DriveTrain M_DriveTrain;
    // Addition of the distances to group them together
    private double DesiredDistance;

    public Drive(DriveTrain DriveTrain, double DistanceInches) {

        M_DriveTrain = DriveTrain;
        DesiredDistance = DistanceInches;
    }

    @Override
    public void initialize() {
        M_DriveTrain.TestingMotors();

        System.out.println("Running");

    }

    @Override
    public void execute() {

        M_DriveTrain.tankDrive(.5, .5);

        SmartDashboard.putNumber("DistanceTraveled", M_DriveTrain.GetEncoderInches());

        // System.out.println(M_DriveTrain.getLeftEncoderValue());

    }

    @Override
    public void end(boolean interrupt) {

        M_DriveTrain.tankDrive(0, 0);

    }

    @Override
    public boolean isFinished() {


        // Sets the distance to zerox
        double DistanceInInches = M_DriveTrain.GetEncoderInches();

        // While our desired distance is greater than our current distance keep running
        // the drive motors
        return (DesiredDistance <= DistanceInInches);

    }
}
