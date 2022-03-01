package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.ShootParamaters;

public class Drive extends CommandBase {

    private DriveTrain M_DriveTrain;
   // public ShootParamaters M_Shoot;

    // Addition of the distances to group them together
    private double m_DesiredDistance;

   

    public Drive(DriveTrain DriveTrain, /*ShootParamaters Shootparamaters*/ double DistanceInches) {
        //M_Shoot = Shootparamaters;
        M_DriveTrain = DriveTrain;
        m_DesiredDistance = DistanceInches;

    }

    @Override
    public void initialize() {
        M_DriveTrain.TestingMotors();

        System.out.println("Running");

    }

    @Override
    public void execute() {


            if (m_DesiredDistance > 0) {

                M_DriveTrain.tankDrive(.5, .5);
            } else {
    
                M_DriveTrain.tankDrive(-.5, -.5);
            }


        

       

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
        if (m_DesiredDistance < 0) {

            return (DistanceInInches <= m_DesiredDistance);
        }

        return (DistanceInInches >= m_DesiredDistance);

    }
}
