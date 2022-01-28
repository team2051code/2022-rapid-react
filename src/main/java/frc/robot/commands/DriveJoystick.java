package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveTrain;

public class DriveJoystick extends CommandBase {
    private DriveTrain M_DriveTrain;
    private XboxController M_Controller;

    public DriveJoystick(DriveTrain driveTrain, XboxController controller) {
        M_DriveTrain = driveTrain;
        M_Controller = controller;
        // Commands that are to be set as default on a subsystem *must* be declared
        // as having that subsystem as a requirement, so we do that here when the command
        // is created.
        addRequirements(M_DriveTrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        M_DriveTrain.tankDrive(M_Controller.getRawAxis(1), M_Controller.getRawAxis(5));
    }

    @Override
    public void end(boolean interrupt) {

    }

    @Override
    public boolean isFinished() {
        return false;

    }
}
