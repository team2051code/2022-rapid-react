package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShootParamaters;

public class SpinUpShooter extends CommandBase {
    private ShootParamaters m_shootParamaters;

    public SpinUpShooter(ShootParamaters shootParamaters) {
        m_shootParamaters = shootParamaters;
    }

    // no end implemented; shooter should keep running
    // when we exit this command


    @Override public void end(boolean interrupt) {
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub

        

        m_shootParamaters.setTargetSpeedToCalculatedSpeed();

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        
        return SmartDashboard.getBoolean("ShootReady", false);
    }

}
