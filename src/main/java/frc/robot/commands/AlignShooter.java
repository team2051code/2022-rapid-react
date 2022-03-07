package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ShootParamaters;

public class AlignShooter extends CommandBase {
    private ShootParamaters m_shootParamaters;

    public AlignShooter(ShootParamaters shootParamaters) {
        m_shootParamaters = shootParamaters;
        addRequirements(m_shootParamaters);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_shootParamaters.setTurretRotatorSpeed(0);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        m_shootParamaters.updateTurretRotation();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return m_shootParamaters.isTurretCentered();
    }

}
