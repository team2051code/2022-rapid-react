package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SingulatorInformation;

public class StartSingulator extends CommandBase {
    private SingulatorInformation m_singulatorInformation;

    public StartSingulator(SingulatorInformation singulatorInformation) {
        m_singulatorInformation = singulatorInformation;
    }

    // intentionally no end; singulator will be running when this command exits

    @Override
    public void execute() {
        m_singulatorInformation.turnSingulatorOn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
