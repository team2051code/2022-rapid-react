package frc.robot;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase {
  private static final double DEADZONE = .18;
  public boolean m_toggleOn = false;
  boolean m_manualModeTogglePressed = false;

  private XboxController m_controller = new XboxController(RobotMap.XBOX_CONTROLLER_PORT);
  private XboxController m_controller2 = new XboxController(RobotMap.XBOX_CONTROLLER_PORT_2);
  boolean m_toggleReady = m_controller.getYButton();
  private boolean m_manualAimMode = false;
  private double m_targetingOffset = 0;

  public double getDriverRawAxis(int axis) {
    return m_controller.getRawAxis(axis);
  }

  public double GetTurretRotationAxis() {
    return Math.pow(m_controller2.getRawAxis(RobotMap.Turning), 3) / 2;
  }

  public boolean getAButton2() {
    return m_controller2.getAButton();
  }

  public boolean getBButton() {
    return m_controller.getBButton();
  }

  public boolean getXButton() {
    return m_controller.getXButton();
  }

  public boolean getYButton() {
    return m_controller.getYButton();
  }

  public boolean getLeftBumper() {
    return m_controller.getLeftBumper();
  }

  public boolean getLeftBumper2() {
    return m_controller2.getLeftBumper();
  }

  public boolean getRightBumper() {
    return m_controller.getRightBumper();
  }

  public boolean getRightBumper2() {
    return m_controller2.getRightBumper();
  }

  public double getRightTrigger2() {
    return m_controller2.getRightTriggerAxis();
  }

  public boolean stickClick() {
    return m_controller.getRightStickButton();
  }

  public boolean getXButton2() {
    return m_controller2.getXButton();
  }

  public boolean getBackButton2() {
    return m_controller2.getBackButton();
  }

  public boolean GetStartButton2() {
    return m_controller2.getStartButton();
  }

  public boolean GetAButton() {
    return m_controller.getAButton();
  }

  public boolean GetBButton2() {
    return m_controller2.getBButton();
  }

  public boolean manualAimMode() {

    return m_manualAimMode;

  }

  @Override
  public void periodic() {

    if(m_manualAimMode){
      m_targetingOffset -= (Math.pow(deadzone(DEADZONE, m_controller2.getLeftY()), 3));
    }




    if (m_controller2.getLeftStickButtonPressed()) {
      System.out.println("Toggle");
      m_manualAimMode = !m_manualAimMode;

    }
    SmartDashboard.putBoolean("ManualAim?", m_manualAimMode);
    SmartDashboard.putNumber("TargetingOffset", m_targetingOffset);

  }
  

  public double targetingOffset() {

    return m_targetingOffset;
  }

  /**
   * Outputs 0 if input is in deadzone, otherwise outputs input
   * @param threshold Positive threshold value. Values -threshold < input < threshold output as 0
   * @param input input to check for deadzone
   * @return input checked against deadzone
   */
  private double deadzone(double threshold, double input) {
    return Math.abs(input) < threshold ? 0 : input;
  }

}
