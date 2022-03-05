package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
  public boolean m_toggleOn = false;
  boolean m_togglePressed = false;

  private XboxController m_controller = new XboxController(RobotMap.XBOX_CONTROLLER_PORT);
  private XboxController m_controller2 = new XboxController(RobotMap.XBOX_CONTROLLER_PORT_2);
  boolean m_toggleReady = m_controller.getYButton();

  public double getDriverRawAxis(int axis) {
    return m_controller.getRawAxis(axis);
  }

  public boolean getAButton2() {
    return m_controller2.getAButton();
  }

  public boolean getBButton() {
    return m_controller.getBButton();
  }

  public boolean getXButton() {
    return m_controller2.getXButton();
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

  public boolean GetStartButton2(){
    return m_controller2.getStartButton();
  }


  public void updateToggle() {

    if (m_controller.getYButton()) {
      if (!m_togglePressed) {
        m_toggleOn = !m_toggleOn;
        m_togglePressed = true;
      }
    } else {

      m_togglePressed = false;

    }

  }

}
