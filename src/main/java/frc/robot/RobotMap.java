package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {
    public static final int XboxControllerPort = 0; 
    private static XboxController controller = new XboxController(RobotMap.XboxControllerPort);

    public static final int Motor_Right = 1;
    public static final int Motor_RightFollow = 2;
    public static final int Motor_Left = 3;
    public static final int Motor_LeftFollow = 4;
    public static final int XboxControllerUsbPort = 0;

    public static final int ShootingMotor1 = 1;
    public static final int ShootingMotor2 = 5;
    public static final int LeftAxis = 1;
    public static final int RightAxis = 5;

    public static final boolean AButton = controller.getAButton(); 
}
