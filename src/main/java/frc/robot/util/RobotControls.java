package frc.robot.util;
  
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * A loose wrapper for the joysticks to control the robot.
 * The all functions should not have any parameters so that
 * they can be used as a supplier for commands.
 */
public class RobotControls {

  private static final int LEFT_OPERATOR_JOYSTICK_ID = 3;
  private static final int RIGHT_OPERATOR_JOYSTICK_ID = 4;
  private static final int LEFT_DRIVER_JOYSTICK_ID = 0;
  private static final int RIGHT_DRIVER_JOYSTICK_ID = 1;

  private static final int kDriveShiftButton = 11;
  private static final int kGripperButton = 0; // TODO: Get ID
  private static final int kLifterButton = 0; // TODO: Get ID

  private Joystick leftDriverJoystick;
  private Joystick rightDriverJoystick;
  private Joystick leftOperatorJoystick;
  private Joystick rightOperatorJoystick;
  private XboxController xboxController;

  private boolean useXbox;

  /**
   * Controllers RobotControls.
   *
   * @param useXbox Weather to use an xBox controller or joysticks.
   */
  public RobotControls(boolean useXbox) {
    this.useXbox = useXbox;
    if (this.useXbox) {
      xboxController = new XboxController(LEFT_DRIVER_JOYSTICK_ID);
    } else {
      leftDriverJoystick = new Joystick(LEFT_DRIVER_JOYSTICK_ID);
      rightDriverJoystick = new Joystick(RIGHT_DRIVER_JOYSTICK_ID);
      leftOperatorJoystick = new Joystick(LEFT_OPERATOR_JOYSTICK_ID);
      rightOperatorJoystick = new Joystick(RIGHT_OPERATOR_JOYSTICK_ID);
    }
  }
  
  public double getLeftDriverX() {
    return useXbox ? xboxController.getX(Hand.kLeft) :
        leftDriverJoystick.getX(Hand.kLeft);
  }

  public double getLeftDriverY() {
    return useXbox ? -xboxController.getY(Hand.kLeft) :
        -leftDriverJoystick.getY(Hand.kLeft);
  }

  public double getRightDriverX() {
    return useXbox ? xboxController.getRawAxis(2) :
        rightDriverJoystick.getX(Hand.kRight);
  }

  public double getRightDriverY() {
    return useXbox ? -xboxController.getRawAxis(3) :
        -rightDriverJoystick.getY(Hand.kRight);
  }

  public boolean getShifterButton() {
    return useXbox ? xboxController.getBumper(Hand.kRight) :
        leftDriverJoystick.getRawButton(kDriveShiftButton);
  }

  public boolean getGripperButton() {
    return useXbox ? xboxController.getAButton() :
            rightDriverJoystick.getRawButton(kGripperButton);
  }

  public boolean getLifterButton() {
    return useXbox ? xboxController.getYButton() :
            rightDriverJoystick.getRawButton(kLifterButton);
  }
}
