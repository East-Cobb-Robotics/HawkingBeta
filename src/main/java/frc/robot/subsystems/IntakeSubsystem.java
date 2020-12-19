package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final static int GRIPPER_FORWARD_ID = 7; // TODO: Get ID
    private final static int GRIPPER_REVERSE_ID = 0; // TODO: Get ID
    private final static int HINGE_FORWARD_ID = 6; // TODO: Get ID
    private final static int HINGE_REVERSE_ID = 1; // TODO: Get ID

    private DoubleSolenoid gripper;
    private DoubleSolenoid hinge;

    public IntakeSubsystem() {
        gripper = new DoubleSolenoid(GRIPPER_FORWARD_ID, GRIPPER_REVERSE_ID);
        hinge = new DoubleSolenoid(HINGE_FORWARD_ID, HINGE_REVERSE_ID);
    }

    public void setHinge(boolean value) {
        hinge.set(value ? Value.kForward : Value.kReverse);
    }

    public void setGripper(boolean value) {
        gripper.set(value ? Value.kForward : Value.kReverse);
    }

    public boolean isGripperClosed() {
        return gripper.get() == Value.kForward;
    }
    
    public boolean isGripperLifted() {
        return hinge.get() == Value.kForward;
    }

    // DEBUG
//    private void debug(String message) {
//        CommandScheduler.getInstance().schedule(new PrintCommand("GRIPPER: " + message));
//    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
