package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class GripperCommand extends CommandBase {

    private IntakeSubsystem intake;
    private Supplier<Boolean> gripperButton;

    public GripperCommand(Supplier<Boolean> gripperButton, IntakeSubsystem intake) {
        this.intake = intake;
        this.gripperButton = gripperButton;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.setGripper(gripperButton.get());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setGripper(false);
    }
}
