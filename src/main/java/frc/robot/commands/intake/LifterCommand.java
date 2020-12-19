package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

public class LifterCommand extends CommandBase {

    private IntakeSubsystem intake;
    private Supplier<Boolean> lifterButton;

    public LifterCommand(Supplier<Boolean> lifterButton, IntakeSubsystem intake) {
        this.intake = intake;
        this.lifterButton = lifterButton;
        addRequirements(this.intake);
    }

    @Override
    public void execute() {
        intake.setHinge(lifterButton.get());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setHinge(false);
    }

}
