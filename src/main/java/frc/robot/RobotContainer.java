package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.DriveArcadeCommand;
import frc.robot.commands.intake.GripperCommand;
import frc.robot.commands.intake.LifterCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.util.RobotControls;

/**
 * Wraps all the subsystems and commands for the robot.
 */
public class RobotContainer {
  // Note: The robot container is a functional description of the robot.
  // It should not describe how to something but what the robot does
  // when a particular inputs is received.

  ////////////////
  // Subsystems //
  ////////////////
  private DriveSubsystem drive = new DriveSubsystem();
  private IntakeSubsystem intake = new IntakeSubsystem();
  
  private RobotControls controls = new RobotControls(true);
  private Button shifterButton = new Button(controls::getShifterButton);
  private Button gripperButton = new Button(controls::getGripperButton);
  private Button lifterButton = new Button(controls::getLifterButton);

  private enum DriveType {
    TANK, ARCADE 
  }

  private SendableChooser<DriveType> driveTypeChooser
      = new SendableChooser<>();

  /**
   * Default constructor for RobotContainer.
   */
  public RobotContainer() {
    driveTypeChooser.setDefaultOption("Tank", DriveType.TANK);
    SmartDashboard.putData(driveTypeChooser);

    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    shifterButton.whileActiveOnce(new StartEndCommand(
        () -> drive.setShifter(!drive.getShifter()),
        () -> {})
    );

    gripperButton.whenPressed(new GripperCommand(gripperButton::get, intake));
    lifterButton.whenPressed(new LifterCommand(lifterButton::get, intake));
  }

  /**
   * Configure the default commands for each subsystem. The default commands
   * should be the commands run for during tellop.
   */
  private void configureDefaultCommands() {

    // Configure the default command to drive based off of what drive system
    // the user currently has selected.
    // Note that we only want to use one drive type in competition for performance.

    // The selector right now give a null pointer exception. TODO: Look into.
    /*
    drive.setDefaultCommand(new SelectCommand(
        Map.ofEntries(
          Map.entry(DriveType.TANK, new DriveTankCommand(controls::getLeftDriverY,
                                                         controls::getRightDriverY,
                                                         drive)),
          Map.entry(DriveType.ARCADE, new DriveArcadeCommand(controls::getLeftDriverY,
                                                             controls::getRightDriverX,
                                                             drive))
          ),
          driveTypeChooser::getSelected
    ));
    */
    drive.setDefaultCommand(new DriveArcadeCommand(controls::getLeftDriverY,
                                                   controls::getRightDriverX,
                                                   drive));
  }

  /**
   * Get the command to run for auto.
   *
   * @return The command to be run for auto.
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("Auto would run now.");
  }
}
