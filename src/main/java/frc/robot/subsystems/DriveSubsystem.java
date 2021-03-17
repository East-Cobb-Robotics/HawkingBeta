package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.NavX;
import frc.robot.util.SimEncoder;
import frc.robot.util.SimGyro;

public class DriveSubsystem extends SubsystemBase {

  // CONSTANTS //

  private static final int LEFT_MASTER_ID = 1;
  private static final int LEFT_SLAVE_ID = 2;
  private static final int RIGHT_MASTER_ID = 3;
  private static final int RIGHT_SLAVE_ID = 4;

  private static final int SHIFTER_FORWARD_ID = 4;
  private static final int SHIFTER_BACKWARD_ID = 3;
  private static final DoubleSolenoid.Value HIGH_GEAR_STATE = DoubleSolenoid.Value.kForward;
  private static final DoubleSolenoid.Value LOW_GEAR_STATE = DoubleSolenoid.Value.kReverse;

  private static final double KV_LINEAR = 0.2395; // TODO: Find out
  private static final double KA_LINEAR = 0.03624; // TODO: Find out
  private static final double KV_ANGULAR = KV_LINEAR; // TODO: Find out
  private static final double KA_ANGULAR = KA_LINEAR; // TODO: Find out

  private static final DCMotor GEAR_BOX = DCMotor.getCIM(2);
  private static final double HIGH_GEAR_GEARING = 8.6; // TODO: Measure

  private static final double WHEEL_DIAMETER = 0.1524; // meters
  private static final double EFFECTIVE_TRACK_WIDTH = 0.654; // meters
  private static final DifferentialDriveKinematics KINEMATICS
      = new DifferentialDriveKinematics(EFFECTIVE_TRACK_WIDTH);

  // END CONSTANTS //

  private WPI_TalonSRX leftMaster = new WPI_TalonSRX(LEFT_MASTER_ID);
  private WPI_TalonSRX leftSlave = new WPI_TalonSRX(LEFT_SLAVE_ID);
  private WPI_TalonSRX rightMaster = new WPI_TalonSRX(RIGHT_MASTER_ID);
  private WPI_TalonSRX rightSlave = new WPI_TalonSRX(RIGHT_SLAVE_ID);

  private DoubleSolenoid shifter = new DoubleSolenoid(SHIFTER_FORWARD_ID, SHIFTER_BACKWARD_ID);

  // These are set in constuctor because they depend on weather we are in
  // simulation or not.
  private NavX gyro;
  private DifferentialDriveOdometry odometry;

  private SimEncoder leftEncoderSim;
  private SimEncoder rightEncoderSim;
  private SimGyro gyroSim;
  private DifferentialDrivetrainSim sim;
  private Field2d field;

  /**
   * Configs all motor control settings and sets the robot to high gear.
   */
  public DriveSubsystem() {
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
        (int) (Robot.LOOP_TIME * 1000));
    leftMaster.setSelectedSensorPosition(0);
    leftMaster.setInverted(InvertType.InvertMotorOutput);

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
        (int) (Robot.LOOP_TIME * 1000));
    rightMaster.setSelectedSensorPosition(0);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    setShifter(true);
    leftSlave.setInverted(InvertType.FollowMaster);

    if (RobotBase.isSimulation()) {
      leftEncoderSim = new SimEncoder("Left Drive");
      rightEncoderSim = new SimEncoder("Right Drive");
      gyroSim = new SimGyro("NavX");
      odometry = new DifferentialDriveOdometry(gyroSim.getHeading());

      sim = new DifferentialDrivetrainSim(GEAR_BOX,
                                          HIGH_GEAR_GEARING,
                                          3,
                                          68,
                                          WHEEL_DIAMETER / 2,
                                          EFFECTIVE_TRACK_WIDTH,
                                          null);
      
      field = new Field2d();
    } else {
      gyro = new NavX(Port.kUSB);
      odometry = new DifferentialDriveOdometry(gyro.getYaw());
    }
  }

  /**
   * Set the shifters to high gear or low gear.
   *
   * @param highGear True if to use high gear false for low gear.
   */
  public void setShifter(boolean highGear) {
    shifter.set(highGear ? HIGH_GEAR_STATE : LOW_GEAR_STATE);
  }

  /**
   * Gets the current state of the shifter from the PCM.
   *
   * @return True if in high gear false if in low gear.
   */
  public boolean getShifter() {
    return shifter.get() == HIGH_GEAR_STATE;
  }

  @Override
  public void periodic() {
    if (RobotBase.isSimulation()) {
      odometry.update(gyroSim.getHeading(),
          leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
    } else {
      odometry.update(gyro.getYaw(), leftMaster.getSelectedSensorPosition(),
          rightMaster.getSelectedSensorPosition());
    }
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    sim.setInputs(leftMaster.get() * RobotController.getBatteryVoltage(),
          rightMaster.get() * RobotController.getBatteryVoltage());
    sim.update(Robot.LOOP_TIME);

    leftEncoderSim.setDistance(sim.getLeftPositionMeters());
    leftEncoderSim.setSpeed(sim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(sim.getRightPositionMeters());
    rightEncoderSim.setSpeed(sim.getRightVelocityMetersPerSecond());
    gyroSim.setHeading(sim.getHeading());

    field.setRobotPose(getPose());
  }

  /**
   * Drive the drivetrain in open loop (No feedback).
   *
   * @param left The power [-1.0, 1.0] to supply to the right side.
   * @param right The power [-1.0, 1.0] to supply to the right side.
   */
  public void driveOpenLoop(double left, double right) {
    leftMaster.set(left);
    rightMaster.set(right);
    if (RobotBase.isSimulation()) {
      leftSlave.set(leftMaster.get());
      rightSlave.set(rightMaster.get());
    }
  }

  /**
   * Set the robot to a new pose. Note that this will only 
   * set the x and y of the robot.
   *
   * @param newPose The pose to set the robot (x, y) to.
   */
  public void setOdometry(Pose2d newPose) {
    odometry.resetPosition(newPose, gyro.getYaw());
  }

  /**
   * Set's the gyro to a new heading.
   *
   * @param newHeading The Rotation2d to set the gyro to.
   */
  public void setGryo(Rotation2d newHeading) {
    if (RobotBase.isSimulation()) {
      gyroSim.setHeading(newHeading);
    } else {
      gyro.reset();
      gyro.setAngleAdjustment(newHeading);
    }
  }

  /**
   * Reset gryo and odometry to 0.
   */
  public void resetSensors() {
    setGryo(new Rotation2d());
    setOdometry(new Pose2d());
  }

  /**
   * Get the pose of the robot.
   *
   * @return Pose2d representing the robots position. (x and y are in meters)
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
}
