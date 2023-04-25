// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.launcher.SetHoodPosition;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class mChaos {
  // The robot's subsystems
  private final Drive m_drive = new Drive();
  private final Indexer m_indexer = new Indexer();
  private final Launcher m_launcher = new Launcher();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  // Retained command references
  private final Command m_hoodStop = Commands.runOnce(() -> m_launcher.setHoodVoltage(0.0));
  private final Command m_hoodUp = Commands.runOnce(() -> m_launcher.setHoodVoltage(0.3));
  private final Command m_hoodDown = Commands.runOnce(() -> m_launcher.setHoodVoltage(-0.3));
  
  private final Command m_indexBall = Commands.runOnce(() -> m_indexer.indexBall());
  private final Command m_stopIndex = Commands.runOnce(() -> m_indexer.stopIndexer());

  private final Command m_setHoodPosition = new SetHoodPosition(100, m_launcher);

  // Autonomous commands
  private final Command m_simpleAuto = Commands.run(
    // Drive forward for 2 meters at half speed with a 3 second timeout
    () -> m_drive.driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed)
      .withTimeout(AutoConstants.kTimeoutSeconds),
      m_drive
  );

  SendableChooser<Command> m_chooser = new SendableChooser<>();
 
  /**
   * Use this method to define bindings between conditions and commands. These are
   * useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>
   * Should be called during {@link Robot#robotInit()}.
   *
   * <p>
   * Event binding methods are available on the {@link Trigger} class.
   */

  public mChaos() {
    configureBindings();

    m_chooser.setDefaultOption("Simple Auto", m_simpleAuto);

    // Control the drive with split-stick arcade controls
    m_drive.setDefaultCommand(
      Commands.run(
        () ->
          m_drive.arcadeDriveCommand(
            -m_driverController.getLeftY(), -m_driverController.getRightX()),
          m_drive));

    Shuffleboard.getTab("Autonomous").add(m_chooser);
    Shuffleboard.getTab("Drive").add(m_drive);
    Shuffleboard.getTab("Launcher").add(m_launcher);
    Shuffleboard.getTab("Indexer").add(m_indexer);

    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command ->
                Shuffleboard.addEventMarker(
                    "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));
  }

  public void configureBindings() {
    // Automatically run the storage motor whenever the ball storage is not full,
    // and turn it off whenever it fills.
    // new Trigger(m_storage::isFull).whileFalse(m_storage.runCommand());

    // Automatically disable and retract the intake whenever the ball storage is
    // full.
    // new Trigger(m_storage::isFull).onTrue(m_intake.retractCommand());


    m_driverController.a().onTrue(m_hoodUp).onFalse(m_hoodStop);
    m_driverController.b().onTrue(m_hoodDown).onFalse(m_hoodStop);

    m_driverController.x().onTrue(m_setHoodPosition);

    m_driverController.y().onTrue(m_indexBall).onFalse(m_stopIndex);

    /*
     * // Fire the shooter with the A button
     * m_driverController
     * .x()
     * .onTrue(
     * parallel(
     * m_launcher.shootCommand(LauncherConstants.kShooterTargetRPS))
     * // m_storage.runCommand())
     * // Since we composed this inline we should give it a name
     * .withName("Shoot"));
     * 
     */
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>
   * Scheduled during {@link Robot#autonomousInit()}.
   */
  public CommandBase getAutonomousCommand() {
    // Drive forward for 2 meters at half speed with a 3 second timeout
    return m_drive
        .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed)
        .withTimeout(AutoConstants.kTimeoutSeconds);
  }
}
