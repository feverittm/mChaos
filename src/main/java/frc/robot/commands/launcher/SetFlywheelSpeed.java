// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Launcher;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetFlywheelSpeed extends PIDCommand {
  /** Creates a new SetFlywheelSpeed. */
  public SetFlywheelSpeed(int ShooterRPM, Launcher c_launcher) {
    super(
        // The controller that the command will use
        new PIDController(Constants.LauncherConstants.FLYWHEEL_KP
            , Constants.LauncherConstants.FLYWHEEL_KI
            , Constants.LauncherConstants.FLYWHEEL_KD
            ),
        // This should return the measurement
        c_launcher::getFlywheelVelocity,
        // This should return the setpoint (can also be a constant)
        ShooterRPM,
        // This uses the output
        output -> c_launcher.setFlywheelVoltage(output),
        c_launcher 
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
