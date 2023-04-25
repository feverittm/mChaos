// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Launcher;

public class SetHoodPosition extends PIDCommand {

  /** Creates a new SetHoodPosition. */
  public SetHoodPosition(int hoodAngle, Launcher m_launcher) {
    super(
        // The controller that the command will use
        new PIDController(Constants.LauncherConstants.HOOD_KP
              , Constants.LauncherConstants.HOOD_KI
              , Constants.LauncherConstants.HOOD_KD
            ),
        // This should return the measurement
        m_launcher::getHoodPosition,
        // This should return the setpoint (can also be a constant)
        hoodAngle,
        // This uses the output
        output -> m_launcher.setHoodVoltage(output),
        m_launcher 
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
