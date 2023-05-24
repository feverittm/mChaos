/**
Copyright 2023 FRC Team 997

This program is free software: 
you can redistribute it and/or modify it under the terms of the 
GNU General Public License as published by the Free Software Foundation, 
either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, 
but WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. 
If not, see <https://www.gnu.org/licenses/>.
*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;

/** */
public class Launcher extends SubsystemBase {
  private final CANSparkMax forwardFlywheelMotor = new CANSparkMax(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_CANID,
      MotorType.kBrushless);

  private final RelativeEncoder flywheelEncoder = forwardFlywheelMotor.getEncoder();
  private final LinearFilter flywheelVelocityFilter = LinearFilter.movingAverage(
      LauncherConstants.FLYWHEEL_ENCODER_VELOCITY_SMOOTHING_SAMPLES);

  /** */
  public Launcher() {
    forwardFlywheelMotor.setInverted(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_INVERTED);
  }

  /**
   * @param voltage
   */
  public void setFlywheelVoltage(double voltage) {
    forwardFlywheelMotor.setVoltage(voltage);
  }

  /**
   * @return
   */
  public double getFlywheelVelocityRadiansPerSecond() {
    return flywheelVelocityFilter.calculate(
        LauncherConstants.MOTOR_TO_FLYWHEEL.outputFromInput(
            Units.rotationsPerMinuteToRadiansPerSecond(
                flywheelEncoder.getVelocity())));
  }

    /**
   * @return
   */
  public double getFlywheelVelocity() {
    return flywheelVelocityFilter.calculate(
        LauncherConstants.MOTOR_TO_FLYWHEEL.outputFromInput(
                flywheelEncoder.getVelocity()));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish encoder distances to telemetry.
    builder.addDoubleProperty("shooter_velocity", flywheelEncoder::getVelocity, null);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Velocity (RPM)", flywheelEncoder.getVelocity());
    SmartDashboard.putNumber("Flywheel Velocity", getFlywheelVelocityRadiansPerSecond());
  }
}