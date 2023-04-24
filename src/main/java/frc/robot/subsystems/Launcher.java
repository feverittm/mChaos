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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Robot;

/** */
public class Launcher extends SubsystemBase {
  private final CANSparkMax forwardFlywheelMotor = new CANSparkMax(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_CANID,
      MotorType.kBrushless);

  private final CANSparkMax aftFlywheelMotor = new CANSparkMax(LauncherConstants.AFT_FLYWHEEL_MOTOR_CANID,
      MotorType.kBrushless);

  private final RelativeEncoder flywheelEncoder = forwardFlywheelMotor.getEncoder();
  private final LinearFilter flywheelVelocityFilter = LinearFilter.movingAverage(
      LauncherConstants.FLYWHEEL_ENCODER_VELOCITY_SMOOTHING_SAMPLES);

  private final TalonSRX hoodMotor = new TalonSRX(LauncherConstants.HOOD_MOTOR_CANID);

  private final DigitalInput hoodLimitSwitch = new DigitalInput(LauncherConstants.HOOD_LIMIT_SWITCH_DIO);

  private final Encoder hoodAngleEncoder = new Encoder(LauncherConstants.HOOD_ENCODER_DIO_CHANNEL[0],
      LauncherConstants.HOOD_ENCODER_DIO_CHANNEL[1]);

  private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(
      DCMotor.getBag(1),
      (1 / LauncherConstants.MOTOR_TO_HOOD.toDoubleRatioInputToOutput()),
      LauncherConstants.HOOD_MOMENT,
      LauncherConstants.HOOD_RADIUS_METERS,
      0,
      LauncherConstants.HOOD_MAX_ANGLE,
      true,
      VecBuilder.fill(LauncherConstants.HOOD_ENCODER_ERROR_RADIANS_STDDEV));

  private final FlywheelSim flywheelSim = new FlywheelSim(
      DCMotor.getNEO(2),
      1 / LauncherConstants.MOTOR_TO_FLYWHEEL.toDoubleRatioInputToOutput(),
      LauncherConstants.FLYWHEEL_MOMENT,
      VecBuilder.fill(LauncherConstants.FLYWHEEL_ENCODER_ERROR_RADIANS_STDDEV));

  private double hoodAbsDistanceRadians = 0;
  private double flywheelAbsDisanceRadians = 0;

  private double previousHoodAngleRadians = 0;
  private double previousFlywheelVelocityRadiansPerSecond = 0;

  private double approximateFlywheelAcceleration = 0;

  /** */
  public Launcher() {
    forwardFlywheelMotor.setInverted(LauncherConstants.FORWARD_FLYWHEEL_MOTOR_INVERTED);
    aftFlywheelMotor.setInverted(LauncherConstants.AFT_FLYWHEEL_MOTOR_INVERTED);
    aftFlywheelMotor.follow(forwardFlywheelMotor);

    hoodMotor.setInverted(LauncherConstants.HOOD_MOTOR_INVERTED);
    hoodMotor.configSelectedFeedbackSensor(LauncherConstants.HOOD_FEEDBACK_DEVICE);
    hoodMotor.setSensorPhase(LauncherConstants.HOOD_ENCODER_INVERTED);
  }

  /**
   * @return
   */
  public boolean hoodLimitSwitchActivated() {
    SmartDashboard.putBoolean("Hood Limit Switch", hoodLimitSwitch.get());
    if (LauncherConstants.HOOD_LIMIT_SWITCH_TRUE_WHEN_ACTIVATED)
      return hoodLimitSwitch.get();
    else
      return !hoodLimitSwitch.get();
  }

  /**
   * @param voltage
   */
  public void setFlywheelVoltage(double voltage) {
    forwardFlywheelMotor.setVoltage(voltage);
    aftFlywheelMotor.setVoltage(voltage);

    flywheelSim.setInputVoltage(voltage);
  }

  /** */
  public void setHoodVoltage(double voltage) {
    double toApply;

    if (getHoodPosition() >= LauncherConstants.HOOD_MAX_ANGLE && voltage > 0) {
      toApply = 0; // if at or above max angle, cancel all + (up) voltages
    } else if (Robot.isReal()) {
      if (hoodLimitSwitchActivated() && voltage < 0) {
        toApply = 0;
      } else {
        toApply = voltage;
      } // if limit switch active (or at lower limit in sim), cancel all - (down)
        // voltages
    } else {
      if (getHoodPosition() <= 0 && voltage < 0) {
        toApply = 0;
      } else {
        toApply = voltage;
      }
    }

    hoodSim.setInputVoltage(toApply);
    hoodMotor.set(ControlMode.PercentOutput, toApply / RobotController.getBatteryVoltage());
  }

  /**
   * @return
   */
  public double getHoodPosition() {
    if (Robot.isReal()) {
      return LauncherConstants.MOTOR_TO_HOOD.outputFromInput(
          hoodAngleEncoder.get() / LauncherConstants.HOOD_ENCODER_CPR);
    } else {
      return hoodSim.getAngleRads();
    }
  }

  /**
   * @return
   */
  public double getFlywheelVelocityRadiansPerSecond() {
    if (Robot.isReal()) {
      return flywheelVelocityFilter.calculate(
          LauncherConstants.MOTOR_TO_FLYWHEEL.outputFromInput(
              Units.rotationsPerMinuteToRadiansPerSecond(
                  flywheelEncoder.getVelocity())));
    } else {
      return flywheelSim.getAngularVelocityRadPerSec();
    }
  }

  /**
   * @return
   */
  public double getFlywheelAccelerationRadiansPerSecondSquared() {
    return approximateFlywheelAcceleration;
  }

  /**
   * @return
   */
  public double getHoodAbsDistanceRadians() {
    return hoodAbsDistanceRadians;
  }

  /**
   * @return
   */
  public double getFlywheelAbsDistanceRadians() {
    return flywheelAbsDisanceRadians;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Hood Limit Switch", hoodLimitSwitch.get());
    SmartDashboard.putNumber("Hood Position:", hoodAngleEncoder.get());
    SmartDashboard.putNumber("Calc Hood Position", getHoodPosition());
    
    hoodAbsDistanceRadians += Math.abs(getHoodPosition() - previousHoodAngleRadians);
    flywheelAbsDisanceRadians += Math.abs(getFlywheelVelocityRadiansPerSecond() * 0.02);

    approximateFlywheelAcceleration = (getFlywheelVelocityRadiansPerSecond() - previousFlywheelVelocityRadiansPerSecond)
        / 0.02;

    previousHoodAngleRadians = getHoodPosition();
    previousFlywheelVelocityRadiansPerSecond = getFlywheelVelocityRadiansPerSecond();
  }

  @Override
  public void simulationPeriodic() {
    hoodSim.update(0.02);
    flywheelSim.update(0.02);
  }
}