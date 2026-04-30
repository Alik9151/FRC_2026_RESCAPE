package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final StaticBrake brakeRequest = new StaticBrake();
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(0).withOverrideBrakeDurNeutral(true);

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Temperature> followerTemp;

  public ElevatorIOTalonFX() {
    leader = new TalonFX(0);
    follower = new TalonFX(0);

    tryUntilOk(5, () -> leader.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG));
    tryUntilOk(5, () -> follower.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    statorCurrent = leader.getStatorCurrent();
    supplyCurrent = leader.getSupplyCurrent();
    temp = leader.getDeviceTemp();
    followerAppliedVolts = follower.getMotorVoltage();
    followerStatorCurrent = follower.getStatorCurrent();
    followerSupplyCurrent = follower.getSupplyCurrent();
    followerTemp = follower.getDeviceTemp();

    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        position,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);
    ParentDevice.optimizeBusUtilizationForAll(leader, follower);

    PhoenixUtil.registerSignals(
        Constants.CANConstants.SUPERSTRUCTURE_CAN_BUS,
        position,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        temp,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);

    leader.setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var leaderStatus =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    var followerStatus =
        BaseStatusSignal.refreshAll(
            followerAppliedVolts, followerStatorCurrent, followerSupplyCurrent, followerTemp);

    inputs.leaderConnected = leaderStatus.isOK();
    inputs.followerConnected = followerStatus.isOK();

    inputs.positionRad = position.getValue().in(Radians);
    inputs.velocityRadPerSec = velocity.getValue().in(RadiansPerSecond);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double output) {
    leader.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setPosition(double positionRad) {
    leader.setControl(positionRequest.withPosition(Units.radiansToRotations(positionRad)));
  }

  @Override
  public void setPosition(Angle position) {
    leader.setControl(positionRequest.withPosition(position.in(Rotations)));
  }

  @Override
  public void stop() {
    leader.setControl(brakeRequest);
  }

  @Override
  public void resetPosition(Angle newPosition) {
    new Thread(() -> leader.setPosition(newPosition)).start();
  }

  @Override
  public void setBrake(boolean brake) {
    new Thread(() -> leader.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }
}
