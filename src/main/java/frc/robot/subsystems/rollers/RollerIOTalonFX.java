package frc.robot.subsystems.rollers;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class RollerIOTalonFX implements RollerIO {
  private final TalonFX leader;
  private final TalonFX[] followers;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final CoastOut coastRequest = new CoastOut();
  private final StaticBrake brakeRequest = new StaticBrake();

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final BaseStatusSignal[] followerTemps;

  public RollerIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, new int[0], config, new MotorAlignmentValue[0]);
  }

  public RollerIOTalonFX(
      CANBus canbus,
      int id,
      int[] followerIds,
      TalonFXConfiguration config,
      MotorAlignmentValue[] followerAlignments) {
    // Instantiate motors
    leader = new TalonFX(id, canbus);
    followers = new TalonFX[followerIds.length];
    for (int i = 0; i < followers.length; i++) {
      followers[i] = new TalonFX(id, canbus);
    }
    // Configure motors
    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    for (TalonFX follower : followers) {
      follower.getConfigurator().apply(config);
    }
    // Create status signals
    velocity = leader.getVelocity();
    voltage = leader.getMotorVoltage();
    supplyCurrent = leader.getSupplyCurrent();
    statorCurrent = leader.getStatorCurrent();
    temp = leader.getDeviceTemp();
    followerTemps = new BaseStatusSignal[followers.length];
    for (int i = 0; i < followerTemps.length; i++) {
      followerTemps[i] = followers[i].getDeviceTemp();
    }
    // Register status signals
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, velocity, voltage, supplyCurrent, statorCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerTemps);
    leader.optimizeBusUtilization();
    ParentDevice.optimizeBusUtilizationForAll(followers);
    PhoenixUtil.registerSignals(canbus, velocity, voltage, supplyCurrent, statorCurrent, temp);
    PhoenixUtil.registerSignals(canbus, followerTemps);
    // Set follower behavior
    for (int i = 0; i < followers.length; i++) {
      followers[i].setControl(new Follower(leader.getDeviceID(), followerAlignments[i]));
    }
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.isAllGood(velocity, voltage, supplyCurrent, statorCurrent, temp);
    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();

    for (int i = 0; i < followerTemps.length; i++) {
      inputs.followerConnected[i] = followerTemps[i].getStatus().isOK();
      inputs.followerTempCelsius[i] = followerTemps[i].getValueAsDouble();
    }
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(double rps) {
    leader.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void coast() {
    leader.setControl(coastRequest);
  }

  @Override
  public void brake() {
    leader.setControl(brakeRequest);
  }

  @Override
  public int getNumFollowers() {
    return followers.length;
  }
}
