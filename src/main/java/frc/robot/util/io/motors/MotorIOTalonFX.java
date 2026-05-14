package frc.robot.util.io.motors;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.roller.RollerIO;

public class MotorIOTalonFX implements PivotIO, RollerIO {
  private final TalonFX leader;
  private final TalonFX[] followers;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final CoastOut coastRequest = new CoastOut();
  private final StaticBrake brakeRequest = new StaticBrake();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final BaseStatusSignal[] followerTemps;

  public MotorIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, new int[0], config, new MotorAlignmentValue[0]);
  }

  public MotorIOTalonFX(
      CANBus canbus,
      int id,
      int[] followerIds,
      TalonFXConfiguration config,
      MotorAlignmentValue[] followerAlignments) {
    // Instantiate motors
    leader = new TalonFX(id, canbus);
    followers = new TalonFX[followerIds.length];
    for (int i = 0; i < followers.length; i++) {
      followers[i] = new TalonFX(followerIds[i], canbus);
    }
    // Configure motors
    tryUntilOk(5, () -> leader.getConfigurator().apply(config));
    for (TalonFX follower : followers) {
      follower.getConfigurator().apply(config);
    }
    // Create status signals
    position = leader.getPosition();
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
        100.0, position, velocity, voltage, supplyCurrent, statorCurrent, temp);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, followerTemps);
    leader.optimizeBusUtilization();
    ParentDevice.optimizeBusUtilizationForAll(followers);
    PhoenixUtil.registerSignals(
        canbus, position, velocity, voltage, supplyCurrent, statorCurrent, temp);
    PhoenixUtil.registerSignals(canbus, followerTemps);
    leader.setPosition(0);
    // Set follower behavior
    for (int i = 0; i < followers.length; i++) {
      followers[i].setControl(new Follower(leader.getDeviceID(), followerAlignments[i]));
    }
  }

  private void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.isAllGood(position, velocity, voltage, supplyCurrent, statorCurrent, temp);
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
  public void updateInputs(PivotIOInputs inputs) {
    updateMotorInputs(inputs);
    inputs.positionDeg = velocity.getValueAsDouble();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    updateMotorInputs(inputs);
    inputs.velocityRPS = velocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    leader.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(double deg) {
    leader.setControl(positionRequest.withPosition(Units.degreesToRotations(deg)));
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
