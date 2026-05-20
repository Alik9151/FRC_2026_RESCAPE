package frc.robot.util.io.motors;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.pivot.PivotIO;
import frc.robot.util.io.motors.roller.RollerIO;
import frc.robot.util.io.sensors.EncoderIO;

public class MotorIOSparkMax implements PivotIO, RollerIO {
  private static final SparkBaseConfig coastConfig =
      new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kCoast);
  private static final SparkBaseConfig brakeConfig =
      new SparkMaxConfig().idleMode(SparkBaseConfig.IdleMode.kBrake);

  private final SparkMax leader;
  private final SparkMax[] followers;

  private final SparkClosedLoopController controller;
  private SparkBase.ControlType positionControlType = SparkBase.ControlType.kPosition;

  private final RelativeEncoder leaderEncoder;

  private boolean brakeMode;

  public MotorIOSparkMax(
      int id, int[] followerIds, SparkMaxConfig config, boolean[] followersOpposed) {
    leader = new SparkMax(id, SparkLowLevel.MotorType.kBrushless);
    followers = new SparkMax[followerIds.length];
    for (int i = 0; i < followers.length; i++) {
      followers[i] = new SparkMax(followerIds[i], SparkLowLevel.MotorType.kBrushless);
    }

    tryUntilOk(
        5,
        () ->
            leader.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    for (int i = 0; i < followers.length; i++) {
      config.follow(leader.getDeviceId(), followersOpposed[i]);
      followers[i].configure(
          config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    controller = leader.getClosedLoopController();

    leaderEncoder = leader.getEncoder();
    leaderEncoder.setPosition(0);
  }

  public MotorIOSparkMax withPositionControlType(SparkBase.ControlType controlType) {
    switch (controlType) {
      case kPosition:
      case kMAXMotionPositionControl:
        this.positionControlType = controlType;
        break;
      default:
        throw new IllegalArgumentException("ControlType not supported: " + controlType);
    }
    return this;
  }

  private void updateMotorInputs(MotorIOInputs inputs) {
    inputs.connected = !leader.hasActiveFault();
    inputs.appliedVoltage = leader.getBusVoltage() * leader.getAppliedOutput();
    inputs.statorCurrentAmps = leader.getOutputCurrent();
    inputs.tempCelsius = leader.getMotorTemperature();

    for (int i = 0; i < followers.length; i++) {
      inputs.followerConnected[i] = !followers[i].hasActiveFault();
      inputs.followerTempCelsius[i] = followers[i].getMotorTemperature();
    }
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionDeg = leaderEncoder.getPosition();
    updateMotorInputs(inputs);
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocityRPS = leaderEncoder.getVelocity();
    updateMotorInputs(inputs);
  }

  @Override
  public void setVoltage(double volts) {
    leader.setVoltage(volts);
  }

  @Override
  public void setPosition(double deg) {
    controller.setSetpoint(deg, positionControlType);
  }

  @Override
  public void setVelocity(double rps) {
    controller.setSetpoint(rps, SparkBase.ControlType.kVelocity);
  }

  @Override
  public void coast() {
    if (brakeMode) {
      leader.configureAsync(
          coastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      for (SparkMax follower : followers) {
        follower.configureAsync(
            coastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    }
    brakeMode = false;
    leader.stopMotor();
  }

  @Override
  public void brake() {
    if (!brakeMode) {
      leader.configureAsync(
          brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      for (SparkMax follower : followers) {
        follower.configureAsync(
            brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    }
    brakeMode = true;
    leader.stopMotor();
  }

  @Override
  public void resetPosition(Angle angle) {
    leaderEncoder.setPosition(angle.in(Degrees));
  }

  @Override
  public int getNumFollowers() {
    return followers.length;
  }

  public EncoderIO getAbsoluteEncoder() {
    AbsoluteEncoder encoder = leader.getAbsoluteEncoder();
    return (inputs) -> {
      inputs.connected = !leader.hasActiveFault();
      inputs.position = Degrees.of(encoder.getPosition());
    };
  }
}
