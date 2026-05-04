package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.CANConstants;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intake;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<AngularVelocity> velocityRPS;

  public IntakeIOTalonFX() {
    intake = new TalonFX(CANConstants.INTAKE, CANConstants.SUPERSTRUCTURE_CAN_BUS);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    intakeConfig.MotorOutput.Inverted = IntakeConstants.INTAKE_INVERTED;
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.INTAKE_STATOR_LIMIT;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INTAKE_SUPPLY_LIMIT;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeConfig.Slot0.kP = IntakeConstants.INTAKE_KP;
    intakeConfig.Slot0.kI = IntakeConstants.INTAKE_KI;
    intakeConfig.Slot0.kD = IntakeConstants.INTAKE_KD;
    intakeConfig.Slot0.kS = IntakeConstants.INTAKE_KS;
    intakeConfig.Slot0.kV = IntakeConstants.INTAKE_KV;

    tryUntilOk(5, () -> intake.getConfigurator().apply(intakeConfig));

    voltage = intake.getMotorVoltage();
    statorCurrent = intake.getStatorCurrent();
    velocityRPS = intake.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, voltage, statorCurrent, velocityRPS);
    ParentDevice.optimizeBusUtilizationForAll(intake);

    PhoenixUtil.registerSignals(
        CANConstants.SUPERSTRUCTURE_CAN_BUS, voltage, statorCurrent, velocityRPS);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(voltage, statorCurrent, velocityRPS);
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.velocityRPS = velocityRPS.getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    intake.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    intake.stopMotor();
  }
}
