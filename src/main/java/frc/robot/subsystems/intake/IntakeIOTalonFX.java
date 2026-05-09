package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
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

    tryUntilOk(5, () -> intake.getConfigurator().apply(IntakeConstants.INTAKE_CONFIG));

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
