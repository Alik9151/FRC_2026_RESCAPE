package frc.robot.subsystems.outtake;

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

public class OuttakeIOTalonFX implements OuttakeIO {
  private final TalonFX outtake;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<AngularVelocity> velocityRPS;

  public OuttakeIOTalonFX() {
    outtake = new TalonFX(CANConstants.OUTTAKE, CANConstants.SUPERSTRUCTURE_CAN_BUS);

    tryUntilOk(5, () -> outtake.getConfigurator().apply(OuttakeConstants.OUTTAKE_CONFIG));

    voltage = outtake.getMotorVoltage();
    statorCurrent = outtake.getStatorCurrent();
    velocityRPS = outtake.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(100.0, voltage, statorCurrent, velocityRPS);
    ParentDevice.optimizeBusUtilizationForAll(outtake);

    PhoenixUtil.registerSignals(
        CANConstants.SUPERSTRUCTURE_CAN_BUS, voltage, statorCurrent, velocityRPS);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.connected = BaseStatusSignal.isAllGood(voltage, statorCurrent, velocityRPS);
    inputs.appliedVolts = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.velocityRPS = velocityRPS.getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps) {
    outtake.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void stop() {
    outtake.stopMotor();
  }
}
