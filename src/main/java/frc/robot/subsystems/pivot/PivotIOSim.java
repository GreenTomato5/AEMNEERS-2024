package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    // I guessed innacuratley, idk what half of this means fr
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getNEO(1), 
        100,                
        SingleJointedArmSim.estimateMOI(0.7, 2.0), 
        0.5,               
        0,       
        20,           
        true,                            
        0.0
    );

    private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        if (closedLoop) {
            appliedVolts =
                MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }

        sim.update(0.02);

        inputs.pivotCurrentPosition = sim.getAngleRads();
        inputs.pivotAppliedVolts = appliedVolts;
        inputs.pivotSetpoint = pid.getSetpoint();
    }

    @Override
    public void setPosition(double positionRad) {
        closedLoop = true;
        pid.setSetpoint(positionRad);
    }

    @Override
    public void setVoltage(double volts) {
        closedLoop = false;
        appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        setVoltage(0.0);
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pid.setPID(kP, kI, kD);
    }
}