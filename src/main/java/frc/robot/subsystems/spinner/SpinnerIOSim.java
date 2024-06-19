package frc.robot.subsystems.spinner;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

// At this point im just putting override everywhere where it doesent make an error, I should lowkey google what it is

public class SpinnerIOSim implements SpinnerIO{
    // Took from other repo this is based on (idk what type of sim to use, y isnt this flywheel)
    private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getNEO(2), 1, 0.01);
    
    public void updateInputs(SpinnerIOInputs inputs) {
        inputs.wheelSpeed = intakeSim.getAngularVelocityRPM();
        inputs.wheelSpeedPoint = 
    }

    public void setSpeed(double speed) {

    }

    @Override
    public void setVoltage(double volts) {
        intakeSim.setInputVoltage(volts);
    }

    @Override
    public void stop() {
        setVoltage(0);
    }

    public void configurePID(double kP, double kI, double kD) {
        
    } 

}
