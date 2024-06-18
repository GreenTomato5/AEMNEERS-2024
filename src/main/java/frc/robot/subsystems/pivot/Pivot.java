package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Subsystem base automatically runs periodic, too scared to put stuff in robot.java
public class Pivot extends SubsystemBase {
    
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
    PivotIO io;

    public Pivot(PivotIO io) {
        this.io = io;
    }
}
