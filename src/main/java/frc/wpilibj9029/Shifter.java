package frc.wpilibj9029;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import static frc.robot.Constants.DriveBaseConstants.*;

/**
 * Probably will not use...
 * Written just in case
 * 
 * The shifters are properties of the drivebase subsystem.
 */
public class Shifter {
    private final DoubleSolenoid m_left = new DoubleSolenoid(0, MODULE_TYPE, 0, 1);
    private final DoubleSolenoid m_right = new DoubleSolenoid(1, MODULE_TYPE, 2, 3);

    public Shifter() {}

    public void setShift() {
        
    }

    public int[] getState() {
        int leftVal;
        int rightVal;
        Value value;
        value = m_left.get();
        if (value == Value.kOff) {
            leftVal= 0;
        } else if (value == Value.kReverse) {
            leftVal = -1;
        } else if (value == Value.kForward) {
            leftVal = 1;
        } else { leftVal = 0; }
        value = m_right.get();
        if (value == Value.kOff) {
            rightVal= 0;
        } else if (value == Value.kReverse) {
            rightVal = -1;
        } else if (value == Value.kForward) {
            rightVal = 1;
        } else { rightVal = 0; }
        int[] values = {leftVal, rightVal};
        return values;
    }
}

