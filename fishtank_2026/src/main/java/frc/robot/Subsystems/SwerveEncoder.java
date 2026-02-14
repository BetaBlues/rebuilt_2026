// package frc.robot.Subsystems;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;

// public class SwerveEncoder extends DutyCycleEncoder{
    

//     public SwerveEncoder(int port) {
//         super(port);
        
//     }

//     public SwerveEncoder(int port, double fullRange, double expectedZero) {
//         super(port, expectedZero, fullRange);
       
//     }
//     public double get() {
//         return super.get();
//     }

    

    
// }

package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.Units;

//(device_id: int, canbus: phoenix6.canbus.CANBus | str = CANBus())

public class SwerveEncoder extends CoreCANcoder {

    private double fullRange;
    private double expectedZero;

    public SwerveEncoder(int port) {
        super(port);
        this.fullRange = 1.0;
        this.expectedZero = 0.0;
    }

    public SwerveEncoder(int port, double fullRange, double expectedZero) {
        super(port, CANBus.roboRIO());
        this.fullRange = fullRange;
        this.expectedZero = expectedZero;
       
    }
    public double get() {
        // StatusSignal<Angle> signal = super.getAbsolutePosition();
        // return signal.getValue().in(Radians);
        double rawValue = (super.getPosition().getValue().in(Units.Radians)) % Math.PI;
        //rawValue *= 2 * Math.PI; // Convert from radians to a 0-1 range
        //SmartDashboard.putNumber("encoder value", super.getPosition().getValue().in(Units.Radians));
        return rawValue;
    }

    

    
}
