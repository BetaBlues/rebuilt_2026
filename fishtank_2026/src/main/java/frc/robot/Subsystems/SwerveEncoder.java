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
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

//(device_id: int, canbus: phoenix6.canbus.CANBus | str = CANBus())

public class SwerveEncoder {

    private double fullRange;
    private double expectedZero;
    private CoreCANcoder m_CanCoder;
    private DutyCycleEncoder m_DutyCycleEncoder;
    private RelativeEncoder m_relEncoder;
    private double encoderOffset;

    public SwerveEncoder(int port, double offset) {
        this.encoderOffset = offset;
        if (Constants.hasCanCoder)
        {
            m_CanCoder = new CoreCANcoder(port);
            // super(port); 
        }
        else
        {
            m_DutyCycleEncoder = new DutyCycleEncoder(port);
        }
        this.fullRange = 1.0;
        this.expectedZero = 0.0;
    }

    public SwerveEncoder(int port, double fullRange, double expectedZero, double offset) {
        //m_CanCoder = new CoreCANcoder(port, CANBus.roboRIO());
        if (Constants.hasCanCoder)
        {
            m_CanCoder = new CoreCANcoder(port, CANBus.roboRIO());
            // super(port); 
        }
        else
        {
            m_DutyCycleEncoder = new DutyCycleEncoder(port);
          
        }
        // super(port, CANBus.roboRIO());
        this.fullRange = fullRange;
        this.expectedZero = expectedZero;
        this.encoderOffset = offset;
       
    }

    public double get() {
        // StatusSignal<Angle> signal = super.getAbsolutePosition();
        // return signal.getValue().in(Radians);
        double rawValue = 0;
        if(Constants.hasCanCoder)
        {
            rawValue = (m_CanCoder.getPosition().getValue().in(Units.Radians)) % Math.PI;
            //rawValue *= 2 * Math.PI; // Convert from radians to a 0-1 range
            //SmartDashboard.putNumber("encoder value", super.getPosition().getValue().in(Units.Radians));
        }
        else
        {
            rawValue = m_DutyCycleEncoder.get() * 2 * Math.PI;//((m_DutyCycleEncoder.get()) % 2 * Math.PI); // * 2 * Math.PI);
            
            rawValue -= encoderOffset;
            rawValue = rawValue % (Math.PI);
        }
        return rawValue;
        
    }
    public double getRawDutyCycle() {
        return m_DutyCycleEncoder.get();
    }
   

public class DutyCycleEncoderVelocity {
    private final DutyCycleEncoder encoder;
    private double lastPosition;
    private double lastTimestamp;
    private double velocity; // rotations per second

    public DutyCycleEncoderVelocity(int port) {
        m_DutyCycleEncoder.setDistancePerRotation(1.0); // 1 rotation = 1 unit distance
        lastPosition = m_DutyCycleEncoder.get();
        lastTimestamp = Timer.getFPGATimestamp();
        velocity = 0.0;
    }

    /** Call this periodically (e.g., in robotPeriodic) */
    public void update() {
        double currentPosition = m_DutyCycleEncoder.get();
        double currentTime = Timer.getFPGATimestamp();

        double deltaPos = currentPosition - lastPosition;
        double deltaTime = currentTime - lastTimestamp;

        if (deltaTime > 0) {
            velocity = deltaPos / deltaTime; // rotations per second
        }

        lastPosition = currentPosition;
        lastTimestamp = currentTime;
    }

    /** Get velocity in rotations per second */
    public double getVelocity() {
        if (Constants.hasCanCoder)
        {
            return m_CanCoder.getVelocity().getValue().in(Units.MetersPerSecond);
        }
        return velocity;
    }

    /** Get velocity in meters per second (if you know wheel circumference) */
    public double getVelocityMetersPerSecond(double wheelCircumferenceMeters) {
        return velocity * wheelCircumferenceMeters;
    }
}

   
    // public static final double leftFrontAbsOffset = 1.47;
    // public static final double rightFrontAbsOffset = 2.19;
    // public static final double leftRearAbsOffset = -1.57;
    // public static final double rightRearAbsOffset= 2.145;


    

    
}
