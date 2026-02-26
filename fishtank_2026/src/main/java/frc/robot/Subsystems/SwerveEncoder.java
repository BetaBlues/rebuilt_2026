
package frc.robot.Subsystems;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.CANBus;
import frc.robot.Constants;
import com.revrobotics.RelativeEncoder;




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
        if (Constants.hasCanCoder)
        {
            m_CanCoder = new CoreCANcoder(port, CANBus.roboRIO());
        }
        else
        {
            m_DutyCycleEncoder = new DutyCycleEncoder(port, fullRange, offset);
          
        }
        this.fullRange = fullRange;
        this.expectedZero = expectedZero;
        this.encoderOffset = offset;
       
    }

    public double get() {
        double rawValue = 0;
        if(Constants.hasCanCoder)
        {
            rawValue = m_CanCoder.getAbsolutePosition().getValueAsDouble();
            rawValue = rawValue * 2 * Math.PI;
        }
        else
        {
            rawValue = (m_DutyCycleEncoder.get() - 0.5) * 2 * Math.PI;
            rawValue *= -1;
        }
        
        return rawValue;
        
    }
    public double getRawDutyCycle() {
        return m_DutyCycleEncoder.get();
    }
   

    
}
