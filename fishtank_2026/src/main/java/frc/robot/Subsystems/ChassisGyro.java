package frc.robot.Subsystems;
import com.studica.frc.AHRS;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.SerialPort;

public class ChassisGyro extends com.studica.frc.AHRS {

    // public ChassisGyro() { super(); }
    public ChassisGyro(NavXComType comType) { super(comType); }
    public ChassisGyro(NavXComType comType, NavXUpdateRate updateRate) { super(comType, updateRate); }
    public ChassisGyro(NavXComType comType, int customRateHz) { super(comType, customRateHz); }

}
