package frc.robot.Subsystems;



public class ChassisGyro extends com.studica.frc.AHRS {

   
    public ChassisGyro(NavXComType comType) { super(comType); }
    public ChassisGyro(NavXComType comType, NavXUpdateRate updateRate) { super(comType, updateRate); }
    public ChassisGyro(NavXComType comType, int customRateHz) { super(comType, customRateHz); }


    public void resetGyro() {
        this.setAngleAdjustment((this.getAngle()));
    }


}
