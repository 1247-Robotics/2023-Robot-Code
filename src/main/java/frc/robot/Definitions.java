package frc.robot;

public class Definitions {

    public static int m_leftMaster = 1;
    public static int m_leftSlave = 2;
    public static int m_rightMaster = 3;
    public static int m_rightSlave = 4;
    //public static int spinny = 10;
    public static int uppy = 9;

    public static int c_joystick = 0;
    public static int armJoystick = 1;
    public static int c_ps4 = 1;

    public static double c_joystick_deadzone = 0.1;

    //ps4 Controller

    public static int elbowLimit = 4;
    public static int elevLimitB = 0;
    public static int elevLimitT = 9;
    

    public static int clawServo1 = 9;
    public static int clawServo2 = 7;
    public static int armPivot = 6;
    public static int clawPivot = 10;

    public static int servo1Close = 175;
    public static int servo1Open = 90;

    public static int servo2Close = 40;
    public static int servo2Open = 90;
}

// public class SensorConstants {
//     public static final int HMC5883L_ADDRESS        = 0x1E; // idfk what this is for
//     public static final int HMC5883L_CONFIG_REG_A   = 0x00;
//     public static final int HMC5883L_CONFIG_REG_B   = 0x01;
//     public static final int HMC5883L_MODE_REG       = 0x02;
//     public static final int HMC5883L_MEASURE_CONT   = 0x00;
//     public static final int HMC5883L_MEASURE_SINGLE = 0x01;
//     public static final int HMC5883L_MEASURE_IDLE   = 0x03;
//     public static final int HMC5883L_DATA           = 0x03;
//     public static final int HMC5883L_M_TO_READ      = 6;    // 2 bytes for each axis x, y, z
//     public static final int BMP085_ADDRESS          = 0x77; //address of the Barometer
    
//     public static final int BMA180_ADDRESS   = 0x40; //address of the accelerometer
//     public static final int BMA180_RESET     = 0x10;
//     public static final int BMA180_PWR       = 0x0D;
//     public static final int BMA180_BW        = 0x20;
//     public static final int BMA180_RANGE     = 0x35;
//     public static final int BMA180_DATA      = 0x02;
//     public static final int BMA180_A_TO_READ = 6;    // 2 bytes for each axis x, y, z
    
//     public static final int ITG3205_ADDRESS    = 0x68; //address of the Gyroscope
//     public static final int ITG3205_SMPLRT_DIV = 0x15;
//     public static final int ITG3205_DLPF_FS    = 0x16;
//     public static final int ITG3205_INT_CFG    = 0x17;
//     public static final int ITG3205_PWR_MGM    = 0x3E;
//     public static final int ITG3205_DATA       = 0x1B;
//     public static final int ITG3205_G_TO_READ  = 8;    // 2 bytes for each axis x, y, z, temp    
//   }