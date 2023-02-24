package frc.wpilibj9029;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;

import java.lang.Math;

/**
 * Class to use MPU6050 to get rotation and angle.
 */
public class oldMPU6050 {
  private ByteBuffer m_readBuffer = ByteBuffer.allocate(14);
  private MPUData m_configured_state = new MPUData();
  private byte m_I2CAddress;
  private I2C m_i2c;

  private static class MPUData {
    public short m_accX, m_accY, m_accZ, m_gyroX, m_gyroY, m_gyroZ;
    private short m_temperature;

    public MPUData() {
      m_accX = 0;
      m_accY = 0;
      m_accZ = 0;
      m_temperature = 0;
      m_gyroX = 0;
      m_gyroY = 0;
      m_gyroZ = 0;
    }

    public MPUData(ByteBuffer MPUReading) {
      m_accX = MPUReading.getShort(0);
      m_accY = MPUReading.getShort(2);
      m_accZ = MPUReading.getShort(4);
      m_temperature = MPUReading.getShort(6);
      m_gyroX = MPUReading.getShort(8);
      m_gyroY = MPUReading.getShort(10);
      m_gyroZ = MPUReading.getShort(12);
    }

    public void subtract(MPUData other) {
      m_accX -= other.m_accX;
      m_accY -= other.m_accY;
      m_accZ -= other.m_accZ;
      // temprature için offset yok
      // m_temperature -= other.m_temperature;
      m_gyroX -= other.m_gyroX;
      m_gyroY -= other.m_gyroY;
      m_gyroZ -= other.m_gyroZ;
    }

    public double getTemperature() {
      // Sıcaklığı hesaplamak için biraz işlem
      return m_temperature/340. + 36.53;
    }
  }

  // 0x68
  public oldMPU6050(byte I2CAddress) {
    m_I2CAddress = I2CAddress;
    m_i2c = new I2C(I2C.Port.kMXP, m_I2CAddress);
    m_i2c.write(0x6B, 0);
  }

  public void reset() {
    m_configured_state = getMPUData();
  }

  public double _map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  /**
   * @return The address of the device in byte for I2C
   */
  public byte getI2CAddress() {
    return m_I2CAddress;
  }

  private MPUData getMPUData() {
    m_i2c.read(0x3B, 14, m_readBuffer);
    MPUData read_data = new MPUData(m_readBuffer);
    read_data.subtract(m_configured_state);
    return read_data;
  }

  public double[] getAllAngles() {
    MPUData data = getMPUData();
    double ax = _map(data.m_accX, 265, 402, -90, 90);
    double ay = _map(data.m_accY, 265, 402, -90, 90);
    double az = _map(data.m_accZ, 265, 402, -90, 90);

    ax = Math.atan2(-ay, -az) + Math.PI;
    ay = Math.atan2(-ax, -az) + Math.PI;
    az = Math.atan2(-ay, -ax) + Math.PI;
  
    return new double[] { ax, ay, az };
  }

  public double getAngleX() {
    MPUData data = getMPUData();
    double ay = _map(data.m_accY, 265, 402, -90, 90);
    double az = _map(data.m_accZ, 265, 402, -90, 90);

    return Math.atan2(-ay, -az) + Math.PI;
  }

  public double getAngleY() {
    MPUData data = getMPUData();
    double ax = _map(data.m_accX, 265, 402, -90, 90);
    double az = _map(data.m_accZ, 265, 402, -90, 90);

    return Math.atan2(-ax, -az) + Math.PI;
  }

  public double getAngleZ() {
    MPUData data = getMPUData();
    double ax = _map(data.m_accX, 265, 402, -90, 90);
    double ay = _map(data.m_accY, 265, 402, -90, 90);

    return Math.atan2(-ay, -ax) + Math.PI;
  }
  
  /**
   * Get rotation from MPU6050's X axis as wpilib's Rotation2d
   */
  public Rotation2d getRotationX() {
    MPUData data = getMPUData();
    double ay = _map(data.m_accY, 265, 402, -90, 90);
    double az = _map(data.m_accZ, 265, 402, -90, 90);

    return new Rotation2d(Math.atan2(-ay, -az) + Math.PI);
  }

  /**
   * Get rotation from MPU6050's Y axis as wpilib's Rotation2d
   */
  public Rotation2d getRotationY() {
    MPUData data = getMPUData();
    double ax = _map(data.m_accX, 265, 402, -90, 90);
    double az = _map(data.m_accZ, 265, 402, -90, 90);

    return new Rotation2d(Math.atan2(-ax, -az) + Math.PI);
  }

  /**
   * Get rotation from MPU6050's Z axis as wpilib's Rotation2d
   */
  public Rotation2d getRotationZ() {
    MPUData data = getMPUData();
    double ax = _map(data.m_accX, 265, 402, -90, 90);
    double ay = _map(data.m_accY, 265, 402, -90, 90);

    return new Rotation2d(Math.atan2(-ay, -ax) + Math.PI);
  }

  public void printAngles() {
    System.out.print(Math.toDegrees(getAngleX()));
    System.out.print(", \t");
    System.out.print(Math.toDegrees(getAngleY()));
    System.out.print(", \t");
    System.out.print(Math.toDegrees(getAngleX()));
    System.out.println();
  }

  public void printAllData() {
    MPUData data = getMPUData();
    System.out.print("Acc: (");
    System.out.print(data.m_accX/16384.);
    System.out.print(", ");
    System.out.print(data.m_accY/16384.);
    System.out.print(", ");
    System.out.print(data.m_accZ/16384.);
    System.out.print(");  ");
    
    System.out.print(data.getTemperature());
    System.out.print("  ");

    System.out.print("Gyro: (");
    System.out.print(data.m_gyroX/131.);
    System.out.print(", ");
    System.out.print(data.m_gyroY/131.);
    System.out.print(", ");
    System.out.print(data.m_gyroZ/131.);
    System.out.println("); ");
  }

  public void printAccData() {
    MPUData data = getMPUData();
    System.out.print("Acc X: ");
    System.out.print(data.m_accX);
    System.out.print(" Acc Y: ");
    System.out.print(data.m_accY);
    System.out.print(" Acc Z: ");
    System.out.println(data.m_accZ);
  }

  public void printTemperature() {
    MPUData data = getMPUData();
    System.out.println(data.getTemperature());
  }

  /**
   * Prints all data from MPU6050 as a gyroscope.
   * Used for debug.
   */
  public void printGyroData() {
    MPUData data = getMPUData();
    System.out.print("Gyro X: ");
    System.out.print(data.m_gyroX);
    System.out.print(" Gyro Y: ");
    System.out.print(data.m_gyroY);
    System.out.print(" Gyro Z: ");
    System.out.println(data.m_gyroZ);
  }
}