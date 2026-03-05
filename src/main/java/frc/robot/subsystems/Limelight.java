    package frc.robot.subsystems;

    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableInstance;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

    public class Limelight {

        private final NetworkTable table;
        private static Limelight instance = null;

        // Only track these tag IDs
        private final int[] allowedIDs = {9, 10, 25};

        private Limelight() {
            table = NetworkTableInstance.getDefault().getTable("limelight");
        }

        public static Limelight getInstance() {
            if (instance == null) instance = new Limelight();
            return instance;
        }

        /** True if Limelight sees any tag and it is in allowedIDs */
        public boolean hasAllowedTarget() {
            int id = getTargetID();
            for (int allowed : allowedIDs) {
                if (id == allowed) return true;
            }
            return false;
        }

        /** Returns currently tracked tag ID or -1 */
        public int getTargetID() {
            return (int) table.getEntry("tid").getDouble(-1);
        }

        /** Horizontal offset (degrees) */
        public double getTX() {
            return table.getEntry("tx").getDouble(0.0);
        }

        /** Vertical offset (degrees) */
        public double getTY() {
            return table.getEntry("ty").getDouble(0.0);
        }

        /** Target area (0–100%) */
        public double getTA() {
            return table.getEntry("ta").getDouble(0.0);
        }

        /** Full 3D pose [x, y, z, roll, pitch, yaw] */
        public double[] getTargetPoseCameraSpace() {
            return table.getEntry("targetpose_cameraspace")
                    .getDoubleArray(new double[6]);
        }

        /** Distance forward (y-axis) to allowed target */
        public double getForwardDistance() {
            if (!hasAllowedTarget()) return 0;
            double[] pose = getTargetPoseCameraSpace();
            return pose[2]; // forward distance in meters
        }

        /** Resultant XY-plane distance to allowed target */
        public double getXYDistance() {
            if (!hasAllowedTarget()) return 0;
            double[] pose = getTargetPoseCameraSpace();
            double x = pose[0];
            double z = pose[2];
            return Math.sqrt((x * x) + (z * z));
        }
        

        /** Set LED mode (0=pipeline, 1=off, 2=blink, 3=on) */
        public void setLEDMode(int mode) {
            table.getEntry("ledMode").setNumber(mode);
        }

        /** Convenience methods to control LEDs */
        public void ledsOn() {
            setLEDMode(3);
        }

        public void ledsOff() {
            setLEDMode(1);
        }

        /** Outputs telemetry to SmartDashboard */
        public void outputTelemetry() {
            SmartDashboard.putBoolean("Limelight Has Allowed Target", hasAllowedTarget());
            SmartDashboard.putNumber("Limelight Target ID", getTargetID());
            SmartDashboard.putNumber("Limelight TX", getTX());
            SmartDashboard.putNumber("Limelight TY", getTY());
            SmartDashboard.putNumber("Limelight TA", getTA());
            SmartDashboard.putNumber("Limelight Forward Distance (m)", getForwardDistance());
            SmartDashboard.putNumber("Limelight XY Distance (m)", getXYDistance());

            double[] pose = getTargetPoseCameraSpace();

            SmartDashboard.putNumber("TargetPose X (m)", pose[0]);
            SmartDashboard.putNumber("TargetPose Y (m)", pose[1]);
            SmartDashboard.putNumber("TargetPose Z (m)", pose[2]);
            SmartDashboard.putNumber("TargetPose Roll", pose[3]);
            SmartDashboard.putNumber("TargetPose Pitch", pose[4]);
            SmartDashboard.putNumber("TargetPose Yaw", pose[5]);

            SmartDashboard.putNumber("distance to target", getXYDistance());
        }
    }