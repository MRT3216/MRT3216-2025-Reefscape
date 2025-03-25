package frc.robot.settings;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drive.CameraIntrinsics;

public class VisionConstants {
    /**
        * Standard Deviation for trig readings for pose estimation.
        */
    public static final Matrix<N3, N1> trigStdDevs = // Trig Std Devs
            VecBuilder.fill(1, 1, Double.MAX_VALUE);

    /**
         * Standard Deviation for single tag readings for pose estimation.
         */
    public static final Matrix<N3, N1> singleTagStdDevs = // Single-Tag Std Devs
            VecBuilder.fill(4, 4, 8);
    /**
         * Standard deviation for multi-tag readings for pose estimation.
         */
    public static final Matrix<N3, N1> multiTagStdDevs = // Multi-Tag Std Devs
            VecBuilder.fill(0.5, 0.5, 1);

    /**
     * Camera Enum to select each camera
     */
    public enum CamerasConstants {
        /**
        * Center Camera
        */
        FRONT_LEFT("Front-Left",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-15),
                        Units.degreesToRadians(-15)),
                new Translation3d(
                        Units.inchesToMeters(9.162306),
                        Units.inchesToMeters(10.001698),
                        Units.inchesToMeters(9.938572)),
                new CameraIntrinsics(
                        1600, 1304,
                        1378.4945518084592, 1374.6731238827526, 776.2109098410699, 701.3215441831921,
                        new double[] { -0.030218242577181042, 0.010924281226336615, -2.9036984551012926E-4,
                                2.3696076486885392E-4, 0.019186086831858823, 0.0022537742416706484,
                                0.0012685606724970187, -9.966123793520036E-4 })),
        FRONT_RIGHT("Front-Right",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-15),
                        Units.degreesToRadians(15)),
                new Translation3d(
                        Units.inchesToMeters(9.162306),
                        Units.inchesToMeters(-10.001698),
                        Units.inchesToMeters(9.938572)),
                new CameraIntrinsics(
                        1600, 1304,
                        1374.5414918415538, 1371.4502669827698, 757.1200966271149, 650.6543370498349,
                        new double[] { -0.02599220352443215, -0.004255449108769354, -0.0010584158777595613,
                                3.151541785665642E-4, 0.035143858610829005, 0.0010261268177625253, 2.102376068279724E-4,
                                -0.0013940898120434317 })),
        BACK_LEFT("Back-Left",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-5),
                        Units.degreesToRadians(160)),
                new Translation3d(
                        Units.inchesToMeters(3),
                        Units.inchesToMeters(11.947),
                        Units.inchesToMeters(26.125)),

                new CameraIntrinsics(
                        1280, 800,
                        906.8650501556826, 904.8249218594106, 685.0939074656633, 440.3743360464017,
                        new double[] { 0.048427656762511505, -0.07279893955862766, -2.404548497778298E-4,
                                2.922420290616374E-4, 0.01566490569651964, -0.002543695016824542, 0.00631510120502211,
                                0.0019736324419207425 })),
        BACK_RIGHT("Back-Right",
                new Rotation3d(
                        Units.degreesToRadians(0),
                        Units.degreesToRadians(-5),
                        Units.degreesToRadians(200)),
                new Translation3d(
                        Units.inchesToMeters(3),
                        Units.inchesToMeters(-11.947),
                        Units.inchesToMeters(26.125)),
                new CameraIntrinsics(
                        1280, 800,
                        905.165459096268, 903.4404452296051, 640.6433787059736, 412.999885030855,
                        new double[] { 0.05018289179599694, -0.06965163455567655, -3.9595872419393856E-4,
                                1.4575235566772828E-4, 0.011609663736452296, -0.0024169860307781403,
                                0.006570847209678644, 0.0019650281377058872 }));

        public String photonName;
        public Transform3d robotToCamTransform;
        public CameraIntrinsics intrinsics;

        /**
        * Construct a Photon Camera class with help. Standard deviations are fake values, experiment and determine
        * estimation noise on an actual robot.
        *
        * @param photonName                  Name of the PhotonVision camera found in the PV UI.
        * @param robotToCamRotation    {@link Rotation3d} of the camera.
        * @param robotToCamTranslation {@link Translation3d} relative to the center of the robot.
        * @param singleTagStdDevs      Single AprilTag standard deviations of estimated poses from the camera.
        * @param multiTagStdDevsMatrix Multi AprilTag standard deviations of estimated poses from the camera.
        */
        CamerasConstants(String photonName,
                Rotation3d robotToCamRotation,
                Translation3d robotToCamTranslation,
                CameraIntrinsics intrinsics) {
            this.photonName = photonName;
            this.robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
            this.intrinsics = intrinsics;
        }
    }
}
