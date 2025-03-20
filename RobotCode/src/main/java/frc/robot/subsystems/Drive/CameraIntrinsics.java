package frc.robot.subsystems.Drive;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public class CameraIntrinsics {
  public double fx;
  public double fy;
  public double cx;
  public double cy;
  public double[] distCoeffs = new double[8];
  public int resX;
  public int resY;

  private Matrix<N3, N3> cameraMatrix;
  private Vector<N8> distCoeffsVector;

  public CameraIntrinsics(int resX, int resY, double fx, double fy, double cx, double cy, double[] distCoeffs_) {
    this.resX = resX;
    this.resY = resY;
    this.fx = fx;
    this.fy = fy;
    this.cx = cx;
    this.cy = cy;

    for(int i = 0; i < distCoeffs_.length; i++) { // cause we wanna preserve the 0s
      this.distCoeffs[i] = distCoeffs_[i];
    }

    cameraMatrix = MatBuilder.fill(
      Nat.N3(), Nat.N3(),
      fx,  0.0, cx,
      0.0, fy,  cy,
      0.0, 0.0, 1.0
    );

    distCoeffsVector = VecBuilder.fill(distCoeffs[0], distCoeffs[1], distCoeffs[2], distCoeffs[3], distCoeffs[4], distCoeffs[5], distCoeffs[6], distCoeffs[7]);
  }

  public Matrix<N3, N3> getCameraMatrix() {
    return cameraMatrix;
  }

  public Vector<N8> getDistCoeffs() {
    return distCoeffsVector;
  }
}
