package org.example.mission;

public record Vec3(double x, double y, double z) {
  public static final Vec3 ZERO = new Vec3(0, 0, 0);

  public Vec3 {
    if (!Double.isFinite(x + y + z)) throw new IllegalArgumentException("Nonfinite position");
  }

  public Vec3 add(Vec3 b) {
    return new Vec3(x + b.x, y + b.y, z + b.z);
  }

  public Vec3 sub(Vec3 b) {
    return new Vec3(x - b.x, y - b.y, z - b.z);
  }

  public Vec3 mul(double k) {
    return new Vec3(x * k, y * k, z * k);
  }

  public double dot(Vec3 b) {
    return x * b.x + y * b.y + z * b.z;
  }

  public double norm() {
    return Math.sqrt(dot(this));
  }

  public double distance(Vec3 b) {
    return sub(b).norm();
  }

  public double axis(int a) {
    return a == 0 ? x : a == 1 ? y : z;
  }

  public double[] array() {
    return new double[] {x, y, z};
  }
}
