final float RADS_TO_TICKS = 65536.0/PI;
final float DEGREES_TO_TICKS = 65536.0/180;
final int TICKS_PER_REV = 65535;
final int BAUD = 9600;
// Lengths in mm
final float SHOULDER_RADIUS = 90.0;
final float SHOULDER_HEIGHT = 37.6;
final float UPPER_ARM_LEN = 60;
final float FOREARM_LEN = 60;
//final float MAX_DIST = 2*sqrt(SHOULDER_HEIGHT*SHOULDER_HEIGHT + UPPER_ARM_LEN*UPPER_ARM_LEN + FOREARM_LEN*FOREARM_LEN);
final PVector GRIPPER_OFFSET = new PVector(0, 56, -19);

// Define some custom runtimeexception types to know why IK fails

class PointOutOfBoundsException extends RuntimeException {}

class NoninvertableEndEffectorException extends RuntimeException {}

class TwoLinkIKException extends RuntimeException {}

// Performs inverse kinematics calculations on a given point and end effector orientation,
// with the provided end effector matrix. The end effector matrix should transform from the wrist joint
// to the end effector's "end point". The matrix must be invertable.
// Returns four joint positions in radians.
public float[] microArmIK(float x, float y, float z, float theta, PVector endEffector) 
throws RuntimeException {
  if (y < 0 || z < 0 || (dist(0, 0, x, y) < SHOULDER_RADIUS)) { // || sqrt(x*x + y*y + z*z) > MAX_DIST) {
    throw new PointOutOfBoundsException();
  }
  float q1, q2, q3, q4;
  
  float heading = atan2(y, x)-HALF_PI;
  q1 = heading;
  
  PMatrix3D rotMat = new PMatrix3D();
  rotMat.rotateZ(heading);
  rotMat.rotateX(theta);
  PVector invEndEffector = rotMat.mult(endEffector, new PVector());
  invEndEffector.mult(-1).add(x, y, z);
  
  float wrist_x = invEndEffector.x;
  float wrist_y = invEndEffector.y;
  float wrist_z = invEndEffector.z;
  //println("Target pos: " + x + ", " + y + ", " + z);
  //println("Wrist pos: " + wrist_x + ", " + wrist_y + ", " + wrist_z);
  //println("Dist btw: " + PVector.dist(new PVector(x, y, z), new PVector(wrist_x, wrist_y, wrist_z)));
  
  float shoulder_x = 0;
  float shoulder_y = 0;
  float shoulder_z = SHOULDER_HEIGHT;
  
  float two_link_x = sqrt((wrist_x-shoulder_x)*(wrist_x-shoulder_x) 
    + (wrist_y-shoulder_y)*(wrist_y-shoulder_y));
  float two_link_y = wrist_z-shoulder_z;
  //println("Two link pos: " + two_link_x + ", " + two_link_y);
  float D = (two_link_x*two_link_x + two_link_y*two_link_y 
    - UPPER_ARM_LEN*UPPER_ARM_LEN - FOREARM_LEN*FOREARM_LEN)
    / (2.0*UPPER_ARM_LEN*FOREARM_LEN);
  if (D > 1) {
    throw new TwoLinkIKException();
  }
  float elbow = atan2(-sqrt(1-D*D), D); // make sqrt +/- for elbow up/down (not respectively, idk)
  float shoulder = atan2(two_link_y, two_link_x) - atan2(FOREARM_LEN*sin(elbow), UPPER_ARM_LEN + FOREARM_LEN*cos(elbow));
  float wrist = theta-elbow-shoulder;
  q3 = elbow;
  q2 = shoulder;
  q4 = wrist;
  return new float[] {q1, q2, q3, q4};
}

final int[] jointOffsets = {0, 0, 0, 0};
public int[] jointRadiansToServoTicks(float... joints) {
  if (joints.length != 4) return null;
  // the servos have certain limits and offsets
  int[] servos = new int[4];
  for(int i = 0; i < servos.length; i++) {
    servos[i] = int(joints[i]*RADS_TO_TICKS) + jointOffsets[i];
  }
  return servos;
}

public PMatrix3D DH(float t, float d, float r, float a) {
  return new PMatrix3D(
    cos(t), -sin(t)*cos(a), sin(t)*sin(a), r*cos(t),
    sin(t), cos(t)*cos(a), -cos(t)*sin(a), r*sin(t),
    0, sin(a), cos(a), d,
    0, 0, 0, 1
  );
}

// Returns a list of matrices that define the location of each joint relative the previous.
public PMatrix3D[] microArmFK(PMatrix3D[] src, float q1, float q2, float q3, float q4, PVector endEffector) {
  if (src == null) {
    src = new PMatrix3D[5];
  }
  if (src.length < 5) {
    PMatrix3D[] dest = new PMatrix3D[5];
    System.arraycopy(src,0,dest,0,src.length);
    src = dest;
  }
  src[0] = DH(q1, SHOULDER_HEIGHT, 0,q2-HALF_PI);
  src[1] = DH(0, UPPER_ARM_LEN, 0, q3);
  src[2] = DH(0, FOREARM_LEN, 0, q4);
  src[3] = DH(0, 0, 0, HALF_PI);
  src[4] = new PMatrix3D(
  1, 0, 0, endEffector.x,
  0, 1, 0, endEffector.y,
  0, 0, 1, endEffector.z,
  0, 0, 0, 1
  );
  return src;
}

private void drawJointShape(PGraphics c, boolean axes) {
  if (axes) {
    c.stroke(255, 0, 0);
    c.line(0, 0, 0, 5, 0, 0);
    c.stroke(0, 255, 0);
    c.line(0, 0, 0, 0, 5, 0);
    c.stroke(0, 0, 255);
    c.line(0, 0, 0, 0, 0, 5);
  } else {
    c.fill(128);
    c.noStroke();
    c.sphere(5);
  }
}

public void drawJoints(PGraphics c, PMatrix3D[] links, boolean axes) {
  c.pushStyle();
  drawJointShape(c, axes);
  for(int i = 0; i < links.length; i++) {
    c.pushMatrix();
    c.applyMatrix(links[i]);
    drawJointShape(c, axes);
  }
  for(int i = 0; i < links.length; i++) {
    c.popMatrix();
  }
  c.popStyle(); 
}
