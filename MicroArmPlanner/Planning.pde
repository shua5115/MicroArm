final float LINEAR_SPEED_MM_PER_SEC = 50;

class Msg {
  int type;
  float x, y, z, w;
  Msg(int type, float x, float y, float z, float w) {
    this.type = type;
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }
  public String toString() {
    switch(this.type) {
      case FK:
        return String.format("FW: %3.2f, %3.2f, %3.2f, %3.2f", x, y, z, w);
      case GRIPPER:
        return String.format("GRIP: %3.2f", x);
    }
    return "Invalid";
  }
}

class Program {
  PVector current = new PVector();
  float grip = 0.0;
  float currentAngle = 0.0;
  ArrayList<Msg> states = new ArrayList<>();
  int cursor;

  public Program(PVector endEffector) {
    PMatrix3D[] ms = microArmFK(null, 0, 0, 0, 0, endEffector);
    PMatrix3D e = new PMatrix3D();
    for(PMatrix3D m : ms) {
      e.apply(m);
    }
    current.set(e.m03, e.m13, e.m23);
  }

  void goToDirect(float x, float y, float z, float angle, PVector endEffector) {
    if (current.x == x && current.y == y && current.z == z) return;
    float[] joints;
    try {
      joints = microArmIK(x, y, z, angle, endEffector);
    }
    catch(RuntimeException e) {
      System.err.println(e.toString());
      return;
    }
    current.set(x, y, z);
    states.add(new Msg(FK, joints[0], joints[1], joints[2], joints[3]));
  }

  void goToLinear(PVector target, float targetAngle, PVector endEffector) {
    if (current.equals(target)) return;
    float total_dist = PVector.dist(target, current);
    float step_dist = LINEAR_SPEED_MM_PER_SEC*20e-3;
    if (total_dist < step_dist) {
      goToDirect(target.x, target.y, target.z, targetAngle, endEffector);
      return;
    }
    PVector step = PVector.sub(target, current);
    step.setMag(step_dist); // We can only move once ever 20 ms, so this step size effectively determines the velocity.
    while (PVector.dot(PVector.sub(target, current), step) >= 0) {
      current.add(step);
      if (PVector.dot(PVector.sub(target, current), step) <= 0) {
        // If we overshoot, clamp
        current.set(target);
        break;
      }
      float a = lerp(currentAngle, targetAngle, 1.0 - PVector.dist(target, current)/total_dist);
      PVector temp = new PVector(current.x, current.y);
      if(temp.mag() <= SHOULDER_RADIUS) {
        temp.setMag(SHOULDER_RADIUS+1);
      }
      temp.z = current.z;
      float[] joints;
      try {
        joints = microArmIK(temp.x, temp.y, temp.z, a, endEffector);
      }
      catch(RuntimeException e) {
        //System.err.println(e.toString());
        continue;
      }
      states.add(new Msg(FK, joints[0], joints[1], joints[2], joints[3]));
    }
  }

  void setJoints(float q1, float q2, float q3, float q4) {
    states.add(new Msg(FK, q1, q2, q3, q4));
  }

  void setGripper(float q5) {
    grip = q5;
    states.add(new Msg(GRIPPER, q5, 0, 0, 0));
  }

  // returns a float if the gripper needs to move 
  Msg peek() {
    if (states.size() == 0) return null;
    cursor = max(min(cursor, states.size()-1), 0);
    return states.get(cursor);
  }

  Msg poll() {
    Msg m = peek();
    skip(1);
    return m;
  }

  void skip(int howMuch) {
    cursor = max(min(cursor+howMuch, states.size()), 0);
  }

  void setCursor(int cursor) {
    this.cursor = max(min(cursor, states.size()), 0);
  }
}
