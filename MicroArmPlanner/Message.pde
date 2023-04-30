final int NO_OP = 0;
final int FK = 1;
final int GRIPPER = 2;

public byte[] createMessage(int mode, int x, int y, int z, int w) {
  ByteBuffer bb = ByteBuffer.allocate(19).order(ByteOrder.LITTLE_ENDIAN);
  bb.putShort((short)mode).putInt(x).putInt(y).putInt(z).putInt(w).put((byte)'\n');
  return bb.array();
}

public byte[] createFKMessage(float q1, float q2, float q3, float q4) {
  int mode = FK;
  int x = int(q1*RADS_TO_TICKS) + (65536/2);
  // real limits for q2: +45 degrees, -135 degrees
  int y = Math.floorMod((int(q2*RADS_TO_TICKS) + (65536) + 65536/16), 65536);
  // real limits for q3: -45 degrees, 135 degrees
  int z = Math.floorMod(-int(q3*RADS_TO_TICKS) + 3*(65536/2) - 65536/3 + 1280, 65536);
  int w = Math.floorMod(int(q4*RADS_TO_TICKS) + (65536/4), 65536);
  println("Sending: ", mode, x, y, z, w);
  return createMessage(mode, x, y, z, w);
}

public byte[] createGripperMessage(float q5) {
  return createMessage(GRIPPER, int(q5*RADS_TO_TICKS), 0, 0, 0);
}

//public byte[] stringToMessage(String s) {
//  String[] tokens = s.toLowerCase().split("[ \n\t\r,;]+");
//  int mode = -1;
//  switch(tokens[0]) {
//    case "inv": mode = INV; break;
//    case "fw": mode = FW; break;
//    case "grip": {
//      float grip = float(tokens[1]);
//      if (grip != Float.NaN) {
//        return createMessage(GRIP, grip, 0, 0, 0);
//      }
//    } break;
//    case "home": return createMessage(HOME, 0, 0, 0, 0);
//    default:
//    try {
//      mode = Integer.parseUnsignedInt(tokens[0]);
//    } catch(NumberFormatException e) {
//      return null;
//    }
//  }
//  Float[] nums = new Float[4];
//  for(int i = 0; i < nums.length && i < tokens.length-1; i++) {
//    nums[i] = float(tokens[i+1]);
//    if (nums[i] == Float.NaN) nums[i] = null;
//  }
//  for (Float f : nums) {
//    if (f == null) return null;
//  }
//  return createMessage(mode, nums[0], nums[1], nums[2], nums[3]);
//}
