import java.util.*;
import java.awt.Toolkit;
import java.awt.datatransfer.*;
import java.io.InputStreamReader;
import java.nio.*;
import processing.serial.*;
import processing.video.*;
import com.jogamp.newt.opengl.GLWindow;
import com.jogamp.opengl.GL;
import com.jogamp.opengl.GL2ES2;

PJOGL pgl;
GL2ES2 gl;
PApplet APP = this;
int serialDeviceIndex = -1;
String[] serialDeviceNames = new String[0];
Serial device;
PMatrix3D[] microArmLinks = new PMatrix3D[5];
float[] microArmJoints = new float[4];
float microArmGripper = 0.0;
final float fixedDeltaTime = 0.01;

float deltaTime;
long pTime;

boolean[] keys = new boolean[256];

ArrayList<Mode> modes = new ArrayList<Mode>();

void setup() {
  size(1200, 800, P3D);
  hint(ENABLE_KEY_REPEAT);
  loadShaders();
  pgl = (PJOGL) beginPGL();
  gl = pgl.gl.getGL2ES2();
  gl.glEnable(GL.GL_CULL_FACE);
  gl.glCullFace(GL.GL_BACK);
  registerMethod("mouseEvent", this);
  //surface.setLocation(0, 0);
  surface.setResizable(true);
  for(Mode m : modes) {
    m.setup();
  }
  //switchMode(sim);
  //switchMode(cv_mode);
  switchMode(menu);
  microArmFK(microArmLinks, 0, 0, 0, 0, GRIPPER_OFFSET);
}

void draw() {
  long curTime = System.nanoTime();
  deltaTime = (curTime - pTime) * 0.000000001; // convert nanosec to sec
  pTime = curTime;
  gl.glEnable(GL.GL_CULL_FACE);
  gl.glCullFace(GL.GL_FRONT);
  background(0);
  //camera();
  //perspective();
  mode.update();
  mode.draw();
  // GUI
  gl.glDisable(GL.GL_CULL_FACE);
  hint(DISABLE_DEPTH_TEST);
  // Using these settings recreates the view from P2D
  noLights();
  camera();
  ortho();
  mode.drawUI();
  hint(ENABLE_DEPTH_TEST);
}

void mouseEvent(MouseEvent e) {
  mode.mouseEvent(e);
}

void keyPressed() {
  if (keyCode < 256) keys[keyCode] = true;
  mode.keyPressed();
}

void keyReleased() {
  if (keyCode < 256) keys[keyCode] = false;
  mode.keyReleased();
}

void keyTyped() {
  mode.keyTyped();
}

void serialEvent(Serial s) {
  mode.serialEvent(s);
}

void exit() {
  if(key == ESC && mode != menu) {
    key = 0;
    switchMode(menu);
    return;
  }
  mode.onEnd();
  super.exit();
}

public String getClipboardContents() {
  String result = "";
  Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
  // odd: the Object param of getContents is not currently used
  Transferable contents = clipboard.getContents(null);
  boolean hasTransferableText = (contents != null) && contents.isDataFlavorSupported(DataFlavor.stringFlavor);
  if (hasTransferableText && contents != null) {
    try {
      result = (String) contents.getTransferData(DataFlavor.stringFlavor);
    } 
    catch (Exception ex) {
      System.out.println(ex);
      ex.printStackTrace();
    }
  }
  return result;
}

public void setClipboardContents(String str) {
  StringSelection stringSelection = new StringSelection(str);
  Clipboard clipboard = Toolkit.getDefaultToolkit().getSystemClipboard();
  clipboard.setContents(stringSelection, null);
}

public PVector model(PGraphics c, PVector p) {
  return new PVector(c.modelX(p.x, p.y, p.z), c.modelY(p.x, p.y, p.z), c.modelZ(p.x, p.y, p.z));
}
