Mode mode = new Mode() {
};
// Used to switch "scenes". Follows a state machine design pattern.
public abstract class Mode {
  Mode() {
    modes.add(this);
  }
  // setup needed to use PApplet methods during creation of a mode
  void setup() {
  }
  void onStart() {
  }
  void update() {
  }
  void draw() {
  }
  void drawUI() {
  }
  void mouseEvent(MouseEvent e) {
  }
  void keyPressed() {
  }
  void keyReleased() {
  }
  void keyTyped() {
  }
  void onEnd() {
  }
  void serialEvent(Serial s) {
  }
}

void switchMode(Mode m) {
  if (m == null) return;
  mode.onEnd();
  mode = m;
  m.onStart();
}
