Mode menu = new Mode() {
  ModeGUI ui = new ModeGUI();

  float camDist, camYaw, camPitch;
  PVector camFocus = new PVector();

  TextButton softControlButton;
  TextButton hardControlButton;
  TextButton cvControlButton;
  TextButton creditLabel;

  void setup() {
    hardControlButton = new TextButton("Hardware\nControl") {
      {
        maxTextSize = 20;
        textCol = color(0);
        align.setAnchor(0.25, 0.25).setAlign(CENTER, CENTER).setScale(0.2, 0).setMinSize(200, 150);
      }
      public void click(MouseEvent e) {
        switchMode(sim_hardware);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
      }
    };
    ui.addButton(hardControlButton);
    softControlButton = new TextButton("Software\nControl") {
      {
        maxTextSize = 20;
        textCol = color(0);
        align.setAnchor(0.5, 0.25).setAlign(CENTER, CENTER).setScale(0.2, 0).setMinSize(200, 150);
      }
      public void click(MouseEvent e) {
        switchMode(sim);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
      }
    };
    ui.addButton(softControlButton);
    cvControlButton = new TextButton("CV\nDemo") {
      {
        maxTextSize = 20;
        textCol = color(0);
        align.setAnchor(0.75, 0.25).setAlign(CENTER, CENTER).setScale(0.2, 0).setMinSize(200, 150);
      }
      public void click(MouseEvent e) {
        switchMode(cv_sim);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
      }
    };
    ui.addButton(cvControlButton);
    creditLabel = new TextButton("Project by Yehoshua Halle\nFor ENME351") {
      {
        maxTextSize = 20;
        bgCol = color(255, 0);
        outlineCol = color(255, 0);
        textCol = color(255);
        setTextAlign(CENTER, TOP);
        align.setAnchor(0.5, 0.5).setAlign(CENTER, TOP).setScale(0.5, 0).setMinSize(400, 200);
      }
    };
    ui.addButton(creditLabel);
  }
  void onStart() {
    camFocus.set(0, 100, 100);
    camDist = 351;
    camPitch = -0.1*PI;
    camYaw = 0;
  }
  void update() {
    camYaw += deltaTime*0.25;
  }
  void draw() {
    perspective(PI/3.0, float(width)/height, 1, 10000);
    beginCamera();
    camera(camFocus.x + camDist * cos(camYaw) * cos(camPitch), -camFocus.z + camDist * sin(camPitch), -camFocus.y + camDist * sin(camYaw) * cos(camPitch), camFocus.x, -camFocus.z, -camFocus.y, 0, 1, 0);
    applyMatrix(1, 0, 0, 0,
      0, 0, -1, 0,
      0, -1, 0, 0,
      0, 0, 0, 1);
    // Convert to right handed coordinate system with z up.

    endCamera();
    // Ground plane
    push();
    fill(69);
    rectMode(CENTER);
    noStroke();
    rect(0, 0, 1000, 1000);
    pop();
    drawJoints(g, microArmLinks, true);
    push();
    for (PMatrix3D m : microArmLinks) {
      applyMatrix(m);
    }
    noStroke();
    fill(lerpColor(color(255, 0, 0, 64), color(0, 255, 0, 64), microArmGripper / 65535.0));
    sphere(4);
    pop();
    // Camera focus ball
    //push();
    //hint(DISABLE_DEPTH_TEST);
    //stroke(255, 255, 0, 64);
    //line(camFocus.x, camFocus.y, camFocus.z-2, camFocus.x, camFocus.y, 0);
    //noStroke();
    //translate(camFocus.x, camFocus.y, camFocus.z);
    //fill(255, 255, 0, 64);
    //sphere(2);
    //hint(ENABLE_DEPTH_TEST);
    //pop();
  }
  void drawUI() {
    ui.draw(g);
  }
  void mouseEvent(MouseEvent e) {
    boolean uiHit = ui.mouseEvent(e);
  }
  void keyPressed() {
    ui.keyPress(key);
  }
  void keyReleased() {
    ui.keyRelease(key);
  }
  void keyTyped() {
    ui.keyType(key);
  }
  void onEnd() {
  }
};
