Mode sim = new Mode() { //<>//
  //View
  float camDist, camYaw, camPitch;
  PVector camFocus = new PVector();
  PVector armTarget = new PVector(0, 100, 10);
  boolean validTarget = true;
  Program plan = new Program(GRIPPER_OFFSET);
  boolean simLooping, simStep, rewind;
  boolean simSync;
  float lastMoveTimer = fixedDeltaTime;

  void goToDirect(PVector v) {
    plan.goToDirect(v.x, v.y, v.z, 0.0, GRIPPER_OFFSET);
  }

  void goToLinear(PVector v) {
    plan.goToLinear(v, 0.0, GRIPPER_OFFSET);
  }

  void setGripper(float grip) {
    plan.setGripper(grip);
  }

  void home() {
    plan.states.add(new Msg(FK, 0, 0, 0, 0));
  }

  void simMsg(Msg msg) {
    if (msg != null) {
      if (msg.type == FK) {
        microArmFK(microArmLinks, msg.x, msg.y, msg.z, msg.w, GRIPPER_OFFSET);
        if (simSync && device != null) {
          byte[] bmsg = createFKMessage(msg.x, msg.y, msg.z, msg.w);
          device.write(bmsg);
        }
      }
      if (msg.type == GRIPPER) {
        if (simSync && device != null) {
          microArmGripper = msg.x;
          byte[] bmsg = createGripperMessage(msg.x);
          device.write(bmsg);
        }
      }
    }
  }

  void connect(String deviceName) {
    if (device == null) {
      if (serialDeviceIndex > 0 && serialDeviceIndex < serialDeviceNames.length) {
        try {
          device = new Serial(APP, deviceName, BAUD);
        }
        catch(Exception ex) {
          device = null;
          System.err.println(ex.getMessage());
        }
      }
    } else {
      device.stop();
      device = null;
      simSync = false;
    }
  }

  void estop() {
    simLooping = false;
    simStep = false;
    rewind = false;
    simSync = false;
  }

  void onStart() {
    camDist = 500;
    camPitch = -0.1*PI;
    camYaw = 0;
    camFocus.set(0, 0, 2);
    simSync = false;
    simLooping = false;
    simStep = false;
    rewind = false;
  }
  void update() {
    if (ui.focus == null) {
      hint(DISABLE_KEY_REPEAT);
      if (keys['W']) {
        camFocus.add(-cos(camYaw), sin(camYaw), 0);
      }
      if (keys['A']) {
        camFocus.add(-sin(camYaw), -cos(camYaw), 0);
      }
      if (keys['S']) {
        camFocus.add(cos(camYaw), -sin(camYaw), 0);
      }
      if (keys['D']) {
        camFocus.add(sin(camYaw), cos(camYaw), 0);
      }
      if (keys['E'] || keys[' ']) {
        camFocus.add(0, 0, 1);
      }
      if (keys['Q'] || keys[SHIFT]) {
        camFocus.add(0, 0, -1);
      }
      if (keys[LEFT]) {
        camYaw += 0.025;
      }
      if (keys[RIGHT]) {
        camYaw -= 0.025;
      }
      if (keys[UP]) {
        camPitch -= 0.025;
      }
      if (keys[DOWN]) {
        camPitch += 0.025;
      }
    } else {
      hint(ENABLE_KEY_REPEAT);
    }
    camPitch = constrain(camPitch, -HALF_PI+0.001, HALF_PI-0.001);
    camDist = constrain(camDist, 1, 1000);
    if ((mousePressed && mouseButton == RIGHT) || (ui.focus == null && keys['R'])) {
      PVector prevTarget = armTarget.copy();
      armTarget.set(camFocus);
      armTarget.z = max(armTarget.z, 0.0);
      try {
        microArmIK(armTarget.x, armTarget.y, armTarget.z, 0.0, GRIPPER_OFFSET);
        validTarget = true;
        //plan.goToDirect(armTarget.x, armTarget.y, armTarget.z, 0.0, GRIPPER_MATRIX);
      }
      catch (RuntimeException e) {
        //validTarget = false;
        armTarget.set(prevTarget);
      }
    }

    // Simulate the arm
    if ((simLooping || simStep) && plan.cursor < plan.states.size()) {
      if (lastMoveTimer >= fixedDeltaTime || simStep) {
        lastMoveTimer = 0.0;
        // poll a state from the program
        Msg msg = plan.poll();
        simMsg(msg);
      }

      lastMoveTimer += deltaTime;
      simStep = false;
    } else if (rewind) {
      if (lastMoveTimer >= 0.02) {
        lastMoveTimer = 0.0;
        plan.skip(-1);
      }
      lastMoveTimer += deltaTime;
    }
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
    // Arm target ball
    push();
    fill(validTarget ? color(0, 255, 0) : color(255, 0, 0));
    noStroke();
    translate(armTarget.x, armTarget.y, armTarget.z);
    sphere(3);
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
    push();
    hint(DISABLE_DEPTH_TEST);
    stroke(255, 255, 0, 64);
    line(camFocus.x, camFocus.y, camFocus.z-2, camFocus.x, camFocus.y, 0);
    noStroke();
    translate(camFocus.x, camFocus.y, camFocus.z);
    fill(255, 255, 0, 64);
    sphere(2);
    hint(ENABLE_DEPTH_TEST);
    pop();
  }

  int planScroll = 0;

  void drawUI() {
    ui.draw(g);
    fill(255);
    textAlign(LEFT, TOP);
    textSize(20);
    text("Joint Positions:", 0, 150);
    float[] joints = microArmJoints;
    for (int i = 0; i < joints.length; i++) {
      text("q" + (i+1) + ": " + degrees(joints[i]), 0, 175 + 25*i);
    }
    text("Target position: " + armTarget.toString(), 0, 600);
  }

  void serialEvent(Serial s) {
    //print(s.readString());
    s.clear();
  }

  void mouseEvent(MouseEvent e) {
    boolean uiHit = ui.mouseEvent(e);
    if (uiHit) return;
    if (e.getAction() == MouseEvent.DRAG && ui.focus == null) {
      float dy = (mouseY - pmouseY);
      float dx = (mouseX - pmouseX);
      if (e.getButton() == LEFT) {
        camYaw += dx * 0.005;
        camPitch -= dy * 0.005;
      }
      if (e.getButton() == CENTER) {
        float camRightX = -sin(camYaw), camRightY = -cos(camYaw);
        float camUpZ = 1;
        // pan camera
        camFocus.add(dx*0.5*camRightX, dx*0.5*camRightY, dy*0.5*camUpZ);
      }
    }
    if (e.getAction() == MouseEvent.WHEEL) {
      camDist *= pow(1.5, e.getCount());
    }
  }
  void keyPressed() {
    ui.keyPress(key);
    if (ui.focus == null) {
      if (key == ENTER) {
        simLooping = !simLooping;
        rewind = false;
      }
      if (key == TAB) {
        simStep = true;
        simLooping = false;
        rewind = false;
      }
      if (key == BACKSPACE) {
        simStep = false;
        simLooping = false;
        rewind = true;
      }
    }
  }
  void keyReleased() {
    ui.keyRelease(key);
    if (ui.focus == null) {
      if (keyCode == 'F') {
        goToLinear(armTarget);
      }
      if (keyCode == 'T') {
        goToDirect(armTarget);
      }
      if (keyCode == 'G') {
        setGripper(keys[CONTROL] ? 65536/2 : 0);
      }
      if (key == BACKSPACE) {
        rewind = false;
      }
      if (keyCode == 'H') {
        home();
      }
    }
  }
  void keyTyped() {
    ui.keyType(key);
  }
  void onEnd() {
    if (device != null) {
      device.stop();
      device = null;
    }
  }

  // UI
  ModeGUI ui = new ModeGUI();
  TextButton portEntry;
  TextButton connectButton;
  TextButton goDirectButton;
  TextButton goLinearButton;
  TextButton stepButton;
  TextButton playPauseButton;
  TextButton rewindButton;
  TextButton estopButton;
  TextEntry planViewButton;
  int planSelectIndex = 0;
  TextButton syncButton;
  TextButton gripperOnButton;
  TextButton gripperOffButton;

  void setup() {
    portEntry = new TextButton("Device") {
      {
        maxTextSize = 20;
        align.setOffset(0, 0).setAlign(LEFT, TOP).setMinSize(125, 50);
      }
      public void press(MouseEvent e) {
        serialDeviceNames = Serial.list();
      }
      public void keyPress(int key) {
        if (device != null) return;
        if (keyCode == LEFT || keyCode == UP) {
          serialDeviceIndex = max(serialDeviceIndex - 1, 0);
        }
        if (keyCode == RIGHT || keyCode == DOWN) {
          serialDeviceIndex = min(serialDeviceIndex + 1, serialDeviceNames.length - 1);
        }
        if (serialDeviceIndex > 0 && serialDeviceIndex < serialDeviceNames.length) {
          this.text = serialDeviceNames[serialDeviceIndex];
        } else {
          this.text = "No devices";
        }
        if (key == ENTER && !("No devices").equals(this.text)) {
          connect(serialDeviceNames[serialDeviceIndex]);
        }
      }
      public void draw(PGraphics c) {
        this.bgCol = ui.focus == this ? color(169) : color(200);
        super.draw(c);
        if (ui.focus == this) {
          Rect r = align.getCenterRect();
          c.pushStyle();
          c.stroke(0);
          c.line(r.x-r.w*0.45, r.y, r.x-r.w*0.4, r.y - r.h*0.4);
          c.line(r.x-r.w*0.45, r.y, r.x-r.w*0.4, r.y + r.h*0.4);
          c.line(r.x+r.w*0.45, r.y, r.x+r.w*0.4, r.y - r.h*0.4);
          c.line(r.x+r.w*0.45, r.y, r.x+r.w*0.4, r.y + r.h*0.4);
          c.popStyle();
        }
      }
    };
    ui.addField(portEntry);
    connectButton = new TextButton() {
      {
        maxTextSize = 20;
        align.setOffset(0, 50).setAlign(LEFT, TOP).setMinSize(125, 50);
      }
      public void click(MouseEvent e) {
        connect(serialDeviceNames[serialDeviceIndex]);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        this.text = device == null ? "Connect" : "Disconnect";
        super.draw(c);
      }
    };
    ui.addButton(connectButton);
    syncButton = new TextButton() {
      {
        align.setOffset(125, 0).setMinSize(125, 50).setAlign(LEFT, TOP);
        maxTextSize = 20;
      }
      public void click(MouseEvent e) {
        simSync = !simSync;
      }
      public void draw(PGraphics c) {
        if (device != null) {
          this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
          this.text = simSync ? "Syncing" : "Not Syncing";
          super.draw(c);
        }
      }
    };
    ui.addButton(syncButton);
    goDirectButton = new TextButton("Go Direct (g)") {
      {
        maxTextSize = 20;
        align.setAnchor(1.0/10, 1).setAlign(CENTER, BOTTOM).setScale(0.2, 0).setMinSize(0, 50);
      }
      public void click(MouseEvent e) {
        goToDirect(armTarget);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
      }
    };
    ui.addButton(goDirectButton);
    goLinearButton = new TextButton("Go Linear (f)") {
      {
        maxTextSize = 20;
        align.setAnchor(3.0/10, 1).setAlign(CENTER, BOTTOM).setScale(0.2, 0).setMinSize(0, 50);
      }
      public void click(MouseEvent e) {
        goToLinear(armTarget);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
      }
    };
    ui.addButton(goLinearButton);
    gripperOnButton = new TextButton("Grip") {
      {
        maxTextSize = 20;
        align.setAnchor(5.0/10, 1).setAlign(CENTER, BOTTOM).setScale(0.2, 0).setMinSize(0, 50);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
      }
      public void click(MouseEvent e) {
        plan.setGripper(0);
      }
    };
    ui.addButton(gripperOnButton);
    gripperOffButton = new TextButton("Ungrip") {
      {
        maxTextSize = 20;
        align.setAnchor(7.0/10, 1).setAlign(CENTER, BOTTOM).setScale(0.2, 0).setMinSize(0, 50);
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
      }
      public void click(MouseEvent e) {
        plan.setGripper(65536/2);
      }
    };
    ui.addButton(gripperOffButton);
    playPauseButton = new TextButton() {
      {
        align.setAnchor(0.5, 0).setAlign(CENTER, TOP).setMinSize(50, 50);
      }
      public void click(MouseEvent e) {
        // PLAY/PAUSE SIMULATION
        simLooping = !simLooping;
        rewind = false;
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
        Rect r = align.getCenterRect();
        c.pushStyle();
        c.rectMode(CENTER);
        c.fill(this.textCol);
        if (simLooping) {
          c.rect(r.x - r.w*0.2, r.y, r.w*0.25, r.h*0.9);
          c.rect(r.x + r.w*0.2, r.y, r.w*0.25, r.h*0.9);
        } else {
          c.triangle(r.x - r.w*0.1 + r.w*0.5, r.y, r.x - r.w*0.1 + r.w*cos(radians(120))*0.5, r.y + r.h*sin(radians(120))*0.5, r.x - r.w*0.1 + r.w*cos(radians(240))*0.5, r.y + r.h*sin(radians(240))*0.5);
        }
        c.popStyle();
      }
    };
    ui.addButton(playPauseButton);
    stepButton = new TextButton() {
      {
        align.setAnchor(0.5, 0).setAlign(CENTER, TOP).setMinSize(50, 50).setOffset(50, 0);
      }
      public void click(MouseEvent e) {
        simLooping = false;
        simStep = true;
        rewind = false;
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
        Rect r = align.getCenterRect();
        c.pushStyle();
        c.rectMode(CENTER);
        c.fill(this.textCol);
        c.rect(r.x - r.w*0.2, r.y, r.w*0.4, r.h*0.25);
        c.triangle(r.x + r.w*0.3, r.y, r.x + r.w*cos(radians(120))*0.3, r.y + r.h*sin(radians(120))*0.3, r.x + r.w*cos(radians(240))*0.3, r.y + r.h*sin(radians(240))*0.3);
        c.rect(r.x + r.w*0.3, r.y, r.w*0.1, r.h*0.75);
        c.popStyle();
      }
    };
    ui.addButton(stepButton);
    rewindButton = new TextButton() {
      {
        align.setAnchor(0.5, 0).setAlign(CENTER, TOP).setMinSize(50, 50).setOffset(-50, 0);
      }
      public void press(MouseEvent e) {
        simLooping = false;
        simStep = false;
        rewind = true;
      }
      public void release(MouseEvent e) {
        rewind = false;
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        super.draw(c);
        Rect r = align.getCenterRect();
        c.pushStyle();
        c.fill(this.textCol);
        c.translate(r.w*-0.15, 0);
        c.triangle(r.x - r.w*0.3, r.y, r.x + r.w*cos(radians(60))*0.3, r.y + r.h*sin(radians(60))*0.3, r.x + r.w*cos(radians(-60))*0.3, r.y + r.h*sin(radians(-60))*0.3);
        c.translate(r.w*0.4, 0);
        c.triangle(r.x - r.w*0.3, r.y, r.x + r.w*cos(radians(60))*0.3, r.y + r.h*sin(radians(60))*0.3, r.x + r.w*cos(radians(-60))*0.3, r.y + r.h*sin(radians(-60))*0.3);
        c.popStyle();
      }
    };
    ui.addButton(rewindButton);
    estopButton = new TextButton("STOP") {
      {
        minTextSize = 20;
        align.setAnchor(1, 0).setAlign(RIGHT, TOP).setMinSize(150, 50);
        bgCol = color(255, 25, 25);
        textCol = color(0);
      }
      public void press(MouseEvent e) {
        estop();
      }
      public void draw(PGraphics c) {
        super.draw(c);
        Rect r = align.getCenterRect();
        c.pushStyle();
        c.fill(0);
        c.textAlign(CENTER, CENTER);
        c.text(this.text, r.x, r.y);
        c.popStyle();
      }
    };
    ui.addButton(estopButton);
    planViewButton = new TextButton() {
      {
        align.setAnchor(1, 1).setAlign(RIGHT, BOTTOM).setScale(0, 0.8).setMinSize(225, 0);
        bgCol = color(255, 10);
        textCol = color(255);
      }
      public void click(MouseEvent e) {
        Rect r = align.getCenterRect();
        int lineHeight = 20;
        int numRows = int(r.h/lineHeight);
        planScroll = constrain(planScroll, 0, max(plan.states.size()-numRows, 0));
        planSelectIndex = constrain(int(map(e.getY(), r.y-r.h*0.5, r.y+r.h*0.5, planScroll, numRows+planScroll)), 0, max(plan.states.size() - 1, 0));
        if (e.getButton() == RIGHT) {
          plan.cursor = planSelectIndex;
          Msg msg = plan.peek();
          simMsg(msg);
        }
      }
      public void keyPress(int key) {
        Rect r = align.getCenterRect();
        float lineHeight = 20;
        int numRows = int(r.h/lineHeight);
        if (keyCode == 2) { // home
          planSelectIndex = 0;
          if (planSelectIndex < planScroll) {
            planScroll = planSelectIndex;
          }
        }
        if (keyCode == 3) { // end
          planSelectIndex = max(plan.states.size() - 1, 0);
          if (planSelectIndex >= planScroll + numRows) {
            planScroll = planSelectIndex - numRows + 1;
          }
        }
        if (keyCode == UP) {
          planSelectIndex -= keys[CONTROL] ? 10 : 1;
          if (planSelectIndex < planScroll) {
            planScroll = planSelectIndex;
          }
        }
        if (keyCode == DOWN) {
          planSelectIndex += keys[CONTROL] ? 10 : 1;
          if (planSelectIndex >= planScroll + numRows) {
            planScroll = planSelectIndex - numRows + 1;
          }
        }
        if (keyCode == ENTER) {
          plan.cursor = planSelectIndex;
          Msg msg = plan.peek();
          simMsg(msg);
          planSelectIndex += keys[SHIFT] ? -1 : 1;
          if (planSelectIndex < planScroll) {
            planScroll = planSelectIndex;
          }
          if (planSelectIndex >= planScroll + numRows) {
            planScroll = planSelectIndex - numRows + 1;
          }
        }
        planSelectIndex = constrain(planSelectIndex, 0, max(plan.states.size() - 1, 0));
        if ((key == DELETE || key == BACKSPACE) && planSelectIndex >= 0 && planSelectIndex < plan.states.size()) {
          plan.states.remove(planSelectIndex);
          if (key == BACKSPACE) planSelectIndex = max(planSelectIndex - 1, 0);
        }
      }
      public void keyRelease(int key) {
      }
      public void draw(PGraphics c) {
        this.outlineCol = ui.focus == this ? color(255) : color(169);
        super.draw(c);
        Rect r = align.getCenterRect();
        // convert to corner rect
        r.x -= 0.5*r.w;
        r.y -= 0.5*r.h;
        c.push();
        c.translate(r.x, r.y);
        c.textSize(16);
        c.textAlign(LEFT, TOP);
        float lineHeight = 20;
        int numRows = int(r.h/lineHeight);
        planScroll = constrain(planScroll, 0, max(plan.states.size()-numRows, 0));
        for (int i = planScroll; i < plan.states.size() && i < planScroll+numRows; i++) {
          Msg m = plan.states.get(i);
          if (i == plan.cursor) {
            c.fill(255, 255, 0);
          } else {
            c.fill(255);
          }
          if (i == planSelectIndex && ui.focus == this) {
            c.pushStyle();
            c.stroke(map(sin(millis()*0.005), 0, 1, 169, 255));
            c.noFill();
            c.rectMode(CORNER);
            c.rect(0, (i-planScroll)*lineHeight, r.w, lineHeight);
            c.popStyle();
          }
          c.text(i + ": " + m.toString(), 0, (i-planScroll)*lineHeight);
        }
        c.pop();
      }
    };
    ui.addField(planViewButton);
  } // setup()
}; // new Mode()
