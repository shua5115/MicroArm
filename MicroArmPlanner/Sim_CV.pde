Mode cv_sim = new Mode() {
  ModeGUI ui = new ModeGUI();

  // Computer vision variables
  Capture webcam;
  PostProcessor postFX;
  PGraphics cvcanvas;
  PVector postFXSize;
  PVector redFilterMin = new PVector(0.9, 0.69, 0.6);
  PVector redFilterMax = new PVector(0.1, 1, 1);
  PVector whiteFilterMin = new PVector(0, 0, 0.95);
  PVector whiteFilterMax = new PVector(1, 1, 1);
  //PVector blueFilterMin = new PVector(0.5, 0.5, 0.4);
  //PVector blueFilterMax = new PVector(0.75, 1, 1);
  PVector greenFilterMin = new PVector(0.2, 0.3, 0.25);
  PVector greenFilterMax = new PVector(0.4, 0.8, 1);
  ArrayList<PVector> targets = new ArrayList<PVector>();
  ArrayList<PVector> targetDeltas = new ArrayList<PVector>();
  PVector restPos = new PVector(0, 110, 0);
  PVector finalTarget;
  float finalTargetConfidence = 0;
  final float confidenceThreshold = 3.0;

  // sim variables
  PVector minDot = new PVector();
  PVector maxDot = new PVector();
  PMatrix3D camToWorldMat;
  boolean simSync;
  boolean findBounds = true;
  float lastMoveTimer = fixedDeltaTime;
  float camDist, camYaw, camPitch;
  PVector camFocus = new PVector();
  Program plan = new Program(GRIPPER_OFFSET);
  PVector[] destinations = {
    new PVector(-120.2082, 120.2082, 0),
    new PVector(-95.4594, 95.4954, 0),
    new PVector(-70.7107, 70.7107, 0),
    new PVector(-107.8338, 107.8338, 31),
    new PVector(-83.0850, 83.0850, 31),
    new PVector(-95.4594, 95.4954, 62),
  };
  int destIndex = 0;

  void goToDirect(PVector v) {
    plan.goToDirect(v.x, v.y, v.z, 0.0, GRIPPER_OFFSET);
  }

  void goToLinear(PVector v) {
    plan.goToLinear(v, 0.0, GRIPPER_OFFSET);
  }

  void setGripper(float grip) {
    plan.setGripper(grip);
  }

  void waitMS(int ms) {
    int numMsgs = ms/int(fixedDeltaTime*1000);
    for (int i = 0; i < numMsgs; i++) {
      plan.setGripper(plan.grip);
    }
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
    if (device != null) {
      plan = new Program(GRIPPER_OFFSET);
      goToLinear(restPos);
    }
  }

  void estop() {
    simSync = false;
  }

  void onStart() {
    destIndex = 0;
    camDist = 500;
    camPitch = -0.1*PI;
    camYaw = 0;
    camFocus.set(0, 0, 2);
    simSync = false;
    findBounds = true;
    camToWorldMat = null;
    String[] cams = Capture.list();
    println("Cameras:");
    printArray(cams);
    int attempts = 1;
    String camName = "DroidCam Source 3"; // cams[0];
    while (webcam == null || attempts <= 10) {
      println("(" + attempts + ") Attempting to open camera " + camName + "...");
      if (cams.length > 0) {
        webcam = new Capture(APP, camName);
      } else {
        break;
      }
      attempts += 1;
    }
    //if (webcam == null) {
    //  throw new RuntimeException("Could not open camera.");
    //}

    webcam.start();
    postFX = new PostProcessor(webcam.width, webcam.height);
    cvcanvas = createGraphics(webcam.width, webcam.height, JAVA2D);
    postFXSize = new PVector(postFX.w, postFX.h);
  }
  void onEnd() {
    if (device != null) {
      device.stop();
      device = null;
    }
    if (webcam != null) {
      webcam.stop();
      webcam = null;
    }
  }

  PVector camToWorld(PVector camPoint, PVector dest) {
    if (camToWorldMat == null || camPoint == null) return null;
    return camToWorldMat.mult(camPoint, dest);
  }

  void detectBounds() {
    if (webcam == null || !webcam.isCapturing()) return;
    // look for green dots in the image to set the camera space coordinate system
    postFX.set(webcam);
    thresholdShader.set("minHSV", greenFilterMin);
    thresholdShader.set("maxHSV", greenFilterMax);
    postFX.applyShader(thresholdShader);
    avgShader.set("size", postFXSize);
    avgShader.set("dir", new PVector(16, 0));
    postFX.applyShader(avgShader);
    avgShader.set("dir", new PVector(0, 16));
    postFX.applyShader(avgShader);
    avgShader.set("dir", new PVector(11.3, 11.3));
    postFX.applyShader(avgShader);
    avgShader.set("dir", new PVector(11.3, -11.3));
    postFX.applyShader(avgShader);
    thresholdShader.set("minHSV", whiteFilterMin);
    thresholdShader.set("maxHSV", whiteFilterMax);
    postFX.applyShader(thresholdShader);
    cvcanvas.beginDraw();
    postFX.getCurrent().loadPixels();
    cvcanvas.background(postFX.getCurrent());
    cvcanvas.endDraw();
    PVector[] dotTargets = detectBlobs(cvcanvas, false);
    if (dotTargets.length >= 3) {
      float[] x = new float[dotTargets.length];
      float[] y = new float[dotTargets.length];
      for (int i = 0; i < dotTargets.length; i++) {
        x[i] = dotTargets[i].x;
        y[i] = dotTargets[i].y;
      }
      float minX = min(x);
      float minY = min(y);
      float maxX = max(x);
      float maxY = max(y);
      float xScale = 150.0/(maxX - minX);
      float yScale = 150.0/(maxY - minY);
      if (camToWorldMat == null) {
        camToWorldMat = new PMatrix3D();
      }
      camToWorldMat.set(
        0, xScale, 0, -minY*yScale,
        yScale, 0, 0, -minX*xScale,
        0, 0, 1, 0,
        0, 0, 0, 1
        );
      minDot.set(minX, minY);
      maxDot.set(maxX, maxY);
      // (minx, miny) maps to (0, 0)
      // (maxx, maxy) maps to (150, 150)
      // the bounding box of these pixels corresponds to known coordiantes
      // the marked bounds for the robot is the 150x150 mm square in the positive x and y directions at its base.
    }
  }

  void detectTargets() {
    if (webcam == null || !webcam.isCapturing()) {
      targetDeltas.clear();
      return;
    }
    // look for red cylinders in the image to find
    postFX.set(webcam);
    thresholdShader.set("minHSV", redFilterMin);
    thresholdShader.set("maxHSV", redFilterMax);
    postFX.applyShader(thresholdShader);
    avgShader.set("size", postFXSize);
    avgShader.set("dir", new PVector(16, 0));
    postFX.applyShader(avgShader);
    avgShader.set("dir", new PVector(0, 16));
    postFX.applyShader(avgShader);
    avgShader.set("dir", new PVector(11.3, 11.3));
    postFX.applyShader(avgShader);
    avgShader.set("dir", new PVector(11.3, -11.3));
    postFX.applyShader(avgShader);
    thresholdShader.set("minHSV", whiteFilterMin);
    thresholdShader.set("maxHSV", whiteFilterMax);
    postFX.applyShader(thresholdShader);
    cvcanvas.beginDraw();
    postFX.getCurrent().loadPixels();
    cvcanvas.background(postFX.getCurrent());
    cvcanvas.endDraw();
    PVector[] newTargets = detectBlobs(cvcanvas, false);
    targetDeltas.clear();
    if (targets.size() == newTargets.length) {
      for (int i = 0; i < newTargets.length; i++) {
        targetDeltas.add(PVector.sub(newTargets[i], targets.get(i)));
      }
    }
    targets.clear();
    for (PVector p : newTargets) {
      targets.add(p); // NOTE: targets contains positions in CAMERA SPACE
    }
  }

  void transportTarget(PVector target) {
    // pick up the target and move it to the next open loading location
    PVector dest = destinations[destIndex];
    destIndex += 1;
    // Steps:
    // 0. Go above rest position
    PVector aboveRestPos = new PVector(0, 0, 40).add(restPos);
    goToLinear(aboveRestPos);
    // 1. Open gripper
    setGripper(65536/4);
    // 2. Go above the target (z>40?)
    PVector aboveTarget = new PVector(0, 0, 40).add(target);
    goToLinear(aboveTarget);
    // 3. Go to the target
    goToLinear(target);
    // 4. Close gripper
    setGripper(0);
    waitMS(250);
    // 5. Go above the target
    goToLinear(aboveTarget);
    // 6. Move above destination
    PVector aboveDest = new PVector(0, 0, 40).add(dest);
    goToLinear(aboveDest);
    // 7. Move to destination
    goToLinear(dest);
    // 8. Open gripper a little bit
    setGripper(65536/8);
    waitMS(250);
    // 9. Move above destination
    goToLinear(aboveDest);
    // 10. Move away (carefully) from the destination to a neutral position
    goToLinear(new PVector(50, 0, 0).add(aboveDest));
    goToLinear(aboveRestPos);
    goToLinear(restPos);
    setGripper(0);
  }

  void update() {
    if (webcam != null && webcam.isCapturing()) {
      if (webcam.available()) {
        webcam.read();
      }
    }
    if (findBounds) {
      detectBounds();
    }
    detectTargets();

    // Robot logic
    // if there are some potential targets that might be stationary and the robot is done moving and there is an open loading location,
    // then look for a new target to seek
    if (camToWorldMat != null && targetDeltas.size() != 0 && plan.cursor == plan.states.size() && destIndex < destinations.length) {
      // free up some RAM
      plan.states.clear();
      plan.cursor = 0;
      PMatrix3D camToWorldScale = new PMatrix3D(camToWorldMat);
      // remove translation
      camToWorldScale.m03 = 0;
      camToWorldScale.m13 = 0;
      camToWorldScale.m23 = 0;
      camToWorldScale.m33 = 1;
      // look through every potential target and see which one is moving the least.
      boolean allNotMoving = true;
      float minVel = 2; // mm per frame
      for (int i = 0; i < targetDeltas.size(); i++) {
        PVector screenVel = targetDeltas.get(i);
        float vel = camToWorldScale.mult(screenVel, null).mag();
        if (vel > minVel) {
          allNotMoving = false;
          break;
        }
      }
      if (allNotMoving) {
        // look for closest target
        PVector closest = new PVector(1000, 0);
        for (PVector p : targets) {
          PVector maybe = camToWorld(p, null);
          if (maybe.magSq() < closest.magSq()) {
            try {
              microArmIK(maybe.x, maybe.y, maybe.z, 0.0, GRIPPER_OFFSET);
            }
            catch(Exception e) {
              continue;
            }
            closest.set(maybe);
          }
        }
        if (finalTarget == null) {
          finalTarget = new PVector();
          finalTarget.set(closest);
          finalTargetConfidence = 0.0;
        }
        if (PVector.dist(closest, finalTarget) <= 5.0) {
          finalTargetConfidence += deltaTime;
        } else {
          finalTargetConfidence = 0.0;
          finalTarget = null;
        }
        //println("Found potential target: " + closest + "; Confidence: " + finalTargetConfidence);
      }
    }

    // if we are confident in a target,
    // then move it!
    if (finalTarget != null && finalTargetConfidence > confidenceThreshold) {
      transportTarget(finalTarget);
      finalTarget = null;
    }

    // Sim control
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
    //if ((mousePressed && mouseButton == RIGHT) || (ui.focus == null && keys['R'])) {
    //  PVector prevTarget = armTarget.copy();
    //  armTarget.set(camFocus);
    //  armTarget.z = max(armTarget.z, 0.0);
    //  try {
    //    microArmIK(armTarget.x, armTarget.y, armTarget.z, 0.0, GRIPPER_OFFSET);
    //    validTarget = true;
    //    //plan.goToDirect(armTarget.x, armTarget.y, armTarget.z, 0.0, GRIPPER_MATRIX);
    //  }
    //  catch (RuntimeException e) {
    //    //validTarget = false;
    //    armTarget.set(prevTarget);
    //  }
    //}

    // Simulate the arm
    if (simSync && plan.cursor < plan.states.size()) {
      if (lastMoveTimer >= fixedDeltaTime) {
        lastMoveTimer = 0.0;
        // poll a state from the program
        Msg msg = plan.poll();
        simMsg(msg);
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
    //push();
    //fill(validTarget ? color(0, 255, 0) : color(255, 0, 0));
    //noStroke();
    //translate(armTarget.x, armTarget.y, armTarget.z);
    //sphere(3);
    //pop();
    // Robot
    drawJoints(g, microArmLinks, true);
    push();
    for (PMatrix3D m : microArmLinks) {
      applyMatrix(m);
    }
    noStroke();
    fill(lerpColor(color(255, 0, 0, 64), color(0, 255, 0, 64), microArmGripper / 65535.0));
    sphere(4);
    pop();
    // CV image plane and other CV things
    if (camToWorldMat != null && webcam != null) {
      // CV image
      push();
      translate(0, 0, 0.1);
      applyMatrix(camToWorldMat);
      imageMode(CORNER);
      image(webcam, 0, 0);
      stroke(0, 128, 0);
      noFill();
      strokeWeight(10);
      translate(0, 0, 1);
      rectMode(CORNERS);
      rect(minDot.x, minDot.y, maxDot.x, maxDot.y);
      pop();
      // CV targets
      push();
      fill(255, 0, 0);
      noStroke();
      PVector worldPos = new PVector();
      for (PVector target : targets) {
        camToWorld(target, worldPos);
        pushMatrix();
        translate(worldPos.x, worldPos.y);
        sphere(5);
        popMatrix();
      }
      if (finalTarget != null) {
        fill(0, 0, 255, 255*finalTargetConfidence/confidenceThreshold);
        noStroke();
        //camToWorld(finalTarget, worldPos);
        //pushMatrix();
        translate(finalTarget.x, finalTarget.y);
        sphere(8);
        //popMatrix();
      }
      pop();
    }
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
  void drawUI() {
    ui.draw(g);
    push();
    fill(255);
    textAlign(LEFT, TOP);
    textSize(20);
    text("Joint Positions:", 0, 150);
    float[] joints = microArmJoints;
    for (int i = 0; i < joints.length; i++) {
      text("q" + (i+1) + ": " + degrees(joints[i]), 0, 175 + 25*i);
    }
    //text("Target position: " + armTarget.toString(), 0, 600);
    pop();
    if (webcam != null) {
      push();
      float view_w = 240;
      float view_h = view_w*(postFXSize.y/postFXSize.x);
      translate(width-view_w, 100);
      image(webcam, 0, 0, view_w, view_h);
      image(cvcanvas, 0, 0, view_w, view_h);
      scale(view_w/postFXSize.x);
      //if (camToWorldMat != null) {
      //  push();
      //  stroke(0, 128, 0);
      //  strokeWeight(5);
      //  noFill();
      //  rectMode(CORNERS);
      //  rect(minDot.x, minDot.y, maxDot.x, maxDot.y);
      //  pop();
      //}
      stroke(128);
      strokeWeight(10);
      noFill();
      for (PVector p : targets) {
        circle(p.x, p.y, 50);
      }
      pop();
    }
  }
  void keyPressed() {
    ui.keyPress(key);
  }
  void keyReleased() {
    ui.keyRelease(key);
    if (ui.focus == null) {
      if (keyCode == 'H') {
        home();
      }
    }
  }
  void keyTyped() {
    ui.keyType(key);
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

  // UI
  TextButton portEntry;
  TextButton connectButton;
  TextButton estopButton;
  TextButton syncButton;
  TextButton findBoundsButton;
  TextEntry planViewButton;
  int planSelectIndex = 0;
  int planScroll = 0;

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
        if (serialDeviceIndex >= 0 && serialDeviceIndex < serialDeviceNames.length) {
          connect(serialDeviceNames[serialDeviceIndex]);
        }
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        this.text = device == null ? "Connect" : "Disconnect";
        super.draw(c);
      }
    };
    ui.addButton(connectButton);
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
    syncButton = new TextButton() {
      {
        align.setOffset(125, 0).setMinSize(125, 50).setAlign(LEFT, TOP);
        maxTextSize = 20;
      }
      public void click(MouseEvent e) {
        simSync = !simSync;
      }
      public void draw(PGraphics c) {
        //if (device != null) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        this.text = simSync ? "Syncing" : "Not Syncing";
        super.draw(c);
        //}
      }
    };
    ui.addButton(syncButton);
    findBoundsButton = new TextButton() {
      {
        maxTextSize = 20;
        align.setAnchor(1.0/10, 1).setAlign(CENTER, BOTTOM).setScale(0.2, 0).setMinSize(0, 50);
      }
      public void click(MouseEvent e) {
        findBounds = !findBounds;
      }
      public void draw(PGraphics c) {
        this.bgCol = (this.align.getCenterRect().contains(mouseX, mouseY) && mousePressed) ? color(169) : color(200);
        this.text = findBounds ? "Lock Bounds" : "Look for bounds";
        super.draw(c);
      }
    };
    ui.addButton(findBoundsButton);
    planViewButton = new TextButton() {
      {
        align.setAnchor(1, 1).setAlign(RIGHT, BOTTOM).setScale(0, 0.6).setMinSize(225, 0);
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
};
