Mode cv_mode = new Mode() {
  Capture webcam;
  ArrayList<PVector> targets = new ArrayList<PVector>();
  //PGraphics camcanvas;
  PostProcessor postFX;
  PGraphics cvcanvas;
  PVector redFilterMin = new PVector(0.9, 0.7, 0.6);
  PVector redFilterMax = new PVector(0.1, 1, 1);
  PVector whiteFilterMin = new PVector(0, 0, 0.95);
  PVector whiteFilterMax = new PVector(1, 1, 1);
  PVector postFXSize;
  void setup() {
    
  }
  void onStart() {
    String[] cams = Capture.list();
    println("Cameras:");
    printArray(cams);
    if(cams.length > 0) {
      webcam = new Capture(APP, cams[0]);
    }
    println("Do we have a camera? " + (webcam != null ? "Yes!" : "No!"));
    if(webcam == null) switchMode(new Mode() {void onStart() { switchMode(cv_mode); }});
    webcam.start();
    postFX = new PostProcessor(webcam.width, webcam.height);
    cvcanvas = createGraphics(webcam.width, webcam.height, JAVA2D);
    postFXSize = new PVector(postFX.w, postFX.h);
    //camcanvas = createGraphics(webcam.width, webcam.height, P2D);
    //camcanvas.beginDraw();
    //camcanvas.background(0);
    //camcanvas.endDraw();
  }
  void onEnd() {
    if(webcam == null) return;
    webcam.stop();
    webcam = null;
  }
  void update() {
    if(webcam == null) return;
    if(webcam.available()) {
      webcam.read();
    }
    if(postFX.getCurrent().width != webcam.width) println("Bruh: " + webcam.width + ", " + postFX.getCurrent().width);
    // look for green dots in the image to set the camera space coordinate system
    
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
    //postFX.applyShader(thresholdShader);
    //avgShader.set("dir", new PVector(32, 0));
    //postFX.applyShader(avgShader);
    //avgShader.set("dir", new PVector(0, 32));
    //postFX.applyShader(avgShader);
    //localMaxShader.set("texSize", postFXSize);
    //localMaxShader.set("radius", 4.0);
    //postFX.applyShader(localMaxShader);
    cvcanvas.beginDraw();
    postFX.getCurrent().loadPixels();
    cvcanvas.background(postFX.getCurrent());
    cvcanvas.endDraw();
    // look for white pixels
    PVector[] newTargets = detectBlobs(cvcanvas, true);
    targets.clear();
    for(PVector p : newTargets) {
      targets.add(p);
    }
  }
  void draw() {
    
  }
  void drawUI() {
    if(webcam == null) return;
    image(webcam, 0, 0);
    image(cvcanvas, postFXSize.x, 0);
    image(postFX.getCurrent(), 0, postFXSize.y);
    stroke(128);
    noFill();
    for(PVector p : targets) {
      circle(p.x, p.y, 10);
    }
  }
  void mouseEvent(MouseEvent e) {
  }
  void keyPressed() {
  }
  void keyReleased() {
  }
  void keyTyped() {
  }
  void serialEvent(Serial s) {
  }
};
