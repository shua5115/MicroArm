PGraphics img;
ArrayList<PVector> pts = new ArrayList<PVector>();
PShape shape;

void setup() {
  size(1600, 600, P2D);
  img = createGraphics(800, 600);
}

void mousePressed() {
  pts.clear();
}

void mouseDragged() {
  PVector last = pts.get(pts.size() - 1);
  if(dist(mouseX, mouseY, last.x, last.y) > 4.0) {
    pts.add(new PVector(mouseX, mouseY));
  }
}

void mouseReleased() {
  if(pts.size() == 0) return;
  PVector avg = new PVector();
  for(PVector v : pts) {
    avg.add(v);
  }
  avg.div(pts.size());
  shape = createShape();
}

void draw() {
  background(0);
  img.beginDraw();
  img.background(0);
  img.fill(255);
  img.noStroke();
  img.circle(100, 100, 100);
  img.triangle(700, 200, 700, 400, 500, 300);
  img.triangle(300, 200, 300, 400, 500, 300);
  img.loadPixels();
  img.endDraw();
  image(img, 0, 0);
  PVector[] blobs = detectBlobs(img);
  println("Length: " + blobs.length);
  printArray(blobs);
  for(PVector p : blobs) {
    stroke(255, 0, 0);
    noFill();
    circle(p.x, p.y, 20);
  }
}

color idColor(int id) {
  if(id == 0) return color(0);
  return color(((id*150) + 0) % 256, ((id*180) + 90) % 256, ((id*230) + 180) % 256);
}

PVector[] detectBlobs(PImage img) {
  img.loadPixels();
  int len = img.pixels.length;
  int nextID = 1;
  int[] ids = new int[len];
  HashMap<Integer, Integer> equiv = new HashMap<>();
  int w = img.width;
  int h = img.height;
  ArrayList<PVector> blobs = new ArrayList<>(); // x, y is average position, z is how many blobs there are of that ID
  blobs.add(new PVector()); // a little buffer so the ids align with indexes into this array
  for (int i = 0; i < len; i++) {
    int x = i % w;
    int y = i / w;
    if (brightness(img.pixels[i]) > 128) {
      // check pixel to the left
      if (x > 0 && ids[i-1] > 0) {
        ids[i] = ids[i-1];
        blobs.get(ids[i]).add(x, y, 1);
        // if there is also a different id above,
        if (y > 0 && ids[i-w] != ids[i]) {
          equiv.putIfAbsent(min(ids[i-w], ids[i]), max(ids[i-w], ids[i]));
        }
      }
      // check pixel above
      else if (y > 0 && ids[i-w] > 0) {
        ids[i] = ids[i-w];
        blobs.get(ids[i]).add(x, y, 1);
      }
      // assign new id
      else {
        ids[i] = nextID;
        blobs.add(new PVector(x, y, 1));
        nextID += 1;
      }
    }
  }
  pushMatrix();
  translate(width/2, 0);
  for (int i = 0; i < len; i++) {
    int x = i % w;
    int y = i / w;
    int id = ids[i];
    //if(id > 0) println("ID " + id + " @ (" + x + ", " + y + ")");
    stroke(idColor(id));
    point(x, y);
  }
  popMatrix();
  // now iterate through every equivalency
  // each id with an equivalency is always mapped to an id greater than itself
  // so iterating in reverse will sum down the id stack, leaving the lower IDs with all the values
  for (int id = blobs.size() - 1; id >= 1; id -= 1) {
    Integer temp = equiv.get(id);
    if (temp != null) {
      int eq = temp; // an index greater than id
      PVector eqVec = blobs.get(eq);
      blobs.get(id).add(eqVec);
      eqVec.set(0, 0, 0); // we have transferred this information into id.
    }
  }
  ArrayList<PVector> outBlobs = new ArrayList<PVector>();
  for(PVector p : blobs) {
    if(p.z > 0.0) {
      // take the average position, then add to outBlobs
      p.x = p.x/p.z;
      p.y = p.y/p.z;
      p.z = 0.0;
      outBlobs.add(p);
    }
  }
  return outBlobs.toArray(PVector[]::new);
}
