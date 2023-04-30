PShader thresholdShader;
PShader avgShader;
PShader localMaxShader;

void loadShaders() {
  thresholdShader = loadShader("colorthresh.glsl");
  avgShader = loadShader("avg.glsl");
  localMaxShader = loadShader("localmax.glsl");
}

color idColor(int id) {
  if (id == 0) return color(0);
  return color(((id*150) + 0) % 256, ((id*180) + 90) % 256, ((id*230) + 180) % 256);
}

PVector[] detectBlobs(PImage img, boolean debug) {
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
    if(debug) {
      int id = ids[i];
      Integer temp = equiv.get(id);
      img.pixels[i] = idColor((id!=0&&temp != null) ? temp : id);
    }
  }
  if(debug) {
    img.updatePixels();
  }
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
  for (PVector p : blobs) {
    if (p.z > 0.0) {
      // take the average position, then add to outBlobs
      p.x = p.x/p.z;
      p.y = p.y/p.z;
      p.z = 0.0;
      outBlobs.add(p);
    }
  }
  return outBlobs.toArray(PVector[]::new);
}
