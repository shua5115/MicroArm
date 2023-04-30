// A class which can apply shader effects in series to an image
class PostProcessor {
  int w, h;
  PGraphics cur, prev;
  public PostProcessor(int width, int height) {
    w = width;
    h = height;
    cur = createGraphics(w, h, P2D);
    prev = createGraphics(w, h, P2D);
    cur.beginDraw();
    cur.background(0);
    cur.endDraw();
    prev.beginDraw();
    prev.background(0);
    prev.endDraw();
  }
  
  // Returns a canvas which can be used for drawing after an effect has been applied.
  PGraphics getCurrent() {
    return this.cur;
  }
  
  void set(PImage img) {
    if(img == null || !img.isLoaded()) return;
    cur.beginDraw();
    cur.resetShader();
    cur.background(0);
    cur.imageMode(CORNER);
    cur.image(img, 0, 0);
    cur.endDraw();
  }
  
  void applyShader(PShader shader) {
    prev.beginDraw();
    prev.background(0);
    prev.shader(shader);
    prev.imageMode(CORNER);
    prev.image(cur, 0, 0);
    prev.endDraw();
    // swap prev and cur
    PGraphics temp = cur;
    cur = prev;
    prev = temp;
    prev.beginDraw();
    prev.endDraw();
  }
}
