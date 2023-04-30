class Rect {
  public float x, y, w, h;
  public Rect() {
    this(0, 0, 0, 0);
  }
  public Rect(float x, float y, float w, float h) {
    set(x, y, w, h);
  }
  public Rect(Rect r) {
    this(r.x, r.y, r.w, r.h);
  }
  public void set(Rect r) {
    set(r.x, r.y, r.w, r.h);
  }
  public void set(float x, float y, float w, float h) {
    this.x = x; this.y = y; this.w = w; this.h = h;
  }
  public void draw(PGraphics c) {
    c.rect(x, y, w, h);
  }
  public boolean contains(float mx, float my) {
    return abs(mx - x) <= w*0.5 && abs(my - y) <= h*0.5;
  }
  public boolean cornerContains(float mx, float my) {
    return mx > x && mx <= x + w && my > y && my <= y + h;
  }
  public String toString() {
    return x + ", " + y + ", " + w + ", " + h;
  }
}
