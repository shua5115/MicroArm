class Align {
  PGraphics canvas;
  float ax, ay;
  int alignX, alignY;
  float ox, oy;
  float sw, sh;
  float mw, mh, Mw, Mh;
  Align() {
    alignX = CENTER;
    alignY = CENTER;
    mw = 10;
    mh = 10;
    Mw = Float.MAX_VALUE;
    Mh = Float.MAX_VALUE;
  }
  Align copy(Align a) {
    canvas = a.canvas;
    ax = a.ax;
    ay = a.ay;
    alignX = a.alignX;
    alignY = a.alignY;
    ox = a.ox;
    oy = a.oy;
    sw = a.sw;
    sh = a.sh;
    mw = a.mw;
    mh = a.mh;
    Mw = a.Mw;
    Mh = a.Mh;
    return this;
  }
  Align setBounds(PGraphics c) {
    canvas = c;
    return this;
  }
  Align setAlign(int x, int y) {
    if (x == LEFT || x == CENTER || x == RIGHT)
      alignX = x;
    if (y == TOP || y == CENTER || y == BOTTOM)
      alignY = y;
    return this;
  }
  Align setAnchor(float ax, float ay) {
    this.ax = ax;
    this.ay = ay;
    return this;
  }
  Align setOffset(float ox, float oy) {
    this.ox = ox;
    this.oy = oy;
    return this;
  }
  Align setScale(float sw, float sh) {
    this.sw = sw;
    this.sh = sh;
    return this;
  }
  Align setMinSize(float w, float h) {
    this.mw = w;
    this.mh = h;
    return this;
  }
  Align setMaxSize(float w, float h) {
    this.Mw = w;
    this.Mh = h;
    return this;
  }
  float getX() {
    float w = canvas == null ? 0 : canvas.width;
    return w * ax + ox;
  }
  float getY() {
    float h = canvas == null ? 0 : canvas.height;
    return h * ay + oy;
  }
  float getW() {
    float w = canvas == null ? 0 : canvas.width;
    return constrain(w * sw, mw, Mw);
  }
  float getH() {
    float h = canvas == null ? 0 : canvas.height;
    return constrain(h * sh, mh, Mh);
  }
  Rect getCenterRect() {
    Rect r = new Rect(getX(), getY(), getW(), getH());
    switch(alignX) {
    case LEFT:
      r.x += r.w*0.5;
      break;
    case RIGHT:
      r.x -= r.w*0.5;
      break;
    }
    switch(alignY) {
    case TOP:
      r.y += r.h*0.5;
      break;
    case BOTTOM:
      r.y -= r.h*0.5;
      break;
    }
    return r;
  }
  boolean contains(float mx, float my) {
    Rect r = getCenterRect();
    return r.contains(mx, my);//abs(mx - r.x) <= r.w*0.5 && abs(my - r.y) <= r.h*0.5;
  }
}
