public class TextButton implements TextEntry {
  boolean enabled = true;
  public String text;
  public int textAlignX, textAlignY;
  public float minTextSize, maxTextSize, textScale;
  public int bgCol, textCol, outlineCol;
  public float outlineWeight;
  public Align align = new Align();
  public TextButton() {
    this("");
  }
  public TextButton(String text) {
    textAlignX = CENTER;
    textAlignY = CENTER;
    textScale = 0.8;
    minTextSize = 12;
    maxTextSize = Float.MAX_VALUE;
    bgCol = color(255);
    textCol = color(0);
    outlineCol = color(0);
    outlineWeight = 2;
    this.text = (text == null ? "" : text);
  }
  boolean contains(float mx, float my) {
    return align.contains(mx, my);
  }
  boolean isEnabled() {
    return enabled;
  }
  void setEnabled(boolean e) {
    enabled = e;
  }
  void draw(PGraphics c) {
    align.setBounds(c);
    c.pushStyle();
    c.rectMode(CENTER);
    Rect r = align.getCenterRect();
    c.strokeWeight(outlineWeight);
    c.stroke(outlineCol);
    c.fill(bgCol);
    r.draw(c);
    c.fill(textCol);
    c.textSize(getTextSize());
    c.textAlign(textAlignX, textAlignY);
    if (text != null)
      c.text(text, r.x, r.y, r.w, r.h);
    c.popStyle();
  }
  void press(MouseEvent e) {
  }
  void release(MouseEvent e) {
  }
  void click(MouseEvent e) {
  }
  void keyPress(int key) {
    if (key == CODED) return;
    text = enterText(text, key, true);
  }
  void keyRelease(int key) {
  }
  void keyType(int key) {
  }
  TextButton setTextAlign(int x, int y) {
    if (x == LEFT || x == CENTER || x == RIGHT)
      textAlignX = x;
    if (y == TOP || y == CENTER || y == BOTTOM)
      textAlignY = y;
    return this;
  }
  float getTextSize() {
    return constrain(align.getH() * textScale, minTextSize, maxTextSize);
  }
  String getText() {
    return text;
  }
}

public class IntButton extends TextButton {
  int value;
  boolean negatable;
  public IntButton() {
    this(0);
  }
  public IntButton(int val) {
    super();
    value = val;
  }
  @Override
    void draw(PGraphics c) {
    text = getText();
    super.draw(c);
  }
  @Override
    void keyPress(int key) {
    if (key == CODED) return;
    value = enterInteger(value, key, negatable);
  }
  @Override
    String getText() {
    return String.valueOf(value);
  }
  int getValue() {
    return value;
  }
}

public class CheckBox implements Button {
  public int bgCol, trueCol, falseCol, outlineCol;
  public float outlineWeight;
  public boolean value, enabled = true;
  public Align align = new Align();
  public CheckBox(boolean val) {
    value = val;
    bgCol = color(255);
    trueCol = color(0, 255, 0);
    falseCol = color(255, 0, 0);
    outlineCol = color(0);
    outlineWeight = 2;
  }
  public CheckBox() {
    this(false);
  }
  boolean isEnabled() { 
    return enabled;
  }
  void setEnabled(boolean e) { 
    enabled = e;
  }
  void draw(PGraphics c) {
    align.setBounds(c);
    c.pushStyle();
    c.rectMode(CENTER);
    Rect r = align.getCenterRect();
    c.strokeWeight(outlineWeight);
    c.stroke(outlineCol);
    c.fill(bgCol);
    r.draw(c);
    c.fill(value ? trueCol : falseCol);
    c.textAlign(CENTER, CENTER);
    c.textSize(r.h * 0.8);
    c.text(value ? "y" : "n", r.x, r.y);
    c.popStyle();
  }
  boolean contains(float mx, float my) {
    return align.contains(mx, my);
  }
  void press(MouseEvent e) {
  }
  void release(MouseEvent e) {
  }
  void click(MouseEvent e) {
    value = !value;
  }
}
