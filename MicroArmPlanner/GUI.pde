interface Button {
  boolean contains(float mx, float my);
  boolean isEnabled();
  void setEnabled(boolean e);
  void draw(PGraphics c);
  void press(MouseEvent e);
  void release(MouseEvent e);
  void click(MouseEvent e);
}

interface TextEntry extends Button {
  void keyPress(int key);
  void keyRelease(int key);
  void keyType(int key);
  String getText();
}

String enterText(String text, int keyTyped, boolean singleLine) {
  if (keyTyped == BACKSPACE || keyTyped == DELETE) {
    return text.length() > 0 ? text.substring(0, text.length() - 1) : text;
  }
  if (keyTyped == CODED || keyTyped < 32) return text;
  if (singleLine && keyTyped == '\n') return text;
  return text + (char) keyTyped;
}

// final for security reasons, this is meant to handle password input
final int enterText(char[] text, int cursor, int keyTyped, boolean singleLine) {
  if (keyTyped == BACKSPACE) {
    if (cursor > 0) text[--cursor] = 0;
  }
  if (keyTyped == CODED || keyTyped < 32) return cursor;
  if (singleLine && keyTyped == '\n') return cursor;
  if (cursor < text.length) text[cursor++] = (char) keyTyped;
  return cursor;
}

int enterInteger(int number, int keyTyped, boolean negatable) {
  if (keyTyped == BACKSPACE) {
    number /= 10;
  }
  if (negatable && keyTyped == '-') {
    number *= -1;
  }
  if (Character.isDigit(keyTyped)) {
    if (Integer.MAX_VALUE / 10 >= number) {
      number = number * 10 + Integer.parseInt(Character.toString((char) keyTyped));
    }
  }
  return number;
}
