class ModeGUI {
  ArrayList<Button> buttons;
  HashSet<TextEntry> fields;
  Button focus;
  TextEntry entry;
  ModeGUI() {
    buttons = new ArrayList<Button>();
    fields = new HashSet<TextEntry>();
  }
  ModeGUI addButton(Button b) {
    buttons.add(b);
    return this;
  }
  ModeGUI addField(TextEntry te) {
    buttons.add(te);
    fields.add(te);
    return this;
  }
  void draw(PGraphics c) {
    for (int i = 0; i < buttons.size(); i++) {
      Button b = buttons.get(i);
      if (!b.isEnabled()) continue;
      b.draw(c);
    }
  }
  boolean mouseEvent(MouseEvent e) {
    if (e.getAction() == MouseEvent.PRESS) {
      focus = null;
      entry = null;
      for (int i = buttons.size() - 1; i >= 0; i--) {
        Button b = buttons.get(i);
        if (!b.isEnabled()) continue;
        if (b.contains(e.getX(), e.getY())) {
          focus = b;
          if (fields.contains(focus)) entry = (TextEntry) focus;
          focus.press(e);
          break;
        }
      }
      return focus != null;
    }
    if (e.getAction() == MouseEvent.RELEASE) {
      if (focus != null && focus.isEnabled()) {
        focus.release(e);
        if (focus.contains(e.getX(), e.getY())) focus.click(e);
      }
    }
    return false;
  }
  void keyPress(int key) {
    if (entry == null || !entry.isEnabled()) return;
    if (key == 3) {
      setClipboardContents(entry.getText());
    } else if (key == 22) {
      String clip = getClipboardContents();
      if (clip != null) {
        for (int i = 0; i < clip.length(); i++) {
          entry.keyPress(clip.charAt(i));
        }
      }
    } else {
      entry.keyPress(key);
    }
  }
  void keyRelease(int key) {
    if (entry == null || !entry.isEnabled()) return;
    entry.keyRelease(key);
  }
  void keyType(int key) {
    if (entry == null || !entry.isEnabled()) return;
    entry.keyType(key);
  }
}
