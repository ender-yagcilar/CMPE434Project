from pynput import keyboard
import time

class KeyboardControl:
    def __init__(self):
        self.forward_backward = 0
        self.right_left = 0
        self.keys_pressed = set()

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.daemon = True
        listener.start()

    def on_press(self, key):
        self.keys_pressed.add(key)

    def on_release(self, key):
        if key in self.keys_pressed:
            self.keys_pressed.remove(key)

    def update(self,CarPosition,CarOrientation):
        try:
            # Reset outputs
            self.forward_backward = 0
            self.right_left = 0

            if keyboard.Key.up in self.keys_pressed:
                self.forward_backward = 5
            elif keyboard.Key.down in self.keys_pressed:
                self.forward_backward = -5

            if keyboard.Key.right in self.keys_pressed:
                self.right_left = -5
            elif keyboard.Key.left in self.keys_pressed:
                self.right_left = 5
     
        except KeyboardInterrupt:
            print("Exiting...")
        return self.forward_backward,self.right_left




