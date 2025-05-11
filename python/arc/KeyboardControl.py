from pynput import keyboard
import time

class KeyboardController:
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

    def update(self):
        # Reset outputs
        self.forward_backward = 0
        self.right_left = 0

        if keyboard.Key.up in self.keys_pressed:
            self.forward_backward = 5
        elif keyboard.Key.down in self.keys_pressed:
            self.forward_backward = -5

        if keyboard.Key.right in self.keys_pressed:
            self.right_left = 3
        elif keyboard.Key.left in self.keys_pressed:
            self.right_left = -3

    def run(self):
        try:
            while True:
                self.update()
                print(f"Forward/Backward Output: {self.forward_backward}, Right/Left Output: {self.right_left}")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Exiting...")

if __name__ == "__main__":
    controller = KeyboardController()
    controller.run()
