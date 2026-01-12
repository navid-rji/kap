#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty, Float32
from sensor_msgs.msg import Image

HELP_TEXT = """
Interactive Clarius Test Client

Commands:
  f           : toggle freeze
  r           : request latest image (publish /clarius/request_image)
  F <MHz>     : set tx frequency, e.g. "F 5" (MHz)
  Z <cm>      : set tx focus, e.g. "Z 3" (cm)
  h           : show this help
  q           : quit

Images arriving on /clarius/bmode will be logged.
"""

class ClariusInteractiveClient:
    def __init__(self):
        rospy.init_node("clarius_interactive_client")

        self.pub_freeze = rospy.Publisher("/clarius/toggle_freeze", Empty, queue_size=1)
        self.pub_request = rospy.Publisher("/clarius/request_image", Empty, queue_size=1)
        self.pub_tx_freq = rospy.Publisher("/clarius/tx_freq", Float32, queue_size=1)
        self.pub_tx_focus = rospy.Publisher("/clarius/tx_focus", Float32, queue_size=1)

        rospy.Subscriber("/clarius/bmode", Image, self.on_image)

        rospy.loginfo("Interactive client initialized")
        print(HELP_TEXT)

        # give pubs some time to connect
        rospy.sleep(1.0)

    def on_image(self, msg: Image):
        # simple feedback that images are being received
        try:
            t = msg.header.stamp.to_sec()
        except Exception:
            t = 0.0
        rospy.loginfo(
            f"Image received: {msg.width}x{msg.height}, encoding={msg.encoding}, t={t:.3f}"
        )

    def run(self):
        while not rospy.is_shutdown():
            try:
                line = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExiting.")
                return
            
            if not line:
                continue
            
            cmd, *rest = line.split()

            if cmd == "q":
                print("Quitting.")
                break
            
            elif cmd == "h":
                print(HELP_TEXT)

            elif cmd == "f":
                rospy.loginfo("Toggling freeze.")
                self.pub_freeze.publish(Empty())
            
            elif cmd == "r":
                rospy.loginfo("Requesting latest image.")
                self.pub_request.publish(Empty())
            
            elif cmd == "F":
                if not rest:
                    print("Usage: F <MHz>, e.g. 'F 5'")
                    continue
                try:
                    val = float(rest[0])
                except ValueError:
                    print(f"Invalid frequency: {rest[0]!r}")
                    continue
                rospy.loginfo(f"Setting txFreq to {val:.2f} MHz")
                self.pub_tx_freq.publish(Float32(val))
            
            elif cmd == "Z":
                if not rest:
                    print("Usage: Z <cm>, e.g. 'Z 3'")
                    continue
                try:
                    val = float(rest[0])
                except ValueError:
                    print(f"Invalid focus: {rest[0]!r}")
                    continue
                rospy.loginfo(f"Setting txFocus to {val:.2f} cm")
                self.pub_tx_focus.publish(Float32(val))

            else:
                print(f"Unknown command: {cmd}")
                print("Type 'h' for help.")
            
if __name__ == "__main__":
    client = ClariusInteractiveClient()
    client.run()