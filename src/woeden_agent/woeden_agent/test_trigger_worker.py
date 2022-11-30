import json

from rclpy.node import Node
from std_msgs.msg import String

from trigger_worker import WoedenTriggerWorker

def main():
    rclpy.init(args=args)
    worker = WoedenTriggerWorker()

    with open("example_conf.json", "r") as f:
        trigger_dict = json.load(f)
        msg = String()
        msg.data = json.dumps(trigger_dict)
        worker.triggers_callback(msg)


if __name__ == "__main__":
    main()
