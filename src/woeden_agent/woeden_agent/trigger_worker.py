import functools
from importlib.util import module_from_spec, spec_from_loader
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from woeden_trigger import Triggers


class WoedenTriggerWorker(Node):
    def __init__(self):
        super().__init__("woeden_trigger_worker")
        self.subscription = self.create_subscription(
            String, "__woeden_triggers", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.registered_types = set()
        self.trigger_subs = []
        self.callbacks = dict()


    def triggers_callback(self, msg: String):
        self.get_logger().info('I heard: "%s"' % msg.data)
        trigger_dict = json.loads(msg.data)
        trigger = Trigger.from_dict(trigger_dict)

        if trigger.enabled:
            if trigger.type not in self.registered_types:
                self.register_message(trigger.msgdef_python)
                self.add_subscription(trigger.topic, trigger.id)

    def register_message(self, msgdef):
        custom_module_name = 'woeden.usertypes'
        spec = spec_from_loader(custom_module_name, loader=None)
        assert spec
        module = module_from_spec(spec)
        sys.modules[custom_module_name] = module
        exec(msgdef, module.__dict__)  # pylint: disable=exec-used

    def add_subscription(self, topic, trigger_id):
        msg_cls = woeden.usertypes.getattr(f"Trigger{trigger_id}")
        def callback_template(cls, msg):
            del cls
            print(f"received message {msg.__class__}")

        callback = functools.partial(callback_template, self)
        self.trigger_subs += self.create_subscription(msg_cls, topic, callback, 10)






def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = WoedenTriggerWorker()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
