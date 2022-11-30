#! /usr/bin/env python3
import functools
from importlib.util import module_from_spec, spec_from_loader
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range

from interfaces.srv import CustomTrigger, Record

from typing import List
from typing import Any
from dataclasses import dataclass


@dataclass
class Topic:
    """Topic whose data should be recorded for a new trigger"""
    frequency: float
    max_frequency: bool
    name: str
    type: str

    @staticmethod
    def from_dict(obj: Any) -> 'Topic':
        _frequency = float(obj.get("frequency"))
        _max_frequency = bool(obj.get("max_frequency"))
        _name = str(obj.get("name"))
        _type = str(obj.get("type"))
        return Topic(_frequency, _max_frequency, _name, _type)

@dataclass
class KeyValue:
    comparator: str # one of GREATER_THAN, LESS_THAN, EQUAL
    field: str # dot separated field name
    value: str
    value_type: str

    @staticmethod
    def from_dict(obj: Any) -> 'KeyValue':
        _comparator = str(obj.get("comparator"))
        _field = str(obj.get("field"))
        _value = str(obj.get("value"))
        _value_type = str(obj.get("value_type"))
        return KeyValue(_comparator, _field, _value, _value_type)


@dataclass
class Comparison:
    key_values: List[KeyValue]
    levels: List[int]
    name: str

    @staticmethod
    def from_dict(obj: Any) -> 'Comparison':
        _key_values = [KeyValue.from_dict(y) for y in obj.get("key_values")]
        _levels = obj.get("levels")
        _name = str(obj.get("name"))
        return Comparison(_key_values, _levels, _name)

@dataclass
class Trigger:
    base_path: str # where the triggered bag is stored
    comparison: KeyValue
    duration: int #number of seconds for recording after the trigger
    enabled: bool # Whether this trigger is enabled
    id: int # ID for this particular trigger
    topic: str # topic name to inspect
    type: str # type of the topic
    msgdef: str
    topics: List[Topic] # topics to log

    @staticmethod
    def from_dict(obj: Any) -> 'Trigger':
        _base_path = str(obj.get("base_path"))
        _comparison = KeyValue.from_dict(obj.get("comparison"))
        _duration = int(obj.get("duration"))
        _enabled = bool(obj.get("enabled"))
        _id = int(obj.get("id"))
        _topic = str(obj.get("topic"))
        _type = str(obj.get("type"))
        _msgdef = str(obj.get("msgdef"))
        _topics = [Topic.from_dict(y) for y in obj.get("topics")]

        return Trigger(_base_path, _comparison, _duration, _enabled, _id, _topic, _type, _msgdef, _topics)

@dataclass
class Triggers:
    triggers: List[Trigger]
    @staticmethod
    def from_dict(obj: Any) -> 'Triggers':
        return Triggers(obj.get("triggers"))



class WoedenTriggerWorker(Node):
    def __init__(self):
        super().__init__("woeden_trigger_worker")
        self.srv = self.create_service(CustomTrigger, "/custom_trigger", self.triggers_callback)
        self.registered_types = set()
        self.trigger_subs = []
        self.callbacks = dict()
        self.get_logger().info("trigger worker node running")



    def triggers_callback(self, request, response):
        trigger_dict = json.loads(request.data)
        trigger = Trigger.from_dict(trigger_dict)

        if trigger.enabled:
            if trigger.type not in self.registered_types:
                msg_cls = self.load_class(trigger.id, trigger.msgdef)
                self.add_subscription(trigger.topic, trigger.id, msg_cls)
        response.success = True
        return response


    def load_class(self, trigger_id, msgdef):
        custom_module_name = 'woeden.usertypes'
        spec = spec_from_loader(custom_module_name, loader=None)
        assert spec
        module = module_from_spec(spec)
        sys.modules[custom_module_name] = module
        exec(msgdef, module.__dict__)  # pylint: disable=exec-used
        return getattr(module, f"Trigger{trigger_id}")
    
    def topic_callback(self, msg):
        print('kek')

    def add_subscription(self, topic, trigger_id, msg_cls):
        # def callback_template(cls, msg):
        #     del cls
        #     print(f"received message {msg.__class__}")

        # callback = functools.partial(callback_template, self

        self.trigger_subs.append(self.create_subscription(msg_cls, "/range", self.topic_callback, 10, raw=True))






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
