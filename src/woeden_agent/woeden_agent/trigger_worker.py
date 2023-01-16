#! /usr/bin/env python3
from collections import defaultdict
from dataclasses import dataclass
import functools
from importlib.util import module_from_spec, spec_from_loader
import importlib
import logging
import json
import os
import sys
from typing import Any, List

# ROS deps
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Range

# rosbags deps
from rosbags.typesys import get_types_from_msg, register_types
from rosbags.serde import deserialize_cdr

# custom types
from interfaces.srv import CustomTrigger, Record
from interfaces.msg import WrappedBytes


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
        self.bytes_sub = self.create_subscription(WrappedBytes, "/woeden", self.bytes_callback, 10)
        self.handlers = defaultdict(dict)
        self.client = self.create_client(Record, 'record')

    def triggers_callback(self, request, response):
        trigger_dict = json.loads(request.data)
        trigger = Trigger.from_dict(trigger_dict)

        if trigger.enabled:
            msg_cls = self.load_class(trigger.id, trigger.msgdef)
            self.add_handler(trigger.topic, trigger, msg_cls)
        response.success = True
        return response


    def load_class(self, trigger_id, msgdef):
        register_types(get_types_from_msg(msgdef, f'woeden_msgs/msg/Trigger{trigger_id}'))
        module = importlib.import_module(f"rosbags.typesys.types")
        trigger_cls =  getattr(module, f"woeden_msgs__msg__Trigger{trigger_id}")
        return trigger_cls

    def add_handler(self, topic, trigger, msg_cls):
        def rgetattr(obj, attr):
            def _getattr(obj, attr):
                return getattr(obj, attr)
            return functools.reduce(_getattr, [obj] + attr.split('.'))

        def partial_handler(deserialization_func, byte_array):
            msg_instance = deserialization_func(byte_array)
            value = rgetattr(msg_instance, trigger.comparison.field)

            comparator = trigger.comparison.comparator
            trigger_val = trigger.comparison.value

            if trigger.comparison.value_type == 'NUMBER':
                value = float(value)
                trigger_val = float(trigger_val)
            
            fired = (comparator == 'EQUAL_TO' and value == trigger_val) or \
                    (comparator == 'LESS_THAN' and value < trigger_val) or \
                    (comparator == 'GREATER_THAN' and value > trigger_val)
            if fired:
                self.req = Record.Request()
                self.req.trigger_id = trigger.id
                self.client.call_async(self.req)

        serdes_func = lambda raw_data: deserialize_cdr(raw_data, msg_cls.__msgtype__)
        self.handlers[topic][trigger.id] = functools.partial(partial_handler, serdes_func)

    def bytes_callback(self, wrapped_bytes):
        handlers = self.handlers.get(wrapped_bytes.topic)
        if handlers:
            for handler in handlers.values():
                handler(wrapped_bytes.contents)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = WoedenTriggerWorker()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
