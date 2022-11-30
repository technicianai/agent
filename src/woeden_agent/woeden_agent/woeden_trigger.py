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
    comparison: List[Comparison]
    duration: int #number of seconds for recording after the trigger
    enabled: bool # Whether this trigger is enabled
    id: int # ID for this particular trigger
    topic: str # topic name to inspect
    type: str # type of the topic
    msgdef_python: str
    topics: List[Topic] # topics to log

    @staticmethod
    def from_dict(obj: Any) -> 'Trigger':
        _base_path = str(obj.get("base_path"))
        _comparison = [Comparison.from_dict(y) for y in obj.get("comparison")]
        _duration = int(obj.get("duration"))
        _enabled = bool(obj.get("enabled"))
        _id = int(obj.get("id"))
        _topic = str(obj.get("topic"))
        _type = str(obj.get("type"))
        _msgdef_python = str(obj.get("msgdef_python"))
        _topics = [Topic.from_dict(y) for y in obj.get("topics")]

        return Trigger(_base_path, _comparison, _duration, _enabled, _id, _topic, _type, _msgdef_python, _topics)

@dataclass
class Triggers:
    triggers: List[Trigger]
    @staticmethod
    def from_dict(obj: Any) -> 'Triggers':
        return Triggers(obj.get("triggers"))
