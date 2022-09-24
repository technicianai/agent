#include "key_value_comparison.hpp"

#include <string>

using namespace std;

namespace woeden
{
key_value_comparison::key_value_comparison(string field, string comparator, string value, string value_type)
{
  field_ = field;
  comparator_ = comparator;
  value_ = value;
  value_type_ = value_type;
}

bool key_value_comparison::evaluate(nlohmann::json data)
{
  if (!data.contains(field_)) return false;

  if (value_type_ == "STRING") {
    string value = data[field_].get<string>();
    return value == value_;
  } else {
    double comp = stod(value_);
    double value = data[field_].get<double>();
    if (comparator_ == "EQUAL_TO") {
      return abs(value - comp) < 0.1;
    } else if (comparator_ == "GREATER_THAN") {
      return value > comp;
    } else if (comparator_ == "LESS_THAN") {
      return value < comp;
    }
  }
  return false;
}

bool key_value_comparison::evaluate(diagnostic_msgs::msg::KeyValue data)
{
  if (data.key != field_) return false;

  if (value_type_ == "STRING") {
    return data.value == value_;
  } else {
    double comp = stod(value_);
    double value = stod(data.value);
    if (comparator_ == "EQUAL_TO") {
      return abs(value - comp) < 0.1;
    } else if (comparator_ == "GREATER_THAN") {
      return value > comp;
    } else if (comparator_ == "LESS_THAN") {
      return value < comp;
    }
  }
  return false;
}

nlohmann::json key_value_comparison::to_json()
{
  nlohmann::json json;
  json["field"] = field_;
  json["comparator"] = comparator_;
  json["value"] = value_;
  json["value_type"] = value_type_;
  return json;
}

key_value_comparison* key_value_comparison::from_json(nlohmann::json data)
{
  return new key_value_comparison(
    data["field"].get<string>(),
    data["comparator"].get<string>(),
    data["value"].get<string>(),
    data["value_type"].get<string>()
  );
}
}
