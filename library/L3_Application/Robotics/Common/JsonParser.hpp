// constructor
#include <string_view>
#include <string>
#include <unistd.h>

#include "third_party/json-c/json.h"
#include "third_party/json-c/json_tokener.h"

#include "library/L3_Application/Robotics/Common/parser.cpp"

class JsonParser
{
 public:
  explicit constexpr JsonParser() : json_data_(NULL) {}
  void SetJsonString(const std::string_view & json_string)
  {
    json_data_ = json_tokener_parse(json_string.data());
  }

  int GetIntField(const std::string_view & field, int * buffer)
  {
    struct json_object * result;
    json_object_object_get_ex(json_data_, field.data(), &result);
    *buffer = json_object_get_int(result);
  }

  char GetStringField(const std::string_view & field, char * buffer)
  {
    struct json_object * result;
    json_object_object_get_ex(json_data_, field.data(), &result);
    strcpy(buffer, json_object_get_string(result));
  }

 private:
  struct json_object * json_data_;
};
