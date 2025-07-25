# 源文件
https://github.com/nlohmann/json

# 使用说明
## converting c class to json
来源: https://stackoverflow.com/questions/8220130/converting-c-class-to-json

```cpp
#include "MyUtils/third-party/nlohmann_json/json.hpp"

#include <iostream>
#include <string>

using namespace nlohmann;

class Example
{
    std::string string = "name";
    std::map< std::string, std::string > map;
    std::vector< int > vector;
public:
    // Add this to your code.
    // NLOHMANN_DEFINE_TYPE_INTRUSIVE( Example, string, map, vector );
    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Example, string, map, vector);
    // Since the class members are private you can use the macro:
    // NLOHMANN_DEFINE_TYPE_INTRUSIVE(...).
};

struct MyStruct
{
    std::string name;
    int number;

    NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE( MyStruct, name, number );
    // Note how I used NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE because the struct
    // members are public.
};
int main( )
{
    nlohmann::json j;

    Example ex;
    ex.to_json( j, ex );
    std::cout << j << std::endl;//{"map":{},"string":"name","vector":[]}
    

    MyStruct s;
    s.name = "test";
    s.number = 11;

    s.to_json( j, s );
    MyStruct t;
    s.from_json(j, t);
    std::cout << j << std::endl;//{"map":{},"name":"test","number":11,"string":"name","vector":[]}
    std::cout << t.name << std::endl;//test
    // {"name":"test","number":11}
    return 0;
}

```
