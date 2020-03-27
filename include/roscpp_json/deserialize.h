#ifndef ROSCPP_JSON_DESERIALIZE_H
#define ROSCPP_JSON_DESERIALIZE_H

#include <ros/ros.h>
#include <ros/serialization.h>
#include <rapidjson/document.h>
#include <cassert>

namespace roscpp_json {

class JSONIter {
public:

    using ContainerAllocator = std::allocator<void>;
    using Serializer = ros::serialization::Serializer<ContainerAllocator>;

    rapidjson::Value::ConstMemberIterator itr;

    // this can only be valur or object, no array
    template <typename T>
    typename std::enable_if<std::is_class<T>::value>::type extract(T& value, const rapidjson::Value& itr_value)
    {
        assert(itr_value.IsObject());
        JSONIter subiter;
        subiter.parse_impl(value, itr_value);
    }

    template <typename T>
    typename std::enable_if<std::is_integral<T>::value>::type extract(T& value, const rapidjson::Value& itr_value)
    {
        assert(itr_value.IsInt());
        value = itr_value.GetInt();
    }

    template <typename T>
    typename std::enable_if<std::is_floating_point<T>::value>::type extract(T& value, const rapidjson::Value& itr_value)
    {
        assert(itr_value.IsNumber());
        if (itr_value.IsDouble()) {
            value = itr_value.GetDouble();
        }
        else {
            value = T(itr_value.GetDouble());
        }
    }

    void extract(std::string& value, const rapidjson::Value& itr_value)
    {
        assert(itr_value.IsString());
        value = std::string(itr_value.GetString());
    }

    void extract(ros::Time& value, const rapidjson::Value& itr_value)
    {
        assert(itr_value.HasMember("secs"));
        assert(itr_value.HasMember("nsecs"));
        value.sec = itr_value["secs"].GetInt();
        value.nsec = itr_value["nsecs"].GetInt();
    }

    template <typename T>
    void next(T& value)
    {
        extract(value, itr->value);
        ++itr;
    }

    template <typename T, size_t N>
    void next(boost::array<T, N>& value)
    {
        assert(itr->value.IsArray());
        size_t length = itr->value.Size();
        assert(length == N);
        for (size_t i = 0; i < length; ++i) {
            extract(value[i], itr->value[i]);
        }
        ++itr;
    }

    template <typename T, typename Allocator>
    void next(std::vector<T, Allocator>& value)
    {
        assert(itr->value.IsArray());
        size_t length = itr->value.Size();
        value.resize(length);
        for (size_t i = 0; i < length; ++i) {
            extract(value[i], itr->value[i]);
        }
        ++itr;
    }

    template <typename MSG>
    void parse_impl(MSG& msg, const rapidjson::Value& itr_value)
    {
        itr = itr_value.MemberBegin();
        ros::serialization::Serializer<MSG>::template allInOne<JSONIter, MSG&>(*this, msg);
        assert(itr == itr_value.MemberEnd());
    }

    template <typename MSG>
    MSG parse(const std::string& json)
    {
        rapidjson::Document document;
        document.Parse(json.c_str());
        // this may not work for service returns
        assert(document.IsObject());
        MSG msg;
        parse_impl(msg, document);
        return msg;
    }

    JSONIter()
    {
    }

};

template <typename MSG>
MSG deserialize(const std::string& json)
{
    roscpp_json::JSONIter iter;
    return iter.parse<MSG>(json);
}

} // namespace roscpp_json

#endif // ROSCPP_JSON_DESERIALIZE
