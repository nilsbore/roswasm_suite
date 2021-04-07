#ifndef ROSCPP_JSON_SERIALIZE_H
#define ROSCPP_JSON_SERIALIZE_H

#include <ros/ros.h>
#include <ros/message_operations.h>

//#include <iomanip>
//#include <limits>

namespace roscpp_json {
   
class JSONStream {

private:

    struct ParsingContext {
        bool has_key;
        std::string key;
        bool key_list;
        bool is_list;
        int list_index;
        bool first_value;
        std::stringstream stream;
        ParsingContext() : has_key(false), key_list(false), is_list(false), list_index(-1), first_value(true) {}
    };

    size_t current_indent;
    size_t initial_indent;
    bool indent_needs_handling;
    bool output_service_list;
    bool only_bool;
    std::list<ParsingContext> contexts;

    ParsingContext& context()
    {
        return contexts.back();
    }

    std::stringstream& stream()
    {
        return context().stream;
    }

    void enter_dict()
    {
        stream() << "{ ";
    }

    void exit_dict()
    {
        stream() << " }";
    }

    void enter_list()
    {
        stream() << "[ ";
    }

    void exit_list()
    {
        stream() << " ]";
    }

    void add_delimiter()
    {
        stream() << ", ";
    }

    template <typename T>
    void add_key_value(const std::string& key, const T& value)
    {
        if (context().first_value) {
            context().first_value = false;
        }
        else {
            add_delimiter();
        }

        if (output_service_list && contexts.size() == 1) {
            add_list_value_impl(value);
        }
        else {
            add_key_value_impl(key, value);
        }
    }

    template <typename T>
    void add_key_value_impl(const std::string& key, const T& value)
    {
        stream() << "\"" << key << "\": " << value;
    }
    
    void add_key_value_impl(const std::string& key, const std::stringstream& value)
    {
        stream() << "\"" << key << "\": " << value.str();
    }

    void add_key_value_impl(const std::string& key, const std::string& value)
    {
        stream() << "\"" << key << "\": \"" << value << "\"";
    }

    void add_key_value_impl(const std::string& key, const float& value)
    {
        stream() << "\"" << key << "\": ";
        if (std::isnan(value)) {
            stream() << "NaN";
        }
        else if (std::isinf(value)) {
            if (value < 0) {
                stream() << "-";
            }
            stream() << "Infinity";
        }
        else {
            stream() << std::setprecision(std::numeric_limits<float>::max_digits10) << value;
        }
    }

    void add_key_value_impl(const std::string& key, const double& value)
    {
        stream() << "\"" << key << "\": ";
        if (std::isnan(value)) {
            stream() << "NaN";
        }
        else if (std::isinf(value)) {
            if (value < 0) {
                stream() << "-";
            }
            stream() << "Infinity";
        }
        else {
            stream() << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
        }
    }

    void add_key_value_impl(const std::string& key, const uint32_t& value)
    {
        stream() << "\"" << key << "\": ";
        if (!only_bool) {
            stream() << value;
        }
        else if (value) {
            stream() << "true";
        }
        else {
            stream() << "false";
        }
    }

    void add_key_value_impl(const std::string& key, const ros::Time& value)
    {
        stream() << "\"" << key << "\": { \"secs\": " << value.sec << ", \"nsecs\": " << value.nsec << " }";
    }

    template <typename T>
    void add_list_value(const T& value)
    {
        if (context().list_index != -1) {
            if (context().first_value) {
                context().first_value = false;
            }
            else {
                add_delimiter();
            }
            add_list_value_impl(value);
            context().list_index = -1;
        }
        else {
            add_list_value_impl(value);
        }
    }

    template <typename T>
    void add_list_value_impl(const T& value)
    {
        stream() << value;
    }

    void add_list_value_impl(const size_t& value)
    {
        if (context().list_index == -1) {
            context().list_index = value;
        }
        else {
            stream() << value;
        }
    }
    
    void add_list_value_impl(const std::stringstream& value)
    {
        stream() << value.str();
    }

    void add_list_value_impl(const std::string& value)
    {
        stream() << "\"" << value << "\"";
    }

    void handle_indent(size_t indent)
    {
        size_t new_indent = initial_indent + indent;

        if (new_indent <= current_indent && context().key_list) {
            contexts.push_back(ParsingContext());
            context().is_list = true;
            enter_list();
            exit_context(current_indent++);
        }

        if (new_indent < current_indent) {
            exit_context(new_indent);
        }
        else if (new_indent > current_indent) {
            if (context().key_list) {
                contexts.push_back(ParsingContext());
                context().is_list = true;
                enter_list();
            }
            else {
                contexts.push_back(ParsingContext());
                enter_dict();
            }
            current_indent = new_indent;
        }

        indent_needs_handling = false;
    }

    void parse_string(const std::string& value)
    {
        size_t start = value.find_first_not_of(' ');
        if (start == std::string::npos) {
            return;
        }
        else if (indent_needs_handling) {
            handle_indent(start/2);
        }

        if (context().is_list) {
            return;
        }

        size_t send = value.substr(start, value.size()-start).find(':');
        if (send == std::string::npos) {
            if (context().has_key) {
                add_key_value(context().key, value.substr(start, value.size()-start));
                context().has_key = false;
            }
            else {
                send = value.substr(start, value.size()-start).find("[]");
                if (send != std::string::npos) {
                    context().has_key = true;
                    context().key = value.substr(start, send-start);
                    context().key_list = true;
                }
            }
        }
        else {
            std::string key = value.substr(start, send-start);
            context().has_key = true;
            context().key = key;
            context().key_list = false;
        }
    }

    void exit_context(int new_indent)
    {
        while (current_indent > new_indent) {

            if (context().is_list) {
                exit_list();
            }
            else {
                exit_dict();
            }

            std::stringstream context_stream;
            context_stream << stream().rdbuf();
            contexts.pop_back();
            if (context().has_key) {
                add_key_value(context().key, context_stream);
                context().has_key = false;
            }
            else if (context().is_list) {
                add_list_value(context_stream);
            }

            --current_indent;
        }
    }

public:

    template <typename T>
    JSONStream& operator<<(T value)
    {
        if (context().is_list) {
            add_list_value(value);
        }
        else if (context().has_key) {
            add_key_value(context().key, value);
            context().has_key = false;
        }
        return *this;
    }

    JSONStream& operator<<(const char* value)
    {
        if (*value == '\n') {
            return *this;
        }
        parse_string(value);

        return *this;
    }

    JSONStream& operator<<(const std::string& value)
    {
        if (value.empty() || value.find_first_not_of(' ') != std::string::npos) {
            if (context().is_list) {
                add_list_value(value);
            }
            else if (context().has_key) {
                add_key_value(context().key, value);
                context().has_key = false;
            }
        }
        else {
            initial_indent = value.size()/2;
            indent_needs_handling = true;
            return *this;
        }

        return *this;
    }

    // these are all for handling std::endl
    typedef std::basic_ostream<char, std::char_traits<char> > CoutType;
    typedef CoutType& (*StandardEndLine)(CoutType&);
    JSONStream& operator<<(StandardEndLine manip)
    {
        return *this;
    }

    JSONStream(bool output_service_list=false, bool only_bool=false) : current_indent(1), initial_indent(0), indent_needs_handling(false), output_service_list(output_service_list), only_bool(only_bool)
    {
        contexts.push_back(ParsingContext());
    }

    std::string str()
    {
        //exit_context(1);
        initial_indent = 0;
        handle_indent(1);
        if (output_service_list) {
            return std::string("[ ") + stream().str() + " ]";
        }
        else {
            return std::string("{ ") + stream().str() + " }";
        }
    }

};

template <typename MSG>
std::string serialize(const MSG& msg, bool output_service_list=false, bool only_bool=false)
{
    roscpp_json::JSONStream stream(output_service_list, only_bool);
    ros::message_operations::Printer<MSG>::stream(stream, "  ", msg);
    return stream.str();
}

} // namespace roscpp_json

#endif // ROSCPP_JSON_SERIALIZE_H
