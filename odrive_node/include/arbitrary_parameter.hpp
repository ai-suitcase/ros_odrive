#ifndef ARBITRARY_PARAMETER_HPP
#define ARBITRARY_PARAMETER_HPP

#include <iostream>
#include <fstream>
#include <any>
#include <nlohmann/json.hpp>
#include <boost/bimap.hpp>
#include <mutex>
#include <condition_variable>
#include "demangle.hpp"

class ArbitraryParameter {
public:
    ArbitraryParameter(){}
    ~ArbitraryParameter(){}
    void init(const std::string& json_file_path);
    bool contains(const std::string& name);
    bool contains(uint16_t id);

    template<typename T>
    void set_fresh(uint16_t id, T input_val);

    template<typename T>
    void get_fresh(uint16_t id, T& output_val);

    std::string get_type(uint16_t id); // obtain mangled type name
    std::string get_type_demangled(uint16_t id); // obtain demangled type name
    uint16_t get_id(const std::string& name);
private:
    bool contains_float(uint16_t id);
    bool contains_bool(uint16_t id);
    std::string get_name(uint16_t id);
    std::mutex& get_mutex(uint16_t id);
    std::condition_variable& get_cv(uint16_t id);
    bool& get_flag(uint16_t id);

    std::map<uint16_t, std::any> parameters;
    //std::map<uint16_t, uint16_t> endpoint_ref_parameters;
    //std::map<uint16_t, uint16_t> function_parameters;
    //std::vector<uint16_t> ids_;
    boost::bimaps::bimap<uint16_t, std::string> names_;
    //std::map<uint16_t, std::string> names_;
    std::map<uint16_t, std::string> types_;

    std::map<uint16_t, std::mutex> mutexes_;
    std::map<uint16_t, std::condition_variable> cvs_;
    std::map<uint16_t, bool> flags_;
};

template<typename T>
void ArbitraryParameter::set_fresh(uint16_t id, T input_val) {
    std::string name = get_name(id);
    if(!contains(name)) {
        std::cerr << "endpoint name " << name << " does not exist" << std::endl;
        return;
    }
    if(get_type(id) != typeid(T).name()) {
        std::cerr << "endpoint name " << name << " is a " << get_type_demangled(id);
	std::cerr << " type parameter, not " << typeid(T).name() << std::endl;
        return;
    }
    {
        std::mutex& input_val_mutex = get_mutex(id);
        std::unique_lock<std::mutex> guard(input_val_mutex);
        bool& resource_ready = get_flag(id);
	std::any& target_parameter = parameters.at(id);
        target_parameter = input_val;
        resource_ready = true;
    }
    get_cv(id).notify_all();
    return;
}

template<typename T>
void ArbitraryParameter::get_fresh(uint16_t id, T& output_val) {
    std::string name = get_name(id);
    if(!contains(name)) {
        std::cerr << "endpoint name " << name << " does not exist" << std::endl;
        return;
    }
    if(get_type(id) != typeid(T).name()) {
        std::cerr << "endpoint name " << name << " is a " << get_type_demangled(id);
	std::cerr << " type parameter, not " << typeid(T).name() << std::endl;
        return;
    }
    std::mutex& output_val_mutex = get_mutex(id);
    std::unique_lock<std::mutex> guard(output_val_mutex);
    bool& resource_ready = get_flag(id);
    if(get_cv(id).wait_for(guard, std::chrono::milliseconds(100), [&resource_ready]{return resource_ready;})) {
        // parameter is updated in time
        output_val = std::any_cast<T>(parameters.find(id)->second);
        resource_ready = false;
    } else {
        // parameter is not updated
        std::cerr << "sync thread timeout: cannot receive " << name << " in time" << std::endl;
    }
    return;
}
#endif // ARBITRARY_PARAMETER_HPP
