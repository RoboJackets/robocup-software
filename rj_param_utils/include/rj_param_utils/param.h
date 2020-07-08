/** @file */
#pragma once

#include <rclcpp/node.hpp>

namespace params::internal {

/**
 * @brief A Param represents a parameter.
 * @tparam T The type of the parameter.
 */
template <typename T>
class Param {
public:
    using Ptr = std::unique_ptr<Param>;

    Param(const char* name, const char* help, const char* filename, T& param)
        : name_{name},
          help_{help},
          filename_{filename},
          default_value_{param},
          param_{param} {}

    /**
     * @brief Updates the parameter with the new value.
     * @param new_value The new value of the parameter.
     * @throws std::runtime_error if the type is set.
     */
    void Update(T&& new_value) { param_ = std::move(new_value); }
    void Update(const T& new_value) { param_ = new_value; }

    const T& default_value() const { return default_value_; }

    [[nodiscard]] const std::string& name() const { return name_; }

    [[nodiscard]] const std::string& help() const { return help_; }

    template <typename ParamType>
    friend std::ostream& operator<<(std::ostream& os,
                                    const Param<ParamType>& param);

private:
    const std::string name_;
    const std::string help_;
    const std::string filename_;
    const T& default_value_;
    T& param_;
};

/**
 * @brief A ParamProvider represents a "provider" for parameters. In particular,
 * it should provide a way of updating the parameters.
 */
class ParamProvider {
public:
    template <typename T>
    using ParamMap = std::unordered_map<std::string, typename Param<T>::Ptr>;

    template <typename ParamType>
    void Update(const std::string& param_name, const ParamType& new_value);

    template <typename ParamType>
    bool TryUpdate(const std::string& param_name, const ParamType& new_value);

    template <typename ParamType>
    ParamMap<ParamType>& GetParamMap();
};

}  // namespace params::internal

/**
 * @brief Defines a namespaced parameter. The defined parameter can be used as
 * PARAM_#, where * denotes # denotes the name passed in.
 * @param type The type of the parameter.
 * @param prefix The prefix / namespace of the parameter, using . as a
 * separator.
 * @param name The variable name of the parameter.
 * @param val The default value of the parameter.
 * @param description A description of the parameter.
 */
#define DEFINE_NAMESPACED_VARIABLE(type, prefix, name, val, description)      \
    namespace params::variables {                                             \
    static type PARAM_##name##_storage = val;                                 \
    const type& PARAM_##name = PARAM_##name##_storage;                        \
    static params::internal::FlagRegisterer o_##name(#prefix "." #name,       \
                                                     description, __FILE__,   \
                                                     PARAM_##name##_storage); \
    }                                                                         \
    using params::variables::PARAM_##name;

/**
 * @brief Defines a parameter in the root namespace. The defined parameter
 * can be used as PARAM_#, where * denotes # denotes the name passed in.
 * See DEFINE_NAMESPACED_VARIABLE.
 * @param type The type of the parameter.
 * @param name The variable name of the parameter.
 * @param val The default value of the parameter.
 * @param description A description of the parameter.
 */
#define DEFINE_VARIABLE(type, name, val, description) \
    DEFINE_NAMESPACED_VARIABLE(type, , name, val, description)

// Define the DEFINE_* macro for all supported types.
#define DEFINE_NAMESPACED_BOOL(prefix, name, val, description) \
    DEFINE_NAMESPACED_VARIABLE(bool, prefix, name, val, description)
#define DEFINE_NAMESPACED_INT64(prefix, name, val, description) \
    DEFINE_NAMESPACED_VARIABLE(int64_t, prefix, name, val, description)
#define DEFINE_NAMESPACED_FLOAT64(prefix, name, val, description) \
    DEFINE_NAMESPACED_VARIABLE(double, prefix, name, val, description)
#define DEFINE_NAMESPACED_STRING(prefix, name, val, description) \
    DEFINE_NAMESPACED_VARIABLE(std::string, prefix, name, val, description)
#define DEFINE_NAMESPACED_BYTE_VEC(prefix, name, val, description)      \
    DEFINE_NAMESPACED_VARIABLE(std::vector<uint8_t>, prefix, name, val, \
                               description)
#define DEFINE_NAMESPACED_BOOL_VEC(prefix, name, val, description)   \
    DEFINE_NAMESPACED_VARIABLE(std::vector<bool>, prefix, name, val, \
                               description)
#define DEFINE_NAMESPACED_INT64_VEC(prefix, name, val, description)     \
    DEFINE_NAMESPACED_VARIABLE(std::vector<int64_t>, prefix, name, val, \
                               description)
#define DEFINE_NAMESPACED_FLOAT64_VEC(prefix, name, val, description)  \
    DEFINE_NAMESPACED_VARIABLE(std::vector<double>, prefix, name, val, \
                               description)
#define DEFINE_NAMESPACED_STRING_VEC(prefix, name, val, description)        \
    DEFINE_NAMESPACED_VARIABLE(std::vector<std::string>, prefix, name, val, \
                               description)

#define DEFINE_BOOL(name, val, description) \
    DEFINE_VARIABLE(bool, name, val, description)
#define DEFINE_INT64(name, val, description) \
    DEFINE_VARIABLE(int64_t, name, val, description)
#define DEFINE_FLOAT64(name, val, description) \
    DEFINE_VARIABLE(double, name, val, description)
#define DEFINE_STRING(name, val, description) \
    DEFINE_VARIABLE(std::string, name, val, description)
#define DEFINE_BYTE_VEC(name, val, description) \
    DEFINE_VARIABLE(std::vector<uint8_t>, name, val, description)
#define DEFINE_BOOL_VEC(name, val, description) \
    DEFINE_VARIABLE(std::vector<bool>, name, val, description)
#define DEFINE_INT64_VEC(name, val, description) \
    DEFINE_VARIABLE(std::vector<int64_t>, name, val, description)
#define DEFINE_FLOAT64_VEC(name, val, description) \
    DEFINE_VARIABLE(std::vector<double>, name, val, description)
#define DEFINE_STRING_VEC(name, val, description) \
    DEFINE_VARIABLE(std::vector<std::string>, name, val, description)
