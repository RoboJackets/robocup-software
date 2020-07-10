/** @file */
#pragma once

#include <rclcpp/node.hpp>

namespace params {
namespace internal {
/**
 * @brief Class used by the DEFINE_* macros to register a parameter, which
 * happens in the constructor.
 */
class ParamRegisterer {
public:
    template <typename ParamType>
    ParamRegisterer(const char* module, const char* prefix, const char* name,
                    const char* help, const char* filename,
                    ParamType& current_storage);
};
}  // namespace internal

/**
 * @brief A Param represents a parameter.
 * @tparam T The type of the parameter.
 */
template <typename T>
class Param {
public:
    using Ptr = std::unique_ptr<Param>;
    static constexpr auto kPrefixSeparator = "::";

    Param(const char* module, const char* prefix, const char* name,
          const char* help, const char* filename, T& param)
        : module_{module},
          prefix_{prefix},
          name_{name},
          full_name_{prefix_.empty() ? name : prefix_ + "::" + name_},
          help_{help},
          filename_{filename},
          default_value_{param},
          param_{param} {}

    /**
     * @brief Updates the parameter with the new value.
     * @param new_value The new value of the parameter.
     */
    void Update(T&& new_value) { param_ = std::move(new_value); }
    void Update(const T& new_value) { param_ = new_value; }

    /**
     * Returns the current value of the parameter.
     * @return Current value of the parameter.
     */
    const T& value() const { return param_; }

    /**
     * Returns the default value of the parameter.
     * @return Default value of the parameter.
     */
    const T& default_value() const { return default_value_; }

    [[nodiscard]] const std::string& module() const { return module_; }
    [[nodiscard]] const std::string& prefix() const { return prefix_; }
    [[nodiscard]] const std::string& name() const { return name_; }
    [[nodiscard]] const std::string& full_name() const { return full_name_; }

    [[nodiscard]] const std::string& help() const { return help_; }

    template <typename ParamType>
    friend std::ostream& operator<<(std::ostream& os,
                                    const Param<ParamType>& param);

private:
    const std::string module_;
    const std::string prefix_;
    const std::string name_;
    const std::string full_name_;
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
    explicit ParamProvider(const std::string& module) : module_{module} {};

    template <typename T>
    using ParamMap = std::unordered_map<std::string, typename Param<T>::Ptr>;

    /**
     * @brief Gets the value of a parameter with the passed in param_name,
     * returning true if the parameter was found.
     * @tparam ParamType Type of the parameter.
     * @param full_name Name of the parameter to find.
     * @param[out] value Variable to be filled in if the parameter is found.
     * @return Whether the parameter was found and value was filled in.
     */
    template <typename ParamType>
    bool Get(const std::string& full_name, ParamType* value) const;

    /**
     * @brief Updates the parameter with the passed in full_name and matching
     * type with new_value.
     * @tparam ParamType Type of the parameter.
     * @param full_name The full_name of the parameter.
     * @param new_value The new value of the parameter.
     */
    template <typename ParamType>
    void Update(const std::string& full_name, const ParamType& new_value);

    /**
     * @brief Tries to update the parameter with the passed in full_name and
     * matching type with new_value.
     * @tparam ParamType Type of the parameter.
     * @param full_name The full_name of the parameter.
     * @param new_value The new value of the parameter.
     * @return Whether the parameter was found and updated.
     */
    template <typename ParamType>
    bool TryUpdate(const std::string& full_name, const ParamType& new_value);

    /**
     * Returns the corresponding parameter map of the given ParamType.
     * @tparam ParamType The type of the ParamMap to return.
     * @return The ParamMap for ParamType.
     */
    template <typename ParamType>
    ParamMap<ParamType>& GetParamMap();

protected:
    std::string module_;
};

}  // namespace params

/**
 * @brief Defines a namespaced parameter. The defined parameter can be used as
 * prefix::PARAM_#, where # denotes the name passed in.
 * @param type The type of the parameter.
 * @param prefix The prefix / namespace of the parameter, using :: as a
 * separator.
 * @param name The variable name of the parameter.
 * @param val The default value of the parameter.
 * @param description A description of the parameter.
 */
#define DEFINE_NS_VARIABLE(type, module, prefix, name, val, description) \
    namespace params::storage::prefix {                                  \
    static type PARAM_##name##_storage = val;                            \
    }                                                                    \
    namespace prefix {                                                   \
    const type& PARAM_##name =                                           \
        params::storage::prefix::PARAM_##name##_storage;                 \
    }                                                                    \
    namespace params::param_registerer::prefix {                         \
    static ::params::internal::ParamRegisterer o_##name(                 \
        module, #prefix, #name, description, __FILE__,                   \
        params::storage::prefix::PARAM_##name##_storage);                \
    }

/**
 * @brief Defines a parameter in the root namespace. The defined parameter
 * can be used as PARAM_#, where * denotes # denotes the name passed in.
 * See DEFINE_NS_VARIABLE.
 * @param type The type of the parameter.
 * @param name The variable name of the parameter.
 * @param val The default value of the parameter.
 * @param description A description of the parameter.
 */
#define DEFINE_VARIABLE(type, module, name, val, description)              \
    namespace params::variables {                                          \
    static type PARAM_##name##_storage = val;                              \
    const type& PARAM_##name = PARAM_##name##_storage;                     \
    static ::params::internal::ParamRegisterer o_##name(                   \
        module, "", #name, description, __FILE__, PARAM_##name##_storage); \
    }                                                                      \
    using params::variables::PARAM_##name;

// Define the DEFINE_* macro for all supported types.
#define DEFINE_NS_BOOL(module, prefix, name, val, description) \
    DEFINE_NS_VARIABLE(bool, module, prefix, name, val, description)
#define DEFINE_NS_INT64(module, prefix, name, val, description) \
    DEFINE_NS_VARIABLE(int64_t, module, prefix, name, val, description)
#define DEFINE_NS_FLOAT64(module, prefix, name, val, description) \
    DEFINE_NS_VARIABLE(double, module, prefix, name, val, description)
#define DEFINE_NS_STRING(module, prefix, name, val, description) \
    DEFINE_NS_VARIABLE(std::string, module, prefix, name, val, description)
#define DEFINE_NS_BYTE_VEC(module, prefix, name, val, description)      \
    DEFINE_NS_VARIABLE(std::vector<uint8_t>, module, prefix, name, val, \
                       description)
#define DEFINE_NS_BOOL_VEC(module, prefix, name, val, description)   \
    DEFINE_NS_VARIABLE(std::vector<bool>, module, prefix, name, val, \
                       description)
#define DEFINE_NS_INT64_VEC(module, prefix, name, val, description)     \
    DEFINE_NS_VARIABLE(std::vector<int64_t>, module, prefix, name, val, \
                       description)
#define DEFINE_NS_FLOAT64_VEC(module, prefix, name, val, description)  \
    DEFINE_NS_VARIABLE(std::vector<double>, module, prefix, name, val, \
                       description)
#define DEFINE_NS_STRING_VEC(module, prefix, name, val, description)        \
    DEFINE_NS_VARIABLE(std::vector<std::string>, module, prefix, name, val, \
                       description)

#define DEFINE_BOOL(module, name, val, description) \
    DEFINE_VARIABLE(bool, module, name, val, description)
#define DEFINE_INT64(module, name, val, description) \
    DEFINE_VARIABLE(int64_t, module, name, val, description)
#define DEFINE_FLOAT64(module, name, val, description) \
    DEFINE_VARIABLE(double, module, name, val, description)
#define DEFINE_STRING(module, name, val, description) \
    DEFINE_VARIABLE(std::string, module, name, val, description)
#define DEFINE_BYTE_VEC(module, name, val, description) \
    DEFINE_VARIABLE(std::vector<uint8_t>, module, name, val, description)
#define DEFINE_BOOL_VEC(module, name, val, description) \
    DEFINE_VARIABLE(std::vector<bool>, module, name, val, description)
#define DEFINE_INT64_VEC(module, name, val, description) \
    DEFINE_VARIABLE(std::vector<int64_t>, module, name, val, description)
#define DEFINE_FLOAT64_VEC(module, name, val, description) \
    DEFINE_VARIABLE(std::vector<double>, module, name, val, description)
#define DEFINE_STRING_VEC(module, name, val, description) \
    DEFINE_VARIABLE(std::vector<std::string>, module, name, val, description)
