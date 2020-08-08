/** @file */
#include <cstdint>
#include <memory>
#include <string>

#include <rj_param_utils/param.h>

namespace params {
namespace internal {
template <typename T>
using ParamMap = std::unordered_map<std::string, typename Param<T>::Ptr>;

/**
 * @brief ParamMaps is a struct that contains the various ParamMap for
 * different types.
 */
struct ParamMaps {
    template <typename T>
    ParamMap<T>& Get();

    ParamMap<bool> bools;
    ParamMap<int64_t> int64s;
    ParamMap<double> doubles;
    ParamMap<std::string> strings;
    ParamMap<std::vector<uint8_t>> byte_vecs;
    ParamMap<std::vector<bool>> bool_vecs;
    ParamMap<std::vector<int64_t>> int64_vecs;
    ParamMap<std::vector<double>> float64_vecs;
    ParamMap<std::vector<std::string>> string_vecs;
};

template <>
ParamMap<bool>& ParamMaps::Get<bool>() {
    return bools;
}
template <>
ParamMap<int64_t>& ParamMaps::Get<int64_t>() {
    return int64s;
}
template <>
ParamMap<double>& ParamMaps::Get<double>() {
    return doubles;
}
template <>
ParamMap<std::string>& ParamMaps::Get<std::string>() {
    return strings;
}
template <>
ParamMap<std::vector<uint8_t>>& ParamMaps::Get<std::vector<uint8_t>>() {
    return byte_vecs;
}
template <>
ParamMap<std::vector<bool>>& ParamMaps::Get<std::vector<bool>>() {
    return bool_vecs;
}
template <>
ParamMap<std::vector<int64_t>>& ParamMaps::Get<std::vector<int64_t>>() {
    return int64_vecs;
}
template <>
ParamMap<std::vector<double>>& ParamMaps::Get<std::vector<double>>() {
    return float64_vecs;
}
template <>
ParamMap<std::vector<std::string>>& ParamMaps::Get<std::vector<std::string>>() {
    return string_vecs;
}

/**
 * @brief A ParamRegistry represents a "registry" that keeps track of all the
 * parameters. Used by ParamProvider to query and update registered parameters.
 */
class ParamRegistry {
public:
    template <typename T>
    using ParamMap = std::unordered_map<std::string, typename Param<T>::Ptr>;

    static ParamRegistry& GlobalRegistry() {
        static ParamRegistry global_registry;
        return global_registry;
    }

    template <typename ParamType>
    ParamMap<std::decay_t<ParamType>>& GetParamMap(const std::string& module) {
        return params_[module].template Get<std::decay_t<ParamType>>();
    }

    template <typename ParamType>
    void RegisterParam(typename Param<ParamType>::Ptr param) {
        const std::string param_name = param->full_name();
        const std::string& module = param->module();
        GetParamMap<ParamType>(module).insert(std::make_pair(param_name, std::move(param)));
    }

    template <typename ParamType>
    [[nodiscard]] bool HasParam(const std::string& module, const std::string& full_name) {
        if (params_.find(module) == params_.end()) {
            return false;
        }
        ParamMaps& param_maps = params_[module];
        ParamMap<ParamType>& param_map = param_maps.Get<ParamType>();
        auto it = param_map.find(full_name);
        return it != param_map.end();
    }

    template <typename ParamType>
    [[nodiscard]] bool GetParam(const std::string& module, const std::string& full_name,
                                ParamType* value) {
        auto& param_map = GetParamMap<ParamType>(module);
        auto it = param_map.find(full_name);
        if (it == param_map.end()) {
            return false;
        }
        *value = it->second->value();
        return true;
    }

    template <typename ParamType>
    void UpdateParam(const std::string& module, const std::string& full_name,
                     ParamType&& new_value) {
        auto& param_map = GetParamMap<ParamType>(module);
        auto it = param_map.find(full_name);
        if (it == param_map.end()) {
            throw std::runtime_error("Couldn't find parameter with name " + full_name);
        }
        it->second->Update(new_value);
    }

    // Delete all other constructors.
    ParamRegistry(const ParamRegistry&) = delete;
    ParamRegistry& operator=(const ParamRegistry&) = delete;

    ParamRegistry(ParamRegistry&&) = delete;
    ParamRegistry& operator=(ParamRegistry&&) = delete;

private:
    // Prevent ParamRegistry from being constructed outside of GlobalRegistry()
    // method by making the ctor and dtor private.
    ParamRegistry() = default;
    ~ParamRegistry() = default;

    std::unordered_map<std::string, ParamMaps> params_;
};

template <typename ParamType>
ParamRegisterer::ParamRegisterer(const char* module, const char* prefix, const char* name,
                                 const char* help, const char* filename,
                                 ParamType& current_storage) {
    std::unique_ptr<Param<ParamType>> param =
        std::make_unique<Param<ParamType>>(module, prefix, name, help, filename, current_storage);
    ParamRegistry::GlobalRegistry().template RegisterParam<ParamType>(std::move(param));
}

#define INSTANTIATE_PARAM_REGISTERER_CTOR(type)                                       \
    template ParamRegisterer::ParamRegisterer(const char* module, const char* prefix, \
                                              const char* name, const char* help,     \
                                              const char* filename, type& current_storage);

// Do this for all supported flag types. For now these correspond to the ROS2
// parameter types.
INSTANTIATE_PARAM_REGISTERER_CTOR(bool)
INSTANTIATE_PARAM_REGISTERER_CTOR(int64_t)
INSTANTIATE_PARAM_REGISTERER_CTOR(double)
INSTANTIATE_PARAM_REGISTERER_CTOR(std::string)
INSTANTIATE_PARAM_REGISTERER_CTOR(std::vector<uint8_t>)
INSTANTIATE_PARAM_REGISTERER_CTOR(std::vector<bool>)
INSTANTIATE_PARAM_REGISTERER_CTOR(std::vector<int64_t>)
INSTANTIATE_PARAM_REGISTERER_CTOR(std::vector<double>)
INSTANTIATE_PARAM_REGISTERER_CTOR(std::vector<std::string>)

#undef INSTANTIATE_PARAM_REGISTERER_CTOR
}  // namespace internal

template <typename ParamType>
bool ParamProvider::Get(const std::string& full_name, ParamType* value) const {
    return internal::ParamRegistry::GlobalRegistry().GetParam(module_, full_name, value);
}

template <typename ParamType>
bool ParamProvider::HasParam(const std::string& full_name) const {
    return internal::ParamRegistry::GlobalRegistry().HasParam<ParamType>(module_, full_name);
}

template <typename ParamType>
void ParamProvider::Update(const std::string& full_name, const ParamType& new_value) {
    internal::ParamRegistry::GlobalRegistry().UpdateParam(module_, full_name, new_value);
}

template <typename ParamType>
bool ParamProvider::TryUpdate(const std::string& full_name, const ParamType& new_value) {
    if (!internal::ParamRegistry::GlobalRegistry().HasParam<ParamType>(module_, full_name)) {
        return false;
    }

    internal::ParamRegistry::GlobalRegistry().UpdateParam(module_, full_name, new_value);
    return true;
}

template <typename ParamType>
internal::ParamMap<ParamType>& ParamProvider::GetParamMap() {
    return internal::ParamRegistry::GlobalRegistry().GetParamMap<ParamType>(module_);
}

// Instantiate Update, TryUpdate and GetParamMap for all supported types.
#define INSTANTIATE_PARAM_PROVIDER_FNS(type)                                                     \
    template bool ParamProvider::Get(const std::string& full_name, type* value) const;           \
    template bool ParamProvider::HasParam<type>(const std::string& full_name) const;             \
    template void ParamProvider::Update(const std::string& full_name, const type& new_value);    \
    template bool ParamProvider::TryUpdate(const std::string& full_name, const type& new_value); \
    template internal::ParamMap<type>& ParamProvider::GetParamMap<type>();

INSTANTIATE_PARAM_PROVIDER_FNS(bool)
INSTANTIATE_PARAM_PROVIDER_FNS(int64_t)
INSTANTIATE_PARAM_PROVIDER_FNS(double)
INSTANTIATE_PARAM_PROVIDER_FNS(std::string)
INSTANTIATE_PARAM_PROVIDER_FNS(std::vector<uint8_t>)
INSTANTIATE_PARAM_PROVIDER_FNS(std::vector<bool>)
INSTANTIATE_PARAM_PROVIDER_FNS(std::vector<int64_t>)
INSTANTIATE_PARAM_PROVIDER_FNS(std::vector<double>)
INSTANTIATE_PARAM_PROVIDER_FNS(std::vector<std::string>)
#undef INSTANTIATE_PARAM_PROVIDER_FNS

template <typename ParamType>
std::ostream& operator<<(std::ostream& os, const Param<ParamType>& param) {
    os << "[" << param.filename_ << "] " << param.full_name_ << " - " << param.help_;
    return os;
}
}  // namespace params
