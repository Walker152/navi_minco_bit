#pragma once

#include <functional>
#include <memory>
#include <vector>

namespace sheriff_os {
enum class CallbackResult { kKeep, kErase };

constexpr struct OneShotT { explicit OneShotT() = default; } kOneShot;

enum class NShot_t : size_t {};

namespace details {

template <class Self>
std::shared_ptr<Self> lock_if_weak(std::weak_ptr<Self> const& self) {
  return self.lock();
}

template <class Self>
Self const& lock_if_weak(Self const& self) {
  return self;
}

template <class Self, class MemFn>
auto bind(Self self, MemFn memfn, OneShotT) {
  return [self = std::move(self), memfn](auto... t) {  // define t
    auto const& ptr = details::lock_if_weak(self);
    if (ptr == nullptr) {
      return CallbackResult::kErase;
    }
    ((*self).*memfn)(t...);  // use t
    return CallbackResult::kErase;
  };
}

template <class Self, class MemFn>
auto bind(Self self, MemFn memfn, NShot_t n) {
  return [self = std::move(self), memfn,
          n = static_cast<size_t>(n)](auto... t) mutable {  // define t
    if (n == 0) {
      return CallbackResult::kErase;
    }
    auto const& ptr = details::lock_if_weak(self);
    if (ptr == nullptr) {
      return CallbackResult::kErase;
    }
    ((*ptr).*memfn)(t...);  // use t
    --n;
    if (n == 0) {
      return CallbackResult::kErase;
    }
    return CallbackResult::kKeep;
  };
}

template <class Self, class MemFn>
auto bind(Self self, MemFn mem_fn) {
  return [self = std::move(self), mem_fn](auto... t) {  // define t
    auto const& ptr = details::lock_if_weak(self);
    if (ptr == nullptr) {
      return CallbackResult::kErase;
    }
    ((*ptr).*mem_fn)(t...);  // use t
    return CallbackResult::kKeep;
  };
}

}  // namespace details

template <class... T>
struct Signal {
 private:
#if __cpp_lib_move_only_function
  using Functor = std::move_only_function<void(T...)>;  // use T
#else
  using Functor = std::function<CallbackResult(T...)>;
#endif

  std::vector<Functor> m_callbacks_;

 public:
#if __cpp_if_constexpr
  template <class Func>
  void connect(Func callback) {
    if constexpr (std::is_invocable_r_v<CallbackResult, Func, T...>) {
      m_callbacks_.push_back(std::move(callback));
    } else {
      m_callbacks_.push_back([callback = std::move(callback)](T... t) mutable {
        callback(std::forward<T>(t)...);
        return CallbackResult::kKeep;
      });
    }
  }
#else
  template <class Func, typename std::enable_if<
                            std::is_convertible<decltype(std::declval<Func>()(
                                                    std::declval<T>()...)),
                                                CallbackResult>::value,
                            int>::type = 0>
  void connect(Func callback) {
    m_callbacks.push_back(std::move(callback));
  }

  template <class Func,
            typename std::enable_if<std::is_void<decltype(std::declval<Func>()(
                                        std::declval<T>()...))>::value,
                                    int>::type = 0>
  void connect(Func callback) {
    m_callbacks.push_back(
        [callback = std::move(callback)](T... t) mutable {  // define t
          callback(std::forward<T>(t)...);                  // use t
          return CallbackResult::Keep;
        });
  }
#endif

  template <class Self, class MemFn, class... Tag>
  void connect(Self self, MemFn memfn, Tag... tag) {
    m_callbacks_.push_back(details::bind(std::move(self), memfn, tag...));
  }

  void emit(T... t) {
    for (auto it = m_callbacks_.begin(); it != m_callbacks_.end();) {
      CallbackResult res = (*it)(t...);
      switch (res) {
        case CallbackResult::kKeep:
          ++it;
          break;
        case CallbackResult::kErase:
          it = m_callbacks_.erase(it);
          break;
      };
    }
  }
};

#if __cplusplus >= 202002L && \
    !(defined(_MSC_VER) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL))
#define SheriffOsSignalWarp(_fun, ...)                                         \
  [=](auto&&... __t) {                                                         \
    return _fun(__VA_ARGS__ __VA_OPT__(, ) std::forward<decltype(_t)>(_t)...); \
  }
#else
#define SheriffOsSignalWarp(__fun) \
  [=](auto&&... _t) { return __fun(std::forward<decltype(_t)>(_t)...); }
#endif

#if __cplusplus >= 202002L && \
    !(defined(_MSC_VER) && (!defined(_MSVC_TRADITIONAL) || _MSVC_TRADITIONAL))
#define SheriffOsSignalWarpRef(_fun, ...)                                      \
  [=](auto&&... __t) {                                                         \
    return _fun(__VA_ARGS__ __VA_OPT__(, ) std::forward<decltype(_t)>(_t)...); \
  }
#else
#define SheriffOsSignalWarpRef(__fun) \
  [&](auto&&... _t) { return __fun(std::forward<decltype(_t)>(_t)...); }
#endif

}  // namespace sheriff_os
