#ifndef ROSMATH_TEMPLATE_H
#define ROSMATH_TEMPLATE_H

#include <tuple>
#include <type_traits>

namespace rosmath {

template <typename T, typename Tuple>
struct has_type;

template <typename T>
struct has_type<T, std::tuple<>> : std::false_type {};

template <typename T, typename U, typename... Ts>
struct has_type<T, std::tuple<U, Ts...>> : has_type<T, std::tuple<Ts...>> {};

template <typename T, typename... Ts>
struct has_type<T, std::tuple<T, Ts...>> : std::true_type {};

template <typename T, typename Tuple>
using tuple_contains_type = typename has_type<T, Tuple>::type;

template<typename T, typename Tuple>
using TupleEnabler = typename std::enable_if<tuple_contains_type<T, Tuple>::value, int>;

template<typename T, typename Tuple>
using TupleDisabler = typename std::enable_if<!tuple_contains_type<T, Tuple>::value, int>;


} // namespace rosmath

#endif // ROSMATH_TEMPLATE_H