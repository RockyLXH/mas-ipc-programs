#ifndef TYPES_ALIAS_H
#define TYPES_ALIAS_H

/**
 * @file types_alias.h
 * @brief 简短类型别名：
 *        整数  u8/u16/u32/u64、i8/i16/i32/i64
 *        浮点  f32/f64/f80（约定命名）、r32/r64（real 命名）
 *
 * 用法：
 *   #include "types_alias.h"
 *   u32  value = 100;
 *   i16  delta = -5;
 *   f32  x     = 1.5f;
 *   f64  y     = 3.141592653589793;
 *
 * 注意：这些名字不是 C++ 标准的一部分，仅在本项目内使用。
 */

#include <cstdint>
#include <climits>
#include <cfloat>

// ---- 平台能力检查 ----------------------------------------------------------
#if !defined(INT8_MAX) || !defined(INT16_MAX) || \
    !defined(INT32_MAX) || !defined(INT64_MAX)
#error "此平台缺少 8/16/32/64 位定宽整数类型，无法使用 types_alias.h"
#endif

// ============================================================================
// 整数别名
// ============================================================================

// ---- 无符号 ----------------------------------------------------------------
using u8 = std::uint8_t;     ///< 无符号 8 位整数
using u16 = std::uint16_t;   ///< 无符号 16 位整数
using u32 = std::uint32_t;   ///< 无符号 32 位整数
using u64 = std::uint64_t;   ///< 无符号 64 位整数
using umax = std::uintmax_t; ///< 最大位宽无符号整数

// ---- 有符号 ----------------------------------------------------------------
using i8 = std::int8_t;     ///< 有符号 8 位整数
using i16 = std::int16_t;   ///< 有符号 16 位整数
using i32 = std::int32_t;   ///< 有符号 32 位整数
using i64 = std::int64_t;   ///< 有符号 64 位整数
using imax = std::intmax_t; ///< 最大位宽有符号整数

// ---- 指针宽度（可选） ------------------------------------------------------
#if defined(UINTPTR_MAX)
using uptr = std::uintptr_t;
#endif
#if defined(INTPTR_MAX)
using iptr = std::intptr_t;
#endif

// ============================================================================
// 浮点别名
// ============================================================================
//
// 说明：C++ 标准**不保证** float=32bit、double=64bit。
// 这里的 f32/f64 是"约定命名"——在现代主流平台（x86/x64/ARM/RISC-V、
// IEEE 754 环境）上成立。下方提供编译期检查以便发现异常平台。
//
// 如果你的项目需要严格保证位宽，请在构建时依赖 <static_assert>，
// 而不是运行期判断。

using f32 = float;       ///< 通常为 IEEE 754 单精度（32 位）
using f64 = double;      ///< 通常为 IEEE 754 双精度（64 位）
using f80 = long double; ///< 扩展精度，位宽依平台而定（x86 上 80 位）

// 与"real"命名等价的别名（有些代码库更喜欢 r32/r64 风格）
using r32 = float;
using r64 = double;

// ---- 编译期提醒（不阻断编译，但能提前发现异常平台） ------------------------
// 若某个平台不满足约定，编译期会在这里报错。如果你确定要在
// 非 IEEE 754 平台上继续使用，可把下面这段整体注释掉。

static_assert(sizeof(float) == 4, "f32 假设 float 为 32 位，本平台不满足");
static_assert(sizeof(double) == 8, "f64 假设 double 为 64 位，本平台不满足");

// 进一步确认浮点模型（可选，需要 C++23 的 <limits> 常量时更严格）：
// static_assert(std::numeric_limits<float>::is_iec559,  "f32 假设 IEEE 754");
// static_assert(std::numeric_limits<double>::is_iec559, "f64 假设 IEEE 754");

// ---- 浮点位宽常量（便于书写） ---------------------------------------------
inline constexpr int f32_bits = static_cast<int>(sizeof(float) * CHAR_BIT);
inline constexpr int f64_bits = static_cast<int>(sizeof(double) * CHAR_BIT);
inline constexpr int f80_bits = static_cast<int>(sizeof(long double) * CHAR_BIT);

#endif // TYPES_ALIAS_H