// 单位换算常量。
// 我们在 CMakeLists 里设了 CMAKE_CXX_EXTENSIONS OFF（-std=c++17，
// 会定义 __STRICT_ANSI__），M_PI 是否可见取决于头文件包含顺序和
// 工具链实现 —— 属于"碰巧能编译"。集中定义一次，去掉这个隐患。
//
// 顺带把各文件里重复定义的 kDeg2Rad / kRad2Deg 收敛到一处。

#pragma once

namespace autoaim {

// 圆周率。自己写一份而不是用 M_PI，理由见文件头注释。
inline constexpr double kPi = 3.14159265358979323846;

// 2π（一整圈），弧度。做角度归一化（把角度折回 ±π）时用。
inline constexpr double kTwoPi = 2.0 * kPi;

// 角度 → 弧度的系数。用法：rad = deg * kDeg2Rad。
inline constexpr double kDeg2Rad = kPi / 180.0;

// 弧度 → 角度的系数。用法：deg = rad * kRad2Deg。
// 协议层发出去的角度必须用它转一下（协议要的是角度制，见 srm_protocol.h）。
inline constexpr double kRad2Deg = 180.0 / kPi;

}  // namespace autoaim
