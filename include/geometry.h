#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__
#include <cmath>
#include <vector>
#include <cassert>
#include <iostream>


//模板类向量
//维度Dimension,元素类型T
template <size_t DIM, typename T> struct vec {
    vec() { for (size_t i=DIM; i--; data_[i] = T()); }
          T& operator[](const size_t i)       { assert(i<DIM); return data_[i]; }
    const T& operator[](const size_t i) const { assert(i<DIM); return data_[i]; }
private:
    T data_[DIM];
};

//类型重新定义
typedef vec<2, float> Vec2f;
typedef vec<3, float> Vec3f;
typedef vec<3, int  > Vec3i;
typedef vec<4, float> Vec4f;

template <typename T> struct vec<2,T> {
    vec() : x(T()), y(T()) {}
    vec(T X, T Y) : x(X), y(Y) {}
    //模板类型特化-构造函数
    // 跨类型的构造函数（如 vec<2, float> -> vec<2, double>）
    template <class U>
    vec(const vec<2, U>& v) : x(static_cast<T>(v.x)), y(static_cast<T>(v.y)) {}
          T& operator[](const size_t i)       { assert(i<2); return i==0 ? x : y; }
    const T& operator[](const size_t i) const { assert(i<2); return i==0 ? x : y; }
    T x,y;
};
// 当 i = 0 → i <= 0 为真 → 返回 x ✅
// 当 i = 1 → i <= 0 为假 → 返回 y ✅
// 看似正确？但注意：size_t 是无符号类型！
// ⚠️ 危险场景：如果用户传入 i = -1（虽然不合理），由于 size_t 无符号，-1 会变成一个极大正数（如 18446744073709551615），此时：
// assert(i < 2) 会触发断言失败（安全）
// 但如果关闭断言（如 Release 模式），i <= 0 对极大正数为假 → 返回 y，造成静默错误！
// ✅ 正确逻辑应为：
// cpp
// return i == 0 ? x : y;
// // 或
// return i ? y : x;
template <typename T> struct vec<3,T> {
    vec() : x(T()), y(T()), z(T()) {}
    vec(T X, T Y, T Z) : x(X), y(Y), z(Z) {}
          T& operator[](const size_t i)       { assert(i<3); return i==0 ? x : (1==i ? y : z); }
    const T& operator[](const size_t i) const { assert(i<3); return i==0 ? x : (1==i ? y : z); }
    float norm() { return std::sqrt(x*x+y*y+z*z); }
    //注意除零风险
    vec<3,T> & normalize(T l=1) { *this = (*this)*static_cast<float>(l/norm()); return *this; }
    T x,y,z;
};

template <typename T> struct vec<4,T> {
    vec() : x(T()), y(T()), z(T()), w(T()) {}
    vec(T X, T Y, T Z, T W) : x(X), y(Y), z(Z), w(W) {}
          T& operator[](const size_t i)       { assert(i<4); return i==0 ? x : (1==i ? y : (2==i ? z : w)); }
    const T& operator[](const size_t i) const { assert(i<4); return i==0 ? x : (1==i ? y : (2==i ? z : w)); }
    T x,y,z,w;
};

//运算符重载
template<size_t DIM,typename T> T operator*(const vec<DIM,T>& lhs, const vec<DIM,T>& rhs) {
    T ret = T();
    for (size_t i=DIM; i--; ret+=lhs[i]*rhs[i]);
    return ret;
}

template<size_t DIM,typename T>vec<DIM,T> operator+(vec<DIM,T> lhs, const vec<DIM,T>& rhs) {
    for (size_t i=DIM; i--; lhs[i]+=rhs[i]);
    return lhs;
}

template<size_t DIM,typename T>vec<DIM,T> operator-(vec<DIM,T> lhs, const vec<DIM,T>& rhs) {
    for (size_t i=DIM; i--; lhs[i]-=rhs[i]);
    return lhs;
}

//标量点乘向量
template<size_t DIM,typename T,typename U> vec<DIM,T> operator*(const vec<DIM,T> &lhs, const U& rhs) {
    vec<DIM,T> ret;
    for (size_t i=DIM; i--; ret[i]=lhs[i]*rhs);
    return ret;
}

//向量取反
template<size_t DIM,typename T> vec<DIM,T> operator-(const vec<DIM,T> &lhs) {
    return lhs*T(-1);
}
//向量叉乘
template <typename T> vec<3,T> cross(vec<3,T> v1, vec<3,T> v2) {
    return vec<3,T>(v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x);
}
//左输出流
template <size_t DIM, typename T> std::ostream& operator<<(std::ostream& out, const vec<DIM,T>& v) {
    for(unsigned int i=0; i<DIM; i++) {
        out << v[i] << " " ;
    }
    return out ;
}
#endif //__GEOMETRY_H__