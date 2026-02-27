#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"
#include <algorithm>

//光源
struct Light {
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;//光源位置
    float intensity;//光源强度
};
// 材质
struct Material
{
    Material(const Vec3f &color) : diffuse_color(color) {}
    Material() : diffuse_color() {}
    Vec3f diffuse_color; // 材质漫反射颜色
};

// 球体
struct Sphere
{
    Vec3f center;      // 球中心
    float radius;      // 半径
    Material material; // 球体材质
    // 球体构造函数
    Sphere(const Vec3f &c, const float &r, const Material &m) : center(c), radius(r), material(m) {}
    // 光线求交，分几种情况，起点在球体内外，方向在圆心正反方向，d与r的判别
    // orig为射线起点，dir为射线方向，t0为求解得到射线到球体的距离
    // 图解链接https://user-images.githubusercontent.com/26228275/52620952-0c88aa80-2ecc-11e9-8917-e7fc438e3536.png
    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const
    {
        Vec3f L = center - orig;      // L为射线起点到圆心向量
        float tca = L * dir;          // dir为单位向量，点乘结果为L在设想dir方向上的投影，可以为负值，即射线起点到垂足的距离
        float d2 = L * L - tca * tca; // 圆心到射线的距离的平方，正数
        if (d2 > radius * radius)
            return false;                        // 若大于半径平方则不相交
        float thc = sqrtf(radius * radius - d2); // 开方浮点数，垂足到圆周的距离，正数
        t0 = tca - thc;                          // 若为正数，射线到球表面的的距离
        float t1 = tca + thc;                    // 若为正数，射线穿过圆周背面的距离
        if (t0 < 0)                              // 射线往球面相反方向发射,可能在球里面（t0为负数，t1为正数），球外面且射线相反（t0为负数，t1为负数）。
            t0 = t1;                             // 射线到球面的负距离（正数在球内，负数在球外）
        if (t0 < 0)
            return false; // 不相交

        return true; // 相交
    }
};

// 场景球体相交判断，orig为视点，dir为方向，spheres球体数组,hit为交点，N为交点指向球心的方向向量
bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material)
{
    float spheres_dist = std::numeric_limits<float>::max(); // 相交点距离默认最大浮点数
    for (size_t i = 0; i < spheres.size(); i++)
    {
        float dist_i;
        // 相交并且距离小于最大值
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist)
        {
            spheres_dist = dist_i;                     // 相交距离
            hit = orig + dir * dist_i;                 // 相交点向量
            N = (hit - spheres[i].center).normalize(); // 交点指向球心的方向向量
            material = spheres[i].material;            // 材质赋值
        }
    }
    return spheres_dist < 1000;
}
// 片段颜色计算函数，orig为视点，dir为方向，spheres球体数组
Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights) 
{
    Vec3f point, N;//交点与法向量
    Material material;
  
    if (!scene_intersect(orig, dir, spheres, point, N, material))
    {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }
    float diffuse_light_intensity = 0;
    for (size_t i=0; i<lights.size(); i++) {
        Vec3f light_dir      = (lights[i].position - point).normalize();//入射光方向向量
        diffuse_light_intensity  += lights[i].intensity * std::max(0.f, light_dir*N);//光照强度乘以入射光向量与法向向量的内积=漫反射强度
    }
    return material.diffuse_color * diffuse_light_intensity;//光线的漫反射强度（材质颜色乘以入射光向量与法向向量点乘结果）
}
//渲染函数
void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights) 
{
    // 图像像数
    const int width = 1024;
    const int height = 768;
    // 动态数组缓冲rgb颜色向量,像数数量为长乘宽
    std::vector<Vec3f> framebuffer(width * height);
    // field of view 视场角度
    const int fov = M_PI / 2.;
    // 缓冲写入数据
    for (size_t j = 0; j < height; j++)
    {
        for (size_t i = 0; i < width; i++)
        {
            // 世界坐标转化为相机坐标（屏幕坐标）
            // 定义一个单位距离即世界空间的距离（-1为屏幕所在位置），求得像素长度为2*tan(fov/2)/width，像素中心为像素点的位置，遍历方向向量求解每个像素的颜色
            // 屏幕上的width个像素对应2 * tan(fov/2)个世界单位。因此，向量的顶点位于距离左边缘(i+0.5)/width * 2*tan(fov/2)个世界单位的位置
            // 或者从屏幕与-z轴的交点起距离为(i+0.5)/width*2*tan(fov/2)-tan(fov/2)。将屏幕的宽高比加入计算中，你就会找到射线方向的精确公式
            float x = (2 * (i + 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float)height; // 标准化成1x1x1的立方体，需要乘以宽高比
            float y = -(2 * (j + 0.5) / (float)height - 1) * tan(fov / 2.);                       // 高度为单位长度进行缩放，屏幕空间y轴向下，取负值
            Vec3f dir = Vec3f(x, y, -1).normalize();                                              // 相机方向向量，-1为屏幕所在位置，标准化方向向量
            framebuffer[i+j*width] = cast_ray(Vec3f(0,0,0), dir, spheres, lights);                 // 原点作为相机坐标，求球体射线相交
        }
    }
    std::ofstream ofs("out.ppm", std::ios::binary);
    ofs << "P6\n"
        << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i)
    {
        unsigned char pixel[3];
        pixel[0] = (unsigned char)(255 * std::clamp(framebuffer[i][0], 0.0f, 1.0f)); // R
        pixel[1] = (unsigned char)(255 * std::clamp(framebuffer[i][1], 0.0f, 1.0f)); // G
        pixel[2] = (unsigned char)(255 * std::clamp(framebuffer[i][2], 0.0f, 1.0f)); // B
        ofs.write(reinterpret_cast<const char *>(pixel), 3);
    }
    ofs.close();
}
void render()
{
    // 图像像数
    const int width = 1024;
    const int height = 768;
    // 动态数组缓冲rgb颜色向量,像数数量为长乘宽
    std::vector<Vec3f> framebuffer(width * height);
    // 缓冲写入数据
    for (size_t j = 0; j < height; j++)
    {
        for (size_t i = 0; i < width; i++)
        {
            // 一维数组模拟矩阵输入rgb分量
            framebuffer[i + j * width] = Vec3f(j / float(height), i / float(width), 0.5f);
        }
    }
    // 文件输出流，将帧缓冲区（framebuffer）保存为 PPM 图像文件（一种简单的无压缩位图格式）
    //     ✅ 1. PPM 格式简介（P6 类型）
    // P6：表示二进制 PPM（Portable PixMap）
    // 接下来一行：宽度 高度
    // 再下一行：最大颜色值（通常是 255）
    // 之后是原始像素数据：每个像素按 R G B 顺序，每个通道 1 字节（unsigned char）
    // std::ofstream ofs; // save the framebuffer to file
    // ofs.open("./out.ppm");
    // ofs << "P6\n"
    //     << width << " " << height << "\n255\n";

    // 使用二进制模式写入像数分量，文件才不会损坏
    std::ofstream ofs("out.ppm", std::ios::binary);
    ofs << "P6\n"
        << width << " " << height << "\n255\n";

    // ❌ 问题：使用 ofs << (char)... 在 非二进制模式 下写入
    // 这是文本输出操作，即使你写的是 (char)，C++ 的 operator<< 对 char 会当作字符输出（比如值为 10 会被写成换行符 \n），而不是原始字节！
    // 这会导致：
    // 颜色值 10 → 写成 \n（换行）
    // 颜色值 13 → 写成 \r（回车）
    // 文件中混入大量控制字符 → PPM 解析失败
    // 🔥 即使你加了 std::ios::binary，operator << 仍然是格式化文本输出，不会变成二进制写入！
    // for (size_t i = 0; i < height * width; ++i)
    // {
    //     for (size_t j = 0; j < 3; j++)
    //     {
    //         ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
    //     }
    // }
    // 写入图像文件
    for (size_t i = 0; i < height * width; ++i)
    {
        unsigned char pixel[3];
        pixel[0] = (unsigned char)(255 * std::clamp(framebuffer[i][0], 0.0f, 1.0f)); // R
        pixel[1] = (unsigned char)(255 * std::clamp(framebuffer[i][1], 0.0f, 1.0f)); // G
        pixel[2] = (unsigned char)(255 * std::clamp(framebuffer[i][2], 0.0f, 1.0f)); // B
        ofs.write(reinterpret_cast<const char *>(pixel), 3);
    }
    ofs.close();
}

/// @brief 主函数入口
/// @return
int main()
{
    // 场景设置
    // 材质类型
    Material ivory(Vec3f(0.4, 0.4, 0.3));
    Material red_rubber(Vec3f(0.3, 0.1, 0.1));
    // 场景球体组合
    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2, red_rubber));
    spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 3, red_rubber));
    spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, ivory));  
    //光线
    std::vector<Light>  lights;
    lights.push_back(Light(Vec3f(-20, 20,  20), 1.5));
    // 渲染函数
    render(spheres, lights);

    return 0;
}
