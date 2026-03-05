#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>

#include "geometry.h"
#include "model.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// 环境贴图
int envmap_width, envmap_height;
std::vector<Vec3f> envmap;

// 加载模型
Model duck("../resource/duck.obj");
//AABB包围盒
Vec3f min, max;
// // 深度缓冲
// //  图像像数
// const int scr_width = 1024;
// const int scr_height = 768;
// double z_buffer[scr_width][scr_height] = {std::numeric_limits<double>::max()}; // 全部初始化为最大值

// 光源
struct Light
{
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;  // 光源位置
    float intensity; // 光源强度
};
// 材质
struct Material
{
    Material(const float &r, const Vec4f &a, const Vec3f &color, const float &spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
    float refractive_index;  // 折射系数
    Vec4f albedo;            // 反照率,0漫反射，1镜面反射，2反射率,3折射率
    float specular_exponent; // 镜面光照
    Vec3f diffuse_color;     // 漫反射光照颜色
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

// 反射光函数,I为入射光，N为法向量
Vec3f reflect(const Vec3f &I, const Vec3f &N)
{
    // 平行四边形法则，N为法向量I*N为入射光投影
    return I - N * 2.f * (I * N);
}
// 折射函数，I为入射光，N为法向量，折射率，实现了 基于斯涅尔定律（Snell's Law）的光线折射方向计算
Vec3f refract(const Vec3f &I, const Vec3f &N, const float &refractive_index)
{ // Snell's law
    float cosi = -std::clamp(I * N, -1.0f, 1.0f);
    // 求解法向量与入射光单位向量的cos,I向内，N向外，取负变为相同方向
    float etai = 1, etat = refractive_index; // 入射光折射率即空气折射率，出射光折射率=介质折射率
    Vec3f n = N;                             // 指向外部的法向量
    // 当 cosi < 0（即 I·N > 0），说明光线从物体内部射向外部。
    // 此时需交换入射/透射介质的折射率，并反转法线方向（使 n 始终指向入射侧）。
    if (cosi < 0)
    {                          // if the ray is inside the object, swap the indices and invert the normal to get the correct result
        cosi = -cosi;          // 取正
        std::swap(etai, etat); // 从内部发射，交换折射率
        n = -N;
    }
    // 计算折射系数和判别式
    float eta = etai / etat;                                               // 折射比
    float k = 1 - eta * eta * (1 - cosi * cosi);                           // 折射光公式
    return k < 0 ? Vec3f(0, 0, 0) : I * eta + n * (eta * cosi - sqrtf(k)); // k<0为全反射，无折射光，返回零向量（调用者需处理，如转为反射）。否则按公式计算折射方向。
    // return normalize(T); // 返回单位向量（重要！）
}

// 光线追踪aabb判断相交函数
bool intersect(const Vec3f &orig, const Vec3f &dir, Vec3f &min, Vec3f &max, float t_min, float t_max)
{
    for (int i = 0; i < 3; i++)
    {
        float invD = 1.0f / dir[i];
        float t0 = (min[i] - orig[i]) * invD;
        float t1 = (max[i] - orig[i]) * invD;
        if (invD < 0)
            std::swap(t0, t1);
        t_min = std::max(t_min, t0);
        t_max = std::min(t_max, t1);
        if (t_max <= t_min)
            return false;
    }
    return true;
}

// 场景相机射线相交判断，orig为视点，dir为方向，spheres球体数组,返回hit为交点，N为交点指向球心的方向向量
bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material, Model &model,Vec3f &min, Vec3f &max)
{
    float dist=std::numeric_limits<float>::max();// 相交点距离初始化默认最大浮点数，不用设置深度缓冲
    // float spheres_dist = std::numeric_limits<float>::max(); // 相交点距离默认最大浮点数
    for (size_t i = 0; i < spheres.size(); i++)
    {
        float dist_i;
        // 相交并且距离小于最大值（距离无穷远）
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < dist)
        {
            dist = dist_i;                     // 相交距离
            hit = orig + dir * dist_i;                 // 相交点位置向量
            N = (hit - spheres[i].center).normalize(); // 法向量：交点指向球心的方向向量
            material = spheres[i].material;            // 材质赋值
        }
    }
    // 模型加载
    float tmax = std::numeric_limits<float>::max();
    // float model_dist = std::numeric_limits<float>::max(); // 相交点距离默认最大浮点数
    // std::cout<<"qian:"<<++num0<<std::endl;
    if (intersect(orig, dir, min, max, 0.0f, tmax))
    {
        for (size_t i = 0; i < duck.nfaces(); i++)
        {
            float dist_i;
            // 相交并且距离小于最大值（距离无穷远）
            if (duck.ray_triangle_intersect(i, orig, dir, dist_i, N) && dist_i < dist)
            {
                dist = dist_i;                              // 相交距离
                hit = orig + dir * dist;                    // 相交点位置向量
                N = N.normalize();                                // 法向量单位化
                material=Material(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.); // 材质赋值
            }
        }
    }

    // 棋盘格
    //  float checkerboard_dist = 1000.0f; // 与 return 条件对齐
    // float checkerboard_dist = std::numeric_limits<float>::max(); // 检测边界距离
    if (fabs(dir.y) > 1e-3)                                      // 绝对值判断，避免除零（光线不平行于地面）
    {
        float d = -(orig.y + 4) / dir.y; // 棋盘格地面（checkerboard plane） y = -4，射线与平面 y=-4 的交点参数 d
        Vec3f pt = orig + dir * d;       // 平面方程：y = -4 → 标准形式：0*x + 1*y + 0*z + 4 = 0
        // 光线：P(d) = orig + d * dir
        // 代入：orig.y + t * dir.y = -4 → d = (-4 - orig.y) / dir.y
        // 检查交点是否在有效区：x:[-10,10],z：[-30,-10],d>0方向向前，小于球体的相机射线交点距离（被球体遮住，避免渲染遮挡的棋盘格）
        if (d > 0 && fabs(pt.x) < 10 && pt.z < -10 && pt.z > -30 && d < dist)
        {
            dist = d; // 参数t
            hit = pt;              // 交点方程
            N = Vec3f(0, 1, 0);    // 定义地面法线向上
            // 棋盘格着色：根据（x,z)坐奇偶交替着色
            // hit.x 和 hit.z 可能很大（虽然被 fabs(pt.x)<10 限制了 x，但 z 在 -30~-10）。
            // 0.5 * hit.x + 1000 → 在 x ∈ [-10,10] 时，结果 ∈ [995, 1005]
            // 0.5 * hit.z → z ∈ [-30,-10] → 结果 ∈ [-15, -5] → int(-15) = -15
            // 所以 (995~1005) + (-15~-5) = 980~1000，按位与 &1 能正常判断奇偶。
            material.diffuse_color = (int(.5 * hit.x + 1000) + int(.5 * hit.z)) & 1 ? Vec3f(1, 1, 1) : Vec3f(1, .7, .3);
            material.diffuse_color = material.diffuse_color * .3;
        }
    }
    return dist < 1000; //
}
// 光线追踪算法实现片段颜色计算，orig为相机位置或者射线出发点，dir为相机射线方向，spheres球体数组,depth递归深度
Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, size_t depth = 0,Vec3f &min=min, Vec3f &max=max)
{
    Vec3f point, N; // 交点与法向量
    Material material;
    // 如果递归深度大于四或射线不相交，设置递归深度为4
    if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material, duck,min, max))
    {
        // 将射线方向 dir 映射到球面经纬度坐标（球面图形）u为横坐标，v为纵坐标
        // phi = atan2(z, x) ∈ [-π, π] → u = (phi/(2π) + 0.5) ∈ [0,1],返回的是从点 (0,0) 到点 (z,x) 的连线与 z 轴正方向之间的夹角，结果以弧度表示，范围在 [-π, π] 之间。
        int a = std::max(0, std::min(envmap_width - 1,
                                     static_cast<int>((atan2(dir.z, dir.x) / (2 * M_PI) + 0.5) * envmap_width)));
        // theta = acos(y) ∈ [0, π] → v = theta/π ∈ [0,1]，y值要求在[-1,1],y轴到xz平面的夹角
        int b = std::max(0, std::min(envmap_height - 1,
                                     static_cast<int>(acos(dir.y) / M_PI * envmap_height)));
        return envmap[a + b * envmap_width]; // 采样环境贴图作为背景
    }
    // 反射
    Vec3f reflect_dir = reflect(dir, N).normalize();                                       // 反射光线
    Vec3f reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;        // 偏移量避免自交
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1,min,max); // 递归计算片段颜色，递归深度关系反射强度
    // 折射
    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();            // 计算折射光反向
    Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;        // 偏移量避免自交
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1,min,max); // 递归计算片段颜色，递归深度关系反射强度

    float diffuse_light_intensity = 0, specular_light_intensity = 0; // 设置漫反射与镜面反射光照强度
    for (size_t i = 0; i < lights.size(); i++)
    {
        Vec3f light_dir = (lights[i].position - point).normalize(); // 入射光方向向量
        float light_distance = (lights[i].position - point).norm(); // 光线的模即距离
        // 光线追踪（Ray Tracing）中用于生成“阴影射线起点”（shadow ray origin）的经典技巧，目的是避免自相交（self-intersection）。
        // 1. 背景：为什么需要 shadow_orig？
        // 在计算某点 point 是否被光源照亮时，我们会从该点向光源发射一条 阴影射线（shadow ray）。
        // 如果这条射线在到达光源前碰到其他物体 → 说明 point 在阴影中。
        // 问题：由于浮点精度误差，阴影射线可能立即与当前表面再次相交（即“自相交”），导致错误地认为自己在阴影里（即使没有遮挡）。
        // 2. 解决方案：沿法线方向偏移起点
        // 将射线起点从 point 沿着表面法线 N 微微移动一点（如 1e-3），使其“离开”表面。
        // 这样可避免与当前三角形/球体等自身相交。
        // 3.如果 dot(light_dir, N) < 0：
        // 说明 光源在表面背面（因为法线通常指向外，而 light_dir 指向光源）。
        // 此时应向法线反方向偏移（即 point - N * ε），因为表面“背面”才是外部空间。
        // 否则（>= 0）：
        // 光源在表面正面 → 向法线正方向偏移（point + N * ε）。
        // ❗ 但注意：light_dir 的定义方向很重要！
        // 如果 light_dir = light_pos - point（从着色点指向光源）✅ → 上述逻辑正确。
        // 4. 偏移量 1e-3 的选择
        // 太小（如 1e-10）→ 仍可能自相交（浮点误差）。
        // 太大（如 0.1）→ 可能跳过薄物体，产生“阴影泄漏”（shadow acne 或 peter panning）。
        // 经验值：1e-3 ~ 1e-4 对大多数场景有效。
        // 更高级方法：使用 ray epsilon based on geometry scale（根据场景大小动态调整）。
        Vec3f shadow_orig = light_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // 检查该点是否light[i]在阴影下面
        Vec3f shadow_pt, shadow_N;                                                   // 阴影射线（阴影点到）与球体的交点，阴影法向量
        Material tmpmaterial;                                                        // 材质
        // 阴影点位置向量shadow_orig，从point点到光源的光线方向向量light_dir，计算交点，有交点且点不在光源背后（到交点小于光线的模），即为阴影点
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial, duck,min, max) && (shadow_pt - shadow_orig).norm() < light_distance)
            continue;                                                                                                                     // 存在阴影则直接跳过漫反射与镜面高光的计算，环境光放入漫反射中
        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);                                                    // 光照强度乘以入射光向量与法向向量的内积=漫反射强度
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N) * dir), material.specular_exponent) * lights[i].intensity; // 反射光与相机射线点乘的specular_exponent的幂乘以光强
    }
    // 光线的漫反射强度（材质漫反射颜色乘以入射光向量与法向向量点乘结果）+镜面高光（白色光乘以镜面光强）,+反射光（反射颜色×反照率），albedo反照率，[0]漫反射，[1]镜面,[2]反射。[3]折射
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1] + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
}
// 渲染函数
void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights)
{
    // 图像像数
    const int width = 1024;
    const int height = 768;
    // 动态数组缓冲rgb颜色向量,像数数量为长乘宽
    std::vector<Vec3f> framebuffer(width * height);
    // field of view 视场角度
    const int fov = M_PI / 2.;
    // 获取包围盒
    duck.get_bbox(min, max);
    // 缓冲写入数据，设置相机空间到屏幕空间
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
            framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), dir, spheres, lights,0,min,max);          // 原点作为相机坐标，求球体射线相交
        }
    }
    std::ofstream ofs("../output/out.ppm", std::ios::binary);
    // 检查是否打开成功
    if (!ofs.is_open())
    {
        std::cerr << "Failed to create out.ppm\n";
        return;
    }
    ofs << "P6\n"
        << width << " " << height << "\n255\n";
    // 检查 header 是否写入成功
    if (!ofs.good())
    {
        std::cerr << "Failed to write PPM header\n";
        return;
    }
    for (size_t i = 0; i < height * width; ++i)
    {
        unsigned char pixel[3];
        // 颜色值不能超过1.0f
        float max = std::max(pixel[0], std::max(pixel[1], pixel[2]));
        if (max > 1)
        {
            pixel[0] = pixel[0] * (1. / max);
            pixel[1] = pixel[1] * (1. / max);
            pixel[2] = pixel[2] * (1. / max);
        }
        pixel[0] = (unsigned char)(255 * std::clamp(framebuffer[i][0], 0.0f, 1.0f));
        pixel[1] = (unsigned char)(255 * std::clamp(framebuffer[i][1], 0.0f, 1.0f));
        pixel[2] = (unsigned char)(255 * std::clamp(framebuffer[i][2], 0.0f, 1.0f));

        ofs.write(reinterpret_cast<const char *>(pixel), 3);
        // 检查是否写入失败
        if (!ofs.good())
        {
            std::cerr << "Write error at pixel " << i << "\n";
            break;
        }
    }
    // 检查是否关闭成功
    ofs.close();
    if (ofs.fail())
    {
        std::cerr << "Error during file close (e.g., disk full)\n";
    }
    else
    {
        std::cout << "Image successfully saved to out.ppm\n";
    }
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
    int n = -1; // 图像通道数量
    unsigned char *pixmap = stbi_load("../resource/envmap.jpg", &envmap_width, &envmap_height, &n, 0);
    if (!pixmap || 3 != n) // 检查通道数是否为三和指针是否存在
    {
        std::cerr << "Error: can not load the environment map" << std::endl;
        return -1;
    }
    envmap = std::vector<Vec3f>(envmap_width * envmap_height);
    for (int j = envmap_height - 1; j >= 0; j--) // 图形y轴向下是负数
    {
        for (int i = 0; i < envmap_width; i++)
        {
            // 加载图像到数组中
            envmap[i + j * envmap_width] = Vec3f(pixmap[(i + j * envmap_width) * 3 + 0], pixmap[(i + j * envmap_width) * 3 + 1], pixmap[(i + j * envmap_width) * 3 + 2]) * (1 / 255.);
        }
    }
    stbi_image_free(pixmap); // 释放图像空间
    // 场景设置
    // 材质类型
    Material ivory(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.);      // 有微弱反射，无折射
    Material glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);     // 玻璃材质有折射
    Material red_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.); // 无反射
    Material mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);  // 镜面高光幂指数大，漫反射白光强度也大，且反射率也大
    // 场景球体组合
    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2, glass));
    spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 3, red_rubber));
    spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, mirror));
    // 光线
    std::vector<Light> lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
    lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f(30, 20, 30), 1.7));
    // 移动模型

    // 渲染函数
    render(spheres, lights);
    std::cout << "end" << std::endl;
    return 0;
}
