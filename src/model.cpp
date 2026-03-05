#include <iostream>
#include <cassert>
#include <fstream>
#include <sstream>
#include "model.h" // Vec3f（3D float 向量）、Vec3i（3D int 向量）等类型在此头文件中定义

// 构造函数：从 Wavefront .obj 文件加载模型
// 注意：仅支持格式为 "v x y z" 和 "f v1 v2 v3" 的简单 obj 文件（无纹理/法线索引，即不含斜杠 /）
Model::Model(const char *filename) : verts(), faces()
{
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return; // 打开失败则提前返回，verts 和 faces 保持为空
    }

    std::string line;
    // 逐行读取 obj 文件（注意：while(!eof()) 虽非最佳实践，但配合 getline 通常可正确处理最后一行）
    while (!in.eof())//不为文件结尾
    {
        std::getline(in, line);
        std::istringstream iss(line.c_str()); // 将当前行转为字符串流，便于解析
        char trash;                           // 用于临时读取 'v' 或 'f' 字符

        // 判断是否为顶点行（以 "v " 开头，注意空格以避免匹配 "vn" 等）
        if (!line.compare(0, 2, "v "))
        {
            iss >> trash; // 读掉 'v'
            Vec3f v;
            for (int i = 0; i < 3; i++)
            {
                iss >> v[i]; // 依次读取 x, y, z 坐标
            }
            verts.push_back(v); // 存入顶点列表
        }
        // 判断是否为面行（以 "f " 开头）
        else if (!line.compare(0, 2, "f "))
        {
            Vec3i f; // 用于存储三角形的三个顶点索引（0-based）
            int idx, cnt = 0;//三角形的索引和数量
            iss >> trash; // 读掉 'f'
            // 逐个读取面中的顶点索引（只处理前三个，确保是三角形）
            while (iss >> idx && cnt < 3)
            {
                idx--; // .obj 文件索引从 1 开始，此处转换为 0-based
                f[cnt++] = idx;
            }
            if (cnt == 3)
            { // 只有恰好三个索引才视为有效三角形
                faces.push_back(f);
            }
        }
        // 其他行（如 vn, vt, g, mtllib 等）被忽略
    }

    // 输出加载统计信息
    std::cerr << "# v# " << verts.size() << " f# " << faces.size() << std::endl;

    // 计算并输出模型的轴对齐包围盒（AABB）
    Vec3f min, max;
    get_bbox(min, max);
}

// 使用 Möller–Trumbore 算法检测射线与指定三角形面的相交
// 参数：
//   fi     —— 面索引
//   orig   —— 射线起点
//   dir    —— 射线方向（无需归一化）
//   tnear  —— 输出：交点沿射线的距离（t 值）
// 返回值：true 表示相交且交点在射线正方向上
bool Model::ray_triangle_intersect(const int &fi, const Vec3f &orig, const Vec3f &dir, float &tnear,Vec3f &N)
{
    // 获取三角形三个顶点的世界坐标
    Vec3f v0 = point(vert(fi, 0));
    Vec3f v1 = point(vert(fi, 1));
    Vec3f v2 = point(vert(fi, 2));

    // 计算两条边向量
    Vec3f edge1 = v1 - v0;
    Vec3f edge2 = v2 - v0;

    // P = dir × edge2，叉乘，辅助标量
    Vec3f pvec = cross(dir, edge2);
    // det = edge1 · P （若 det ≈ 0，说明射线与三角形平行），det（先点乘在叉乘）的数学含有为平行六面体的体积，等于零说明三个向量共平面
    float det = edge1 * pvec; // 点积 

    // 若 det 太小（包括负值）-浮点精度问题，认为不相交或背面（此实现包含背面），剔除背面Backface Culling 即只接受 det > 0（正面）
    if (det < 1e-5f)
        return false;

    // T = orig - v0
    Vec3f tvec = orig - v0;
    // u = T · P （未归一化的重心坐标 u）归一化为u = T · P/det
    float u = tvec * pvec;
    // 检查 u 是否在 [0, det] 范围内 交点在三角形内则标准化u在[0,1]
    if (u < 0 || u > det)
        return false;
    
    // Q = T × edge1
    Vec3f qvec = cross(tvec, edge1);
    // v = dir · Q （未归一化的重心坐标 v），归一化为v = dir · Q/det
    float v = dir * qvec;
    // 检查 v ≥ 0 且 u + v ≤ det，交点在三角形内则标准化u+v<=1,重心坐标的性质
    if (v < 0 || u + v > det)
        return false;

    // 计算交点距离 t = (edge2 · Q) / det
    tnear = (edge2 * qvec) * (1.0f / det);

    //法向量N
    N=cross(edge1, edge2);
    // 忽略太近的交点（防止自相交，如浮点误差导致的表面内部相交）
    return tnear > 1e-5f;
}

// 返回顶点数量
int Model::nverts() const
{
    return (int)verts.size();
}

// 返回三角形面数量
int Model::nfaces() const
{
    return (int)faces.size();
}

// 计算模型的轴对齐包围盒（AABB）
void Model::get_bbox(Vec3f &min, Vec3f &max)
{
    if (verts.empty())
    {
        // 安全处理：空模型
        min = max = Vec3f(0, 0, 0);
        return;
    }
    //初始化为顶点零
    min = max = verts[0];
    for (int i = 1; i < (int)verts.size(); ++i)
    {
        for (int j = 0; j < 3; j++)
        {
            //取向量空间的最大最小值
            min[j] = std::min(min[j], verts[i][j]);
            max[j] = std::max(max[j], verts[i][j]);
        }
    }
    //输出包围盒最大最小值向量
    std::cerr << "bbox: [" << min << " : " << max << "]" << std::endl;
}

// 返回第 i 个顶点（const 版本，只读）
const Vec3f &Model::point(int i) const
{
    //断言顶点索引范围，从零开始
    assert(i >= 0 && i < nverts());
    return verts[i];
}

// 返回第 i 个顶点（非 const 版本，可修改）
Vec3f &Model::point(int i)
{
    assert(i >= 0 && i < nverts());
    return verts[i];
}

// 返回面 fi 的第 li 个顶点的索引（0, 1, 或 2）
int Model::vert(int fi, int li) const
{
    assert(fi >= 0 && fi < nfaces() && li >= 0 && li < 3);
    return faces[fi][li];
}

// 重载 << 运算符，将模型以 .obj 格式输出到 ostream
// 用于保存或调试模型数据
std::ostream &operator<<(std::ostream &out, Model &m)
{
    // 输出所有顶点（格式：v x y z），注意空格
    for (int i = 0; i < m.nverts(); i++)
    {
        out << "v " << m.point(i) << std::endl;
    }
    // 输出所有面（格式：f v1 v2 v3，索引转回 1-based）
    for (int i = 0; i < m.nfaces(); i++)
    {
        out << "f ";
        for (int k = 0; k < 3; k++)
        {
            out << (m.vert(i, k) + 1) << " "; // +1 因为 .obj 索引从 1 开始
        }
        out << std::endl;
    }
    return out;
}