#ifndef __MODEL_H__
#define __MODEL_H__
#include <vector>
#include <string>
#include "geometry.h"

class Model {
private:
    std::vector<Vec3f> verts;//顶点
    std::vector<Vec3i> faces;//三角网格面
public:
    Model(const char *filename);

    int nverts() const;                          // 顶点数量
    int nfaces() const;                          // 三角形网格数量

    bool ray_triangle_intersect(const int &fi, const Vec3f &orig, const Vec3f &dir, float &tnear,Vec3f &N); //三角网格光线相交函数

    const Vec3f &point(int i) const;                   // coordinates of the vertex i
    Vec3f &point(int i);                   // coordinates of the vertex i 顶点i的坐标
    int vert(int fi, int li) const;              //返回面 fi 的第 li 个顶点， 三角形fi的顶点索引和局部索引li index of the vertex for the triangle fi and local index li
    void get_bbox(Vec3f &min, Vec3f &max); // 所有顶点的边界盒，包括孤立顶点
};

std::ostream& operator<<(std::ostream& out, Model &m);//运算符重载

#endif //__MODEL_H__

