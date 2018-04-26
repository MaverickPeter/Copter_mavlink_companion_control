//
// Created by 崔瑜翔 on 2018/4/3.
//

#ifndef MARKER_AR_GEOMETRYTYPES_H
#define MARKER_AR_GEOMETRYTYPES_H
struct Matrix44
{
    union//共用体和结构体十分相似，几个变量共用一个内存位置，在不同的时间保存不同的数据类型和不同长度的变量。同一时间只能储存其中一个成员变量的值。其长度为联合中最大的变量长度的整数倍。
    {
        float data[16];
        float mat[4][4];
    };

    Matrix44 getTransposed() const;//对角线相互交换
    Matrix44 getInvertedRT() const;//取反转置
    static Matrix44 identity();//得到单位矩阵
};

struct Matrix33
{
    union
    {
        float data[9];
        float mat[3][3];
    };

    static Matrix33 identity();//得到单位矩阵
    Matrix33 getTransposed() const;//对角线互换
};

struct Vector4
{
    float data[4];
};

struct Vector3
{
    float data[3];

    static Vector3 zero();//零向量
    Vector3 operator-() const;//取反
};

struct Transformation
{
    Transformation();
    Transformation(const Matrix33& r, const Vector3& t);

    Matrix33& r();
    Vector3&  t();

    const Matrix33& r() const;
    const Vector3&  t() const;

    Matrix44 getMat44() const;

    Transformation getInverted() const;
private:
    Matrix33 m_rotation;
    Vector3  m_translation;
};

#endif //MARKER_AR_GEOMETRYTYPES_H
