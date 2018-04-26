//
// Created by 崔瑜翔 on 2018/4/3.
//

#include "GeometryTypes.h"

Matrix44 Matrix44::getTransposed() const//行列值交换，对角变换
{
    Matrix44 t;

    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            t.mat[i][j] = mat[j][i];

    return t;
}

Matrix44 Matrix44::identity()  //只保留4x4矩阵的对角线元素
{
    Matrix44 eye;

    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            eye.mat[i][j] = i==j?1:0;

    return eye;
}

Matrix44 Matrix44::getInvertedRT() const //对矩阵求反转置
{
    Matrix44 t = identity();//单位矩阵

    for(int col=0;col<3;col++)
    {
        for(int row=0;row<3;row++)
        {
            //Transpose rotation component
            t.mat[row][col] = -t.mat[col][row];//by zhuzhu
        }

        t.mat[3][col] = -t.mat[3][col];

    }
    return t;
}

Matrix33 Matrix33::identity()//得到3x3的对角单位矩阵
{
    Matrix33 eye;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            eye.mat[i][j] = i==j?1:0;

    return eye;
}

Matrix33 Matrix33::getTransposed() const//对角线对换
{
    Matrix33 t;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            t.mat[i][j] = mat[j][i];

    return t;
}

Vector3 Vector3::zero()
{
    Vector3 v = {0,0,0};
    return v;
}

Vector3 Vector3::operator-() const
{
    Vector3 v = { -data[0],-data[1],-data[2] };
    return v;
}

Transformation::Transformation()
        :m_rotation(Matrix33::identity())
        ,m_translation(Vector3::zero())
{

}

Transformation::Transformation(const Matrix33& r,const Vector3& t)
        :m_rotation(r)
        ,m_translation(t)
{

}

Matrix33& Transformation::r()
{
    return m_rotation;
}

Vector3&  Transformation::t()
{
    return  m_translation;
}

const Matrix33& Transformation::r() const
{
    return m_rotation;
}

const Vector3&  Transformation::t() const
{
    return  m_translation;
}

Matrix44 Transformation::getMat44() const//就是把3x3矩阵和3元素的向量放在4x4矩阵中;最后一列是单位向量
{
    Matrix44 res = Matrix44::identity();

    for(int col=0;col<3;col++)
    {
        for(int row=0;row<3;row++)
        {

            res.mat[row][col] = m_rotation.mat[row][col];
        }

        //copy translation component
        res.mat[3][col] = m_translation.data[col];
    }

    return res;
}

Transformation Transformation::getInverted() const//3x3矩阵转置，向量值取反
{
    return Transformation(m_rotation.getTransposed(), -m_translation);
}
