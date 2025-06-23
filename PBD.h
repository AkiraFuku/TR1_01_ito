#pragma once
#include <KamataEngine.h>
#include <Novice.h>
#include <vector>
using namespace KamataEngine;

class PBD
{
public:
    struct Points
    {
        // 質点の位置
        Vector3 position;
        // 質点の推定位置
        Vector3 estimationPosition;
        Vector3 velocity; // 質点の速度
        float mass;       // 質点の質量
        bool isFixed;
    };

    struct Constraint {
        int prevIndex; // ひとつ前の質点
        int nextIndex; // ひとつ次の質点
        float distance;
    };

    void Initialize(Vector3 startPos, Vector3 endPos, int numPoints, Vector3 gravity);
    void Update();
    void Draw(const Matrix4x4& viewProjectionMatrix,
    const Matrix4x4& viewportMatrix);

    void SetEndPos(Vector3 endPos) {
        endPos_ = endPos;
    }
    void SetStartPos(Vector3 startPos) {
        startPos_ = startPos;
    }
    

    
    void SetMass(float mass) {
        for (int i = 0; i < points_.size(); i++) {
            points_[i].mass = mass;
        }
    }

private:
    Vector3 startPos_; // パスの開始位置
    Vector3 endPos_;   // パスの終了位置
    int numPoints_;    // パーティクルの数
   const float springStiffness_= 0.1f; 
   const float dt_ =0.01f / 60.f; 
    
    Vector3 gravity_;
    std::vector<Points> points_;
    std::vector<Constraint> constraints_;
};

