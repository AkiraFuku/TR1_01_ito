#include "PBD.h"
#include "MassFunction.h"
void PBD::Initialize(Vector2 startPos, Vector2 endPos, int numPoints, float k, float dt, float kDamping, Vector2 gravity)
{
  points_;

}

void PBD::Update()
{
	//外力による加速度を計算
	for (int i = 0; i < numPoints_; i++)
	{
		points_[i].velocity += gravity_ * dt_;
		if (points_[i].isFixed)
		{
			points_[i].velocity = Vector2(0.0f, 0.0f);
		}

	}
	//ベロシティ　ダンピング

	// 位置の更新
	for (int i = 0; i < numPoints_; i++)
	{
		points_[i].pPosition = points_[i].position + points_[i].velocity * dt_;
		points_[i].position = points_[i].pPosition;
	}
	points_[0].position = startPos_;
	points_[numPoints_ - 1].position = endPos_;
	//速度の更新
	for (int i = 0; i < constraints_.size(); i++)
	{
		Constraint c= constraints_[i];
		Points p1 = points_[c.i];
		Points p2 = points_[c.j];
		float w1 = 1.0f / p1.mass;// 質量の逆数を計算
		float w2 = 1.0f / p2.mass;// 質量の逆数を計算

		float diff = Length(p1.position - p2.position);
		Vector2 dp1 =Multiply( (-k_ * w1 / (w1 + w2) * (diff - c.d) ),Normalize(p1.position - p2.position));
		Vector2 dp2 = Multiply((k_ * w2 / (w1 + w2) * (diff - c.d)), Normalize(p1.position - p2.position));


		points_[c.i].velocity += dp1 / dt_;
		points_[c.j].velocity += dp2 / dt_;
	}

}

void PBD::draw()
{


	for (int i = 0; i < points_.size() - 1; i++)
	{
		Novice::DrawEllipse(static_cast<int>(points_[i].position.x), static_cast<int>(points_[i].position.y), 5, 5, 0.0f, WHITE, kFillModeSolid);
		Novice::DrawLine(static_cast<int>(points_[i].position.x), static_cast<int>(points_[i].position.y), static_cast<int>(points_[i - 1].position.x), static_cast<int>(points_[i - 1].position.y), WHITE);


	}

}

void PBD::VelocityDamping()  
{  
    Vector2 xcm = Vector2(0.0f, 0.0f);  
    Vector2 vcm = Vector2(0.0f, 0.0f);  
    float totalMass = 0.0f;  
    for (int i = 0; i < numPoints_; i++)  
    {  
        xcm += points_[i].position;  
        vcm += points_[i].velocity;  
        totalMass += points_[i].mass;  
    }  
    xcm /= totalMass;  
    vcm /= totalMass;  
    Vector2 l = Vector2(0.0f, 0.0f);  
    Matrix3x3 i;  
    std::vector<Vector2> rs(numPoints_); 
	for (int i = 0; i < numPoints_; i++)  
	{  
		rs[i] = points_[i].position - xcm;  
		

	}
}
