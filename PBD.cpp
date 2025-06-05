#include "PBD.h"
#include "MassFunction.h"
void PBD::Initialize(Vector2 startPos, Vector2 endPos, int numPoints, float k, float dt, float kDamping, Vector2 gravity)
{points_;

}

void PBD::Update()
{
	//外力による加速度を計算
	for (int i = 0; i < numPoints_; i++)
	{
		points_[i].velocity += gravity_ * dt_;
		if (points_[i].isFixed)
		{
			points_[i].velocity=Vector2(0.0f,0.0f);
		}

	}
	//ベロシティ　ダンピング

	// 位置の更新
	for (int i = 0; i < numPoints_; i++)
	{
	points_[i].pPosition=points_[i].position+points_[i].velocity*dt_;
	points_[i].position = points_[i].pPosition;
	}
	points_[0].position = startPos_;
	points_[numPoints_ - 1].position = endPos_;
	//速度の更新
	for (int i = 0; i < numPoints_; i++)
	{

	}
	
}

void PBD::draw()
{
	

	for (int i = 0; i < points_.size()-1; i++)
	{
		Novice::DrawEllipse(static_cast<int> (points_[i].position.x),static_cast<int>  (points_[i].position.y), 5,5,0.0f, WHITE, kFillModeSolid);
		Novice::DrawLine(static_cast<int> (points_[i].position.x),static_cast<int>  (points_[i].position.y),static_cast<int> (points_[i-1].position.x),static_cast<int>  (points_[i-1].position.y),WHITE);


	}
	
}
