#include "PBDMesh.h"
#include "MassFunction.h"
void nuno::Initialize(Vector2 startPos, Vector2 endPos, int numPoints, Vector2 gravity)
{

	points_.clear();
	constraints_.clear();
	startPos_ = startPos;// 開始位置を設定
	endPos_ = endPos;// 終了位置を設定
	numPoints_ = numPoints;// 点の数を設定
	gravity_ = gravity;// 重力を設定

	if (numPoints_ < 2) return; // 2点未満は無効

	points_.resize(numPoints_);// 点の数だけポイントを確保
	constraints_.resize(numPoints_ - 1);// 制約の数は点の数 - 1

	for (int i = 0; i < numPoints_; i++) {

		float t = static_cast<float>(i) / static_cast<float>((numPoints_ - 1));



		points_[i].position = Lerp(startPos_, endPos_, t);// 初期位置を線形補間で設定

		points_[i].estimationPosition = points_[i].position; // 初期位置を設定	
		points_[i].velocity = Vector2(0.0f, 0.0f);// 初期速度をゼロに設定
		points_[i].mass = 1.0f;// 質量を1.0に設定

	}
	points_[0][0].isFixed = true;
	//points_[1].isFixed = true;
	points_[numPoints_ - 1][numPoints_-1].isFixed = true;

	for (int i = 0; i < numPoints_ - 1; i++) {
		constraints_[i].prevIndex = i;
		constraints_[i].nextIndex = i + 1;
		constraints_[i].distance = Length(Subtract(points_[i].position, points_[i + 1].position)); // 初期距離を設定 

	}
}

void nuno::Update(){
	//外力による加速度を計算
	for (int i = 0; i < numPoints_; i++)
	{
		for (int j = 0; j <numPoints_ ; j++)
		{
			points_[i][j].velocity += gravity_ * dt_;
		if (points_[i][j].isFixed)
		{
			points_[i][j].velocity = Vector2(0.0f, 0.0f);
		}

		}
		

	}

	
	// 位置の更新
	for (int i = 0; i < numPoints_; i++)
	{
		for (int j = 0; j < numPoints_; j++)
		{
			points_[i][j].estimationPosition = points_[i][j].position + (points_[i][j].velocity * dt_);
			points_[i][j].position = points_[i][j].estimationPosition;
		}
		
	}

	points_[0][0].position = startPos_;
	//points_[0].isFixed = true;
	// 最後の点をendPosに固定
	points_[numPoints_ - 1][numPoints_ - 1].position = endPos_;
	//points_[numPoints_ - 1].isFixed = true;

	//速度の更新
	for (int i = 0; i < constraints_.size(); i++)
	{
		Constraint c = constraints_[i];
		Points p1 = points_[c.prevIndex];
		Points p2 = points_[c.nextIndex];
		float w1 = 1.0f / p1.mass;// 質量の逆数を計算
		float w2 = 1.0f / p2.mass;// 質量の逆数を計算

		float diff = Length(Subtract(p1.position , p2.position));
		Vector2 dp1 = Multiply((-springStiffness_ * w1 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));
		Vector2 dp2 = Multiply((springStiffness_ * w2 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));

		points_[c.prevIndex].velocity += dp1 / dt_;
		points_[c.nextIndex].velocity += dp2 / dt_;

	}
	for (int i = 0; i < numPoints_; i++) {
		if (points_[i].isFixed)
		{
			points_[i].velocity = Vector2(0.0f, 0.0f);
		}
	}
}
void nuno::Draw(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix)
{
}
