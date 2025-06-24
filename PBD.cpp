#include "PBD.h"
#include "MassFunction.h"

void PBD::Initialize(Vector2 startPos, Vector2 endPos, int numPoints, float k, float dt, float kDamping, Vector2 gravity)
{
	points_.clear();
	constraints_.clear();
	startPos_ = startPos;// 開始位置を設定
	endPos_ = endPos;// 終了位置を設定
	numPoints_ = numPoints;// 点の数を設定
	springStiffness_ = k;// バネ定数を設定
	dt_ = dt;// タイムステップを設定
	kDamping_ = kDamping;// ダンピング係数を設定
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
	points_[0].isFixed = true;
	//points_[1].isFixed = true;
	points_[numPoints_ - 1].isFixed = true;

	for (int i = 0; i < numPoints_ - 1; i++) {
		constraints_[i].prevIndex = i;
		constraints_[i].nextIndex = i + 1;
		constraints_[i].distance = Length(Subtract(points_[i].position, points_[i + 1].position)); // 初期距離を設定 

	}
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
	//VelocityDamping();

	// 位置の更新
	for (int i = 0; i < numPoints_; i++)
	{

		points_[i].estimationPosition = points_[i].position + (points_[i].velocity * dt_);
		points_[i].position = points_[i].estimationPosition;
	}

	points_[0].position = startPos_;
	points_[0].isFixed = true;
	// 最後の点をendPosに固定
	points_[numPoints_ - 1].position = endPos_;
	points_[numPoints_ - 1].isFixed = true;

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


void PBD::Draw()
{




	for (int i = 0; i < points_.size(); i++)
	{


		Vector2 screen{ points_[i].position.x,(points_[i].position.y - 500) * -1.0f };


		Novice::DrawEllipse(static_cast<int>(screen.x), static_cast<int>(screen.y), 5, 5, 0.0f, WHITE, kFillModeSolid);
		Novice::ScreenPrintf(10, 10 + i * 20, "Point %d: (%.02f, %.02f)%d,velocity:(%.02f, %.02f) ", i, points_[i].position.x, points_[i].position.y, points_[i].isFixed, points_[i].velocity.x, points_[i].velocity.y);
	}

	for (int i = 0; i < constraints_.size(); i++)
	{

		Constraint c = constraints_[i];
		Points p1 = points_[c.prevIndex];
		Points p2 = points_[c.nextIndex];

		int x1 = static_cast<int>(p1.position.x);
		int y1 = static_cast<int>((p1.position.y - 500) * -1.0f);
		int x2 = static_cast<int>(p2.position.x);
		int y2 = static_cast<int>((p2.position.y - 500) * -1.0f);
		Novice::DrawLine(x1, y1, x2, y2, WHITE);
	}

	Novice::DrawEllipse(static_cast<int>(startPos_.x), static_cast<int>((startPos_.y - 500) * -1.0f), 5, 5, 0.0f, RED, kFillModeSolid);
	Novice::DrawEllipse(static_cast<int>(endPos_.x), static_cast<int>((endPos_.y - 500) * -1.0f), 5, 5, 0.0f, BLACK, kFillModeSolid);

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
	float i = 0.0f;
	std::vector<Vector2> rs(numPoints_);
	for (int j = 0; j < numPoints_; j++)
	{
		Vector2 r = points_[j].position - xcm;
		rs[j] = r;

		l += Cross(r, Multiply(points_[j].mass, points_[j].velocity));
		i += Length(r) * points_[j].mass;
	}
	Vector2 omega = Multiply(1.0f / i, l);
	for (int j = 0; j < numPoints_; j++)
	{
		Vector2 deltaV = vcm + Cross(omega, rs[j]) - points_[j].velocity;
		points_[j].velocity += deltaV * kDamping_;
	}



}


