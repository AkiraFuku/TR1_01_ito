#include "PBD.h"
#include "MassFunction.h"
void PBD::Initialize(Vector3 startPos, Vector3 endPos, int numPoints, float k, float dt, float kDamping, Vector3 gravity)
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
		points_[i].velocity = Vector3(0.0f, 0.0f,0.0f);// 初期速度をゼロに設定
		points_[i].mass = 1.0f;// 質量を1.0に設定

	}
	points_[0].isFixed = true;
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
		points_[i].velocity +=Multiply(dt_, gravity_)  ;
		if (points_[i].isFixed)
		{
			points_[i].velocity = Vector3(0.0f, 0.0f,0.0f);
		}

	}

	
	

	// 位置の更新
	std::vector<Vector3> oldPosition(numPoints_);
	for (int i = 0; i < numPoints_; i++)
	{
		oldPosition[i] = points_[i].position;
		points_[i].estimationPosition = Add(points_[i].position , Multiply(dt_ ,points_[i].velocity ));
		points_[i].position = points_[i].estimationPosition;
	}



	//速度の更新
	for (const Constraint& c : constraints_
)
	{
	
		Points p1 = points_[c.prevIndex];
		Points p2 = points_[c.nextIndex];
		if (p1.isFixed && p2.isFixed) {continue;} // 両方固定ならスキップ
		float w1 = 1.0f / p1.mass;// 質量の逆数を計算
		float w2 = 1.0f / p2.mass;// 質量の逆数を計算

		float diff = Length(Subtract(p1.position , p2.position));
		Vector3 dp1 = Multiply((-springStiffness_ * w1 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));
		Vector3 dp2 = Multiply((springStiffness_ * w2 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));

		points_[c.prevIndex].velocity +=Division(dt_, dp1 ) ;
		points_[c.nextIndex].velocity += Division(dt_,dp2  );

	}
	

	// Solver iterations for constraints
	for (int iter = 0; iter < solverIterations_; ++iter) {
		for (const Constraint& c : constraints_) {
			PBD::Points& p1 = points_[c.prevIndex];
			PBD::Points& p2 = points_[c.nextIndex];

			// 推定位置を参照
			Vector3& x1 = p1.estimationPosition;
			Vector3& x2 = p2.estimationPosition;

			// 両方固定ならスキップ
			if (p1.isFixed && p2.isFixed) continue;

			Vector3 delta =Subtract( x2,  x1);
			float dist = Length(delta);
			if (dist == 0.0f) {
				continue;
			} // ゼロ距離は無視
			Vector3 correction = Multiply((c.distance - dist) , Division(dist,delta  ));

			float w1 = p1.isFixed ? 0.0f : 1.0f / p1.mass;
			float w2 = p2.isFixed ? 0.0f : 1.0f / p2.mass;
			float wsum = w1 + w2;
			if (wsum == 0.0f) continue;

			if (!p1.isFixed){ x1 -=Multiply((w1 / wsum) ,correction );} 
			if (!p2.isFixed) {x2 +=Multiply( (w2 / wsum), correction );}
		}

		
	}

	// 制約解決後に追加
	for (int i = 0; i < numPoints_; i++) {
		points_[i].position = points_[i].estimationPosition;
	}

	// Update velocity after position update
	for (int i = 0; i < numPoints_; i++) {
		if (!points_[i].isFixed) {
			points_[i].velocity =Division(dt_, Subtract(points_[i].position , oldPosition[i]))  ;
		}
	}
	points_[0].position = startPos_;
	//points_[0].isFixed = true;
	// 最後の点をendPosに固定
	points_[numPoints_ - 1].position = endPos_;
//	points_[numPoints_ - 1].isFixed = true;
	VelocityDamping();
}

void PBD::Draw(
	Matrix4x4 viewProjectionMatrix, // ビュープロジェクション行列
	Matrix4x4 viewPortMatrix // ビューポート行列
)
{

	//std::vector<Vector3> screenPoints(numPoints_);
	
	for (int i = 0; i < points_.size(); i++)
	{

		Vector3 screen = Transform(Transform(points_[i].position, viewProjectionMatrix), viewPortMatrix);

		


		Novice::DrawEllipse(static_cast<int>(screen.x), static_cast<int>(screen.y), 5, 5, 0.0f, WHITE, kFillModeSolid);
		Novice::ScreenPrintf(10, 10 + i * 20, "Point %d: (%.02f, %.02f)%d,velocity:(%.02f, %.02f) ", i, points_[i].position.x, points_[i].position.y, points_[i].isFixed, points_[i].velocity.x, points_[i].velocity.y);
	}

	for (int i = 0; i < constraints_.size(); i++)
	{

		Constraint c = constraints_[i];
		Points p1 = points_[c.prevIndex];
		Points p2 = points_[c.nextIndex];

		Vector3 p1Screen = Transform(Transform(p1.position, viewProjectionMatrix), viewPortMatrix);
		Vector3 p2Screen = Transform(Transform(p2.position, viewProjectionMatrix), viewPortMatrix);
		
		Novice::DrawLine(static_cast<int>(p1Screen.x),static_cast<int>(p1Screen.y),static_cast<int>(p2Screen.x),static_cast<int>(p2Screen.y), WHITE);
	}

	// 開始位置と終了位置の描画
	Vector3 startScreen = Transform(Transform(startPos_, viewProjectionMatrix), viewPortMatrix);
	Vector3 endScreen = Transform(Transform(endPos_, viewProjectionMatrix), viewPortMatrix);
	Novice::DrawEllipse(static_cast<int>(startScreen.x), static_cast<int>(startScreen.y  ), 5, 5, 0.0f, RED, kFillModeSolid);
	Novice::DrawEllipse(static_cast<int>(endScreen.x), static_cast<int>(endScreen.y ), 5, 5, 0.0f, BLACK, kFillModeSolid);

}

void PBD::VelocityDamping()
{
	Vector3 xcm = Vector3(0.0f, 0.0f,0.0f);
	Vector3 vcm = Vector3(0.0f, 0.0f,0.0);
	float totalMass = 0.0f;
	for (int i = 0; i < numPoints_; i++)
	{
		if (points_[i].isFixed){continue;}
		xcm +=Multiply(points_[i].mass,points_[i].position ) ;
		vcm +=Multiply( points_[i].mass , points_[i].velocity);
		totalMass += points_[i].mass;
	}
	xcm /= totalMass;
	vcm /= totalMass;
	Vector3 l = Vector3(0.0f, 0.0f,0.0f);
	float i = 0.0f;
	std::vector<Vector3> rs(numPoints_);
	for (int j = 0; j < numPoints_; j++)
	{
		if (points_[j].isFixed) {
			continue;
		}
		Vector3 r = Subtract(points_[j].position , xcm);
		rs[j] = r;


		l += Cross(r, Multiply(points_[j].mass, points_[j].velocity));
		i += Dot(r, r) * points_[j].mass;
	}
	Vector3 omega = Multiply(1.0f / i, l);
	for (int j = 0; j < numPoints_; j++)
	{
		if (points_[j].isFixed){ continue;}
		Vector3 deltaV =Subtract( Add(vcm , Cross(omega, rs[j])) , points_[j].velocity);
		points_[j].velocity +=Multiply(kDamping_, deltaV)  ;
	}



}









