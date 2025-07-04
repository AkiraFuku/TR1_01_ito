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

	points_.resize(numPoints_);
	for (int i = 0; i < numPoints_; ++i) {
		points_[i].resize(numPoints_);
	}// 点の数だけポイントを確保
	constraints_.resize(numPoints_ - 1);// 制約の数は点の数 - 1

	for (int i = 0; i < numPoints_; i++) {
		float t = static_cast<float>(i) / static_cast<float>((numPoints_ - 1));
		for (int j = 0; j < numPoints_; j++) {
			points_[i][j].position = Lerp(startPos_, endPos_, t);
			points_[i][j].estimationPosition = points_[i][j].position;
			points_[i][j].velocity = Vector3(0.0f, 0.0f, 0.0f);
			points_[i][j].mass = 1.0f;
		}
	}
	points_[0][0].isFixed = true;
	points_[numPoints_ - 1][numPoints_ - 1].isFixed = true;

	for (int i = 0; i < numPoints_; ++i) {
		for (int j = 0; j < numPoints_; ++j) {
			// 横方向（右隣）
			if (i + 1 < numPoints_) {
				Constraint c;
				c.prevI = i; c.prevJ = j;
				c.nextI = i + 1; c.nextJ = j;
				c.distance = Length(Subtract(points_[i][j].position, points_[i + 1][j].position));
				constraints_.push_back(c);
			}
			// 縦方向（下隣）
			if (j + 1 < numPoints_) {
				Constraint c;
				c.prevI = i; c.prevJ = j;
				c.nextI = i; c.nextJ = j + 1;
				c.distance = Length(Subtract(points_[i][j].position, points_[i][j + 1].position));
				constraints_.push_back(c);
			}
			// 必要なら斜め方向も追加
		}
	}


}

void PBD::Update()
{
	//外力による加速度を計算
	for (int i = 0; i < numPoints_; i++)
	{
		for (int j = 0; j < numPoints_; j++)
		{
			if (points_[i][j].isFixed) {
				continue;
			} // 固定点はスキップ
			points_[i][j].velocity += Multiply(dt_, gravity_);
			if (points_[i][j].isFixed)
			{
				points_[i][j].velocity = Vector3(0.0f, 0.0f, 0.0f);
			}
		}


	}




	// 位置の更新
	std::vector<std::vector<Vector3>> oldPosition(numPoints_, std::vector<Vector3>(numPoints_));
	for (int i = 0; i < numPoints_; ++i) {
		for (int j = 0; j < numPoints_; ++j) {
			oldPosition[i][j] = points_[i][j].position;
			points_[i][j].estimationPosition = Add(points_[i][j].position, Multiply(dt_, points_[i][j].velocity));
			points_[i][j].position = points_[i][j].estimationPosition;
		}
	}



	//速度の更新
	for (const Constraint& c : constraints_)
	{

		Points p1 = points_[c.prevI][c.prevI];
		Points p2 = points_[c.nextI][c.nextJ];
		if (p1.isFixed && p2.isFixed) {
			continue;
		} // 両方固定ならスキップ
		float w1 = 1.0f / p1.mass;// 質量の逆数を計算
		float w2 = 1.0f / p2.mass;// 質量の逆数を計算

		float diff = Length(Subtract(p1.position, p2.position));
		Vector3 dp1 = Multiply((-springStiffness_ * w1 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position, p2.position)));
		Vector3 dp2 = Multiply((springStiffness_ * w2 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position, p2.position)));

		points_[c.prevI][c.prevJ].velocity += Division(dt_, dp1);
		points_[c.nextI][c.nextJ].velocity += Division(dt_, dp2);

	}


	// Solver iterations for constraints
	for (int iter = 0; iter < solverIterations_; ++iter) {
		for (const Constraint& c : constraints_) {
			PBD::Points& p1 = points_[c.prevI][c.prevJ];
			PBD::Points& p2 = points_[c.nextI][c.nextJ];

			// 推定位置を参照
			Vector3& x1 = p1.estimationPosition;
			Vector3& x2 = p2.estimationPosition;

			// 両方固定ならスキップ
			if (p1.isFixed && p2.isFixed) continue;

			Vector3 delta = Subtract(x2, x1);
			float dist = Length(delta);
			if (dist == 0.0f) {
				continue;
			} // ゼロ距離は無視
			Vector3 correction = Multiply((c.distance - dist), Division(dist, delta));

			float w1 = p1.isFixed ? 0.0f : 1.0f / p1.mass;
			float w2 = p2.isFixed ? 0.0f : 1.0f / p2.mass;
			float wsum = w1 + w2;
			if (wsum == 0.0f) continue;

			if (!p1.isFixed) {
				x1 -= Multiply((w1 / wsum), correction);
			}
			if (!p2.isFixed) {
				x2 += Multiply((w2 / wsum), correction);
			}
		}


	}

	// 制約解決後に追加
	for (int i = 0; i < numPoints_; i++) {
		for (int j = 0; j < numPoints_; j++)
		{
		points_[i][j].position = points_[i][j].estimationPosition;

		}
	}

	// Update velocity after position update
	for (int i = 0; i < numPoints_; i++) {
		for (int j = 0; j < numPoints_; j++)
		{
			if (points_[i][j].isFixed) {
				continue;
			} // 固定点はスキップ
			points_[i][j].velocity = Division(dt_, Subtract(points_[i][j].position, oldPosition[i][j]));
		}
		/*if (!points_[i].isFixed) {
			points_[i].velocity = Division(dt_, Subtract(points_[i].position, oldPosition[i]));
		}*/
	}
	points_[0][0].position = startPos_;
	//points_[0].isFixed = true;
	// 最後の点をendPosに固定
	points_[numPoints_ - 1][numPoints_ - 1].position = endPos_;
	//	points_[numPoints_ - 1].isFixed = true;
	VelocityDamping();
}

void PBD::Draw(
	Matrix4x4 viewProjectionMatrix, // ビュープロジェクション行列
	Matrix4x4 viewPortMatrix // ビューポート行列
)
{

	for (int i = 0; i < numPoints_; ++i) {
		for (int j = 0; j < numPoints_; ++j) {
			Vector3 screen = Transform(Transform(points_[i][j].position, viewProjectionMatrix), viewPortMatrix);
			Novice::DrawEllipse(static_cast<int>(screen.x), static_cast<int>(screen.y), 5, 5, 0.0f, WHITE, kFillModeSolid);
			// デバッグ用表示
			// Novice::ScreenPrintf(10, 10 + (i * numPoints_ + j) * 20, "Point (%d,%d): (%.02f, %.02f)", i, j, points_[i][j].position.x, points_[i][j].position.y);
		}
	}

	for (const Constraint& c : constraints_) {
    const Points& p1 = points_[c.prevI][c.prevJ];
    const Points& p2 = points_[c.nextI][c.nextJ];
    Vector3 p1Screen = Transform(Transform(p1.position, viewProjectionMatrix), viewPortMatrix);
    Vector3 p2Screen = Transform(Transform(p2.position, viewProjectionMatrix), viewPortMatrix);
    Novice::DrawLine(static_cast<int>(p1Screen.x), static_cast<int>(p1Screen.y),
                     static_cast<int>(p2Screen.x), static_cast<int>(p2Screen.y), WHITE);
	}

	// 開始位置と終了位置の描画
	Vector3 startScreen = Transform(Transform(startPos_, viewProjectionMatrix), viewPortMatrix);
	Vector3 endScreen = Transform(Transform(endPos_, viewProjectionMatrix), viewPortMatrix);
	Novice::DrawEllipse(static_cast<int>(startScreen.x), static_cast<int>(startScreen.y), 5, 5, 0.0f, RED, kFillModeSolid);
	Novice::DrawEllipse(static_cast<int>(endScreen.x), static_cast<int>(endScreen.y), 5, 5, 0.0f, BLACK, kFillModeSolid);

}

void PBD::VelocityDamping()
{
	Vector3 xcm = Vector3(0.0f, 0.0f, 0.0f);
	Vector3 vcm = Vector3(0.0f, 0.0f, 0.0);
	float totalMass = 0.0f;
	for (int i = 0; i < numPoints_; i++)
	{
		for (int j = 0; j < numPoints_; ++j) {
            if (points_[i][j].isFixed) continue;
            xcm += Multiply(points_[i][j].mass, points_[i][j].position);
            vcm += Multiply(points_[i][j].mass, points_[i][j].velocity);
            totalMass += points_[i][j].mass;
        }
	}
	if (totalMass == 0.0f) {return;}
	xcm /= totalMass;
	vcm /= totalMass;
	 Vector3 l = Vector3(0.0f, 0.0f, 0.0f); // 角運動量
    float inertia = 0.0f; // 慣性モーメント
    // 各点のrベクトルと角運動量・慣性モーメント
    std::vector<std::vector<Vector3>> rs(numPoints_, std::vector<Vector3>(numPoints_));
    for (int i = 0; i < numPoints_; ++i) {
        for (int j = 0; j < numPoints_; ++j) {
            if (points_[i][j].isFixed) continue;
            Vector3 r = Subtract(points_[i][j].position, xcm);
            rs[i][j] = r;
            l += Cross(r, Multiply(points_[i][j].mass, points_[i][j].velocity));
            inertia += Dot(r, r) * points_[i][j].mass;
        }
    }
    if (inertia == 0.0f) return;
    Vector3 omega = Multiply(1.0f / inertia, l);

    // 各点の速度を減衰
    for (int i = 0; i < numPoints_; ++i) {
        for (int j = 0; j < numPoints_; ++j) {
            if (points_[i][j].isFixed) continue;
            Vector3 deltaV = Subtract(Add(vcm, Cross(omega, rs[i][j])), points_[i][j].velocity);
            points_[i][j].velocity += Multiply(kDamping_, deltaV);
        }
    }



}









