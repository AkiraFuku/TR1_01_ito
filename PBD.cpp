#include "PBD.h"
#include "MassFunction.h"
void PBD::Initialize(Vector3 startPos, Vector3 endPos, int numPoints, float k, float dt, float kDamping, Vector3 gravity)
{
	points_.clear();
    constraints_.clear();
    bendConstraints_.clear();
    startPos_ = startPos;
    endPos_ = endPos;
    numPoints_ = numPoints;
    springStiffness_ = k;
    dt_ = dt;
    kDamping_ = kDamping;
    gravity_ = gravity;

    if (numPoints_ < 2) return;

    points_.resize(numPoints_);
    for (int i = 0; i < numPoints_; ++i) {
        points_[i].resize(numPoints_);
    }

    // 格子状に配置
    for (int i = 0; i < numPoints_; ++i) {
        for (int j = 0; j < numPoints_; ++j) {
            float tx = static_cast<float>(i) / (numPoints_ - 1);
            float ty = static_cast<float>(j) / (numPoints_ - 1);
            // X, Y方向に線形補間
            points_[i][j].position.x = (1 - tx) * startPos_.x + tx * endPos_.x;
            points_[i][j].position.y = (1 - ty) * startPos_.y + ty * endPos_.y;
            points_[i][j].position.z = (1 - tx) * startPos_.z + tx * endPos_.z; // 必要ならZも補間

            points_[i][j].estimationPosition = points_[i][j].position;
            points_[i][j].velocity = Vector3(0.0f, 0.0f, 0.0f);
            points_[i][j].mass = 1.0f;
            points_[i][j].isFixed = false;
        }
    }
    points_[0][0].isFixed = true;
    points_[numPoints_ - 1][numPoints_ - 1].isFixed = true;

    // 横方向の曲げ制約
    for (int i = 0; i < numPoints_ - 2; ++i) {
        for (int j = 0; j < numPoints_; ++j) {
            float rest = Length(Subtract(points_[i][j].position, points_[i + 2][j].position));
            BendConstraint bc = {i, j, i + 2, j, rest};
            bendConstraints_.push_back(bc);
        }
    }
    // 縦方向の曲げ制約
    for (int i = 0; i < numPoints_; ++i) {
        for (int j = 0; j < numPoints_ - 2; ++j) {
            float rest = Length(Subtract(points_[i][j].position, points_[i][j + 2].position));
            BendConstraint bc = {i, j, i, j + 2, rest};
            bendConstraints_.push_back(bc);
        }
    }

    // 横方向の距離制約
    for (int i = 0; i < numPoints_ - 1; ++i) {
        for (int j = 0; j < numPoints_; ++j) {
            float rest = Length(Subtract(points_[i][j].position, points_[i + 1][j].position));
            Constraint c = {i, i + 1, j, j, rest};
            constraints_.push_back(c);
        }
    }
    // 縦方向の距離制約
    for (int i = 0; i < numPoints_; ++i) {
        for (int j = 0; j < numPoints_ - 1; ++j) {
            float rest = Length(Subtract(points_[i][j].position, points_[i][j + 1].position));
            Constraint c = {i, i, j, j + 1, rest};
            constraints_.push_back(c);
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

		// Bend constraints
		for (const BendConstraint& bc : bendConstraints_) {
			Points& p1 = points_[bc.i1][bc.j1];
			Points& p2 = points_[bc.i2][bc.j2];
			if (p1.isFixed && p2.isFixed) continue;

			Vector3 delta = Subtract(p2.estimationPosition, p1.estimationPosition);
			float dist = Length(delta);
			if (dist == 0.0f) continue;
			Vector3 correction = Multiply((dist - bc.restLength) / dist * 0.5f, delta);

			if (!p1.isFixed) p1.estimationPosition += correction;
			if (!p2.isFixed) p2.estimationPosition -= correction;
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
    // 各点を描画
    for (int i = 0; i < numPoints_; ++i) {
        for (int j = 0; j < numPoints_; ++j) {
            Vector3 screen = Transform(Transform(points_[i][j].position, viewProjectionMatrix), viewPortMatrix);
            int color = points_[i][j].isFixed ? RED : WHITE;
            Novice::DrawEllipse(static_cast<int>(screen.x), static_cast<int>(screen.y), 5, 5, 0.0f, color, kFillModeSolid);
       			 Novice::ScreenPrintf(10, 10 + (i * numPoints_ + j) * 20, "Point (%d,%d): (%.02f, %.02f)", i, j, points_[i][j].position.x, points_[i][j].position.y);

		}
    }

    // 横方向の線（右隣と結ぶ）
    for (int i = 0; i < numPoints_ - 1; ++i) {
        for (int j = 0; j < numPoints_; ++j) {
            Vector3 p1 = Transform(Transform(points_[i][j].position, viewProjectionMatrix), viewPortMatrix);
            Vector3 p2 = Transform(Transform(points_[i + 1][j].position, viewProjectionMatrix), viewPortMatrix);
            Novice::DrawLine(static_cast<int>(p1.x), static_cast<int>(p1.y), static_cast<int>(p2.x), static_cast<int>(p2.y), WHITE);
        }
    }

    // 縦方向の線（下隣と結ぶ）
    for (int i = 0; i < numPoints_; ++i) {
        for (int j = 0; j < numPoints_ - 1; ++j) {
            Vector3 p1 = Transform(Transform(points_[i][j].position, viewProjectionMatrix), viewPortMatrix);
            Vector3 p2 = Transform(Transform(points_[i][j + 1].position, viewProjectionMatrix), viewPortMatrix);
            Novice::DrawLine(static_cast<int>(p1.x), static_cast<int>(p1.y), static_cast<int>(p2.x), static_cast<int>(p2.y), WHITE);
        }
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









