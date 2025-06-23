#include "PBD.h"
#include "MassFunction.h"
void PBD::Initialize(Vector3 startPos, Vector3 endPos, int numPoints,  Vector3 gravity)
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
		points_[i].velocity = Vector3(0.0f, 0.0f,0.0f);// 初期速度をゼロに設定
		points_[i].mass = 1.0f;// 質量を1.0に設定

	}
	points_[0].isFixed = true;
	//points_[1].isFixed = true;
	points_[numPoints_ - 1].isFixed =true;

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
			points_[i].velocity = Vector3(0.0f, 0.0f,0.0f);
		}

	}

	//ベロシティ　ダンピング
	

	// 位置の更新
	for (int i = 0; i < numPoints_; i++)
	{

		points_[i].estimationPosition =Add( points_[i].position , Multiply( dt_,points_[i].velocity ));
		points_[i].position = points_[i].estimationPosition;
	}

	points_[0].position = startPos_;
	//points_[0].isFixed = true;
	// 最後の点をendPosに固定
	points_[numPoints_ - 1].position = endPos_;
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
		Vector3 dp1 = Multiply((-springStiffness_ * w1 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));
		Vector3 dp2 = Multiply((springStiffness_ * w2 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));

		points_[c.prevIndex].velocity += Division(dt_,dp1);
		points_[c.nextIndex].velocity += Division (dt_,dp2);

	}
	
}

void PBD::Draw(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix)
{
    for (int i = 0; i < points_.size(); i++)
    {
        // 3D→2D変換
        Vector3 pos3d = points_[i].position;
        Vector3 projected = Transform(pos3d, viewProjectionMatrix);
        Vector3 screen = Transform(projected, viewportMatrix);

        Novice::DrawEllipse(
            static_cast<int>(screen.x),
            static_cast<int>(screen.y),
            5, 5, 0.0f, WHITE, kFillModeSolid
        );
        Novice::ScreenPrintf(
            10, 10 + i * 20,
            "Point %d: (%.02f, %.02f, %.02f) %d, velocity:(%.02f, %.02f, %.02f)",
            i, pos3d.x, pos3d.y, pos3d.z, points_[i].isFixed,
            points_[i].velocity.x, points_[i].velocity.y, points_[i].velocity.z
        );
    }

    for (int i = 0; i < constraints_.size(); i++)
    {
        Constraint c = constraints_[i];
        Points p1 = points_[c.prevIndex];
        Points p2 = points_[c.nextIndex];

        Vector3 p1_screen = Transform(Transform(p1.position, viewProjectionMatrix), viewportMatrix);
        Vector3 p2_screen = Transform(Transform(p2.position, viewProjectionMatrix), viewportMatrix);

        Novice::DrawLine(
            static_cast<int>(p1_screen.x), static_cast<int>(p1_screen.y),
            static_cast<int>(p2_screen.x), static_cast<int>(p2_screen.y),
            WHITE
        );
    }

    // 始点・終点も同様に変換して描画
    Vector3 start_screen = Transform(Transform(startPos_, viewProjectionMatrix), viewportMatrix);
    Vector3 end_screen = Transform(Transform(endPos_, viewProjectionMatrix), viewportMatrix);

    Novice::DrawEllipse(static_cast<int>(start_screen.x), static_cast<int>(start_screen.y), 5, 5, 0.0f, RED, kFillModeSolid);
    Novice::DrawEllipse(static_cast<int>(end_screen.x), static_cast<int>(end_screen.y), 5, 5, 0.0f, BLACK, kFillModeSolid);
}



