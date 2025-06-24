#pragma once  
#include "PBD.h"  
#include <vector> // std::vectorの宣言を追加  
//#include "MassFunction.h"
using namespace KamataEngine;
class nuno  
{  
public:  
	void Initialize(Vector2 startPos, Vector2 endPos, int numPoints, Vector2 gravity);  
	void Update();
	void Draw(const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix);  

	void SetEndPos(Vector2 endPos) {  
		endPos_ = endPos;  
	} // Set the end position of the path  
	void SetStartPos(Vector2 straPos) {  
		startPos_ = straPos;  
	} // Set the end position of the path  

private:  
	Vector2 startPos_; // パスの開始位置  
	Vector2 endPos_; // パスの終了位置  
	int numPoints_; // パーティクルの数  
	const float springStiffness_; // バネの硬さ  
	const float dt_; // デルタ時間  
	Vector2 gravity_; // Gravity vector  
	std::vector<std::vector<PBD::Points>> points_; // パーティクルのリスト  
	std::vector<std::vector<PBD::Constraint>> constraints_; // 接続のリスト  
};
