#pragma once
#include <KamataEngine.h>
#include <Novice.h>
#include<vector>
using namespace KamataEngine;



class PBD
{


public:
	struct Points
	{
		//　質点の位置
		Vector2 position;
		// 質点の推定位置
		Vector2 estimationPosition;
		Vector2 velocity;// 質点の速度
		float mass;// 質点の質量
		bool isFixed ;
	};
	struct Point
	{
		//　質点の位置
		Vector2 position;
		bool isFixed ;
	};
	struct Constraint{
		int prevIndex;//ひとつ前の質点
		int nextIndex;//ひとつ次の質点
		float distance;//
	};
	
    void Initialize(Point startPos, Point endPos, int numPoints );
	void Update();
	void Draw();

	void SetEndPos(Vector2 endPos) {
		endPos_ = endPos;
	} // Set the end position of the path
	void SetStartPos(Vector2 straPos) {
		startPos_ = straPos;
	} // Set the end position of the path
	
	
	
	void SetMass(float mass){
		for (int i = 0; i < points_.size(); i++)
		{
			points_[i].mass=mass;

		}
	
	
	}

    // 座標と点数だけでPBDを初期化する簡易関数
    
	
private:
	Vector2 startPos_; // パスの開始位置

	Vector2 endPos_; // パスの終了位置
	int numPoints_; // パーティクルの数
	const float springStiffness_= 0.1f; ;//バネの硬さ
	const float dt_=1.0f / 60.f;;//デルタ時間
	
	Vector2 gravity_={ 0.0f, -9.8f }; // Gravity vector
	std::vector<Points> points_; //パーティクルのリスト
	std::vector<Constraint> constraints_; // 接続のリスト
	
	










};

