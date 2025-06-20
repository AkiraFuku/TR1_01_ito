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
	struct Constraint{
		int prevIndex;//ひとつ前の質点
		int nextIndex;//ひとつ次の質点
		float distance;//
	};
	
	void Initialize(Vector2 startPos, Vector2 endPos, int numPoints, float k, float dt, float kDamping, Vector2 gravity);
	void Update();
	void Draw();

	void SetEndPos(Vector2 endPos) {
		endPos_ = endPos;
	} // Set the end position of the path
	void SetStartPos(Vector2 straPos) {
		startPos_ = straPos;
	} // Set the end position of the path
	void VelocityDamping();
	
	void setK(float springStiffness) {
		springStiffness_ = springStiffness;
	} // Set the spring constant
	void SetDt(float dt){
		dt_=dt;
	}
	void SetMass(float mass){
		for (int i = 0; i < points_.size(); i++)
		{
			points_[i].mass=mass;

		}
	
	
	}
	
private:
	Vector2 startPos_; // パスの開始位置

	Vector2 endPos_; // パスの終了位置
	int numPoints_; // パーティクルの数
	float springStiffness_;//バネの硬さ
	float dt_;//デルタ時間
	float kDamping_; // Damping係数
	Vector2 gravity_; // Gravity vector
	std::vector<Points> points_; //パーティクルのリスト
	std::vector<Constraint> constraints_; // 接続のリスト
	
	










};

