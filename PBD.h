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
		Vector2 position;//　質点の位置
		Vector2 pPosition;// 質点の推定位置
		Vector2 velocity;// 質点の速度
		float mass;// 質点の質量
		bool isFixed ;
	};
	struct Constraint{
		int i;//質点
		int j;//質点
		float d;//
	};
	
	void Initialize(Vector2 startPos, Vector2 endPos, int numPoints, float k, float dt, float kDamping, Vector2 gravity);
	void Update();
	void draw();

	void SetEndPos(Vector2 endPos) {
		endPos_ = endPos;
	} // Set the end position of the path
	void VelocityDamping();
	
private:
	Vector2 startPos_; // Start position of the path

	Vector2 endPos_; // End position of the path
	int numPoints_; // Number of points in the path
	float k_;//バネの硬さ
	float dt_;//Delta time for simulation
	float kDamping_; // Damping coefficient
	Vector2 gravity_; // Gravity vector
	std::vector<Points> points_; // Points in the path
	std::vector<Constraint> constraints_; // Constraints between points
	
	










};

