#pragma once
#include <KamataEngine.h>
#include <Novice.h>
#include<vector>
using namespace KamataEngine;



class Yarm
{


public:
	struct Points
	{
	Vector3 position;
	Vector3 prevPosition;
	Vector3 velocity;
	float mass;

	bool isFixed ;
	};
	struct Constraint{
		int i;//質点
		int j;//質点
		float d;//
	};
	
	void Init(Vector3 startPos, Vector3 endPos, int numPoints, float k, float dt, float kDamping, Vector3 gravity);
	void Update();
	void draw();
	
private:
	Vector3 starrtPos; // Start position of the path

	Vector3 endPos; // End position of the path
	int numPoints; // Number of points in the path
	float k;//バネの硬さ
	float dt;//Delta time for simulation
	float kDamping; // Damping coefficient
	Vector3 gravity; // Gravity vector
	std::vector<Points> points; // Points in the path
	std::vector<Constraint> constraints; // Constraints between points
	
	










};

