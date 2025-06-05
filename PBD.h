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
	Vector2 position;
	Vector2 prevPosition;
	Vector2 velocity;
	float mass;

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
	
private:
	Vector2 starrtPos; // Start position of the path

	Vector2 endPos; // End position of the path
	int numPoints; // Number of points in the path
	float k;//バネの硬さ
	float dt;//Delta time for simulation
	float kDamping; // Damping coefficient
	Vector2 gravity; // Gravity vector
	std::vector<Points> points; // Points in the path
	std::vector<Constraint> constraints; // Constraints between points
	
	










};

