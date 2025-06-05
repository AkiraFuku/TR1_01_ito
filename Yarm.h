#pragma once
#include <KamataEngine.h>
#include <Novice.h>
#include<vector>
#include"MassPoint.h"
#include "Constraint.h"
#include "MassFunction.h"
using namespace KamataEngine;



class Yarm
{



	
private:
	Vector3 starrtPos_; // Start position of the path

	Vector3 endPos_; // End position of the path
	int numPoints_; // Number of points in the path
	float k_;//ÉoÉlÇÃçdÇ≥
	float kDamping_; // Damping coefficient
	Vector3 gravity_; // Gravity vector
	std::vector<MassPoint> points_; // Points in the path
	std::vector<Vector3> velocities_; // Velocities at each point
	std::vector<Constraint> constraints_; // Constraints between points

	std::vector<float> m_;//éøó 
	





public:
	
	void Update();
	void draw();

	


};

