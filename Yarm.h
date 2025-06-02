#pragma once
#include <KamataEngine.h>
#include <Novice.h>
#include<vector>
using namespace KamataEngine;



class Yarm
{



	
private:
	Vector3 starrtPos; // Start position of the path

	Vector3 endPos; // End position of the path
	int numPoints; // Number of points in the path
	float k;//ÉoÉlÇÃçdÇ≥
	float kDamping; // Damping coefficient
	Vector3 gravity; // Gravity vector
	std::vector<Vector3> points; // Points in the path
	std::vector<Vector3> velocities; // Velocities at each point
	std::vector<Vector3> PrevPoints; // Previous points for drawing
	std::vector<float> m;//éøó 
	





public:
	
	void Update();
	void draw();




};

