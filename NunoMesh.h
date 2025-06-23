#pragma once  
#include "PBD.h"  
#include <vector>  
using namespace KamataEngine;  
class NunoMesh  
{  
public:  
	void Initialize(Vector3 StartPos,Vector3 GolePos);  
	void Update();  
	void Draw(  
		const Matrix4x4& viewProjectionMatrix,  
		const Matrix4x4& viewportMatrix  
	);  

private:  
	Vector3 StartPos_;
	Vector3 GolePos_;

	std::vector<std::vector<PBD::Points>> nestedP_; // Fixed the template argument issue  
	std::vector<std::vector<PBD::Constraint>>Constraint;

};
