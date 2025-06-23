#pragma once  
//#include "PBD.h"  
#include <vector>  
#include <KamataEngine.h>
//#include "PBDLinkConstraint.h"
using namespace KamataEngine;  
class NunoMesh  
{  
public:  
	void Initialize();  
	void Update();  
	void Draw(  
		const Matrix4x4& viewProjectionMatrix,  
		const Matrix4x4& viewportMatrix  
	);  
	void SyncNunoMeshWithPBD(NunoMesh& mesh, const PBD& pbd) ;
    

private:  
	
	int kSubdivision_ = 1; // 分割数（1なら2x2=4点）
	std::vector<std::vector<Vector3>> meshPoints_;
	
};
