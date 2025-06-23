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

private:  
	
	int kSubdivision_ = 1; // 分割数
	std::vector<std::vector<PBD::Points>> meshPoints_; // 各頂点の物理情報
	std::vector<PBD::Constraint> constraints_;         // 拘束リスト
	PBD pbd_; // PBDの物理演算用インスタンス
	
};
