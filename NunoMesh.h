#pragma once  
#include <vector>  
#include <KamataEngine.h>
#include "PBD.h" // PBDをインクルード
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
	int kSubdivision_ = 1; // 分割数（1なら2x2=4点）

	// PBD用の頂点情報
	std::vector<std::vector<PBD::Points>> meshPoints_; // 2次元配列で管理
	std::vector<PBD::Constraint> constraints_;         // 拘束リスト

	// PBD本体
	PBD pbd_;
};
