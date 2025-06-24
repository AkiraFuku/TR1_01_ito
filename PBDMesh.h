#pragma once  
#include "PBD.h"  
#include <vector> // std::vector�̐錾��ǉ�  
//#include "MassFunction.h"
using namespace KamataEngine;
class nuno  
{  
public:  
	void Initialize(Vector2 startPos, Vector2 endPos, int numPoints, Vector2 gravity);  
	void Update();
	void Draw(const Matrix4x4& viewProjectionMatrix,
	const Matrix4x4& viewportMatrix);  

	void SetEndPos(Vector2 endPos) {  
		endPos_ = endPos;  
	} // Set the end position of the path  
	void SetStartPos(Vector2 straPos) {  
		startPos_ = straPos;  
	} // Set the end position of the path  

private:  
	Vector2 startPos_; // �p�X�̊J�n�ʒu  
	Vector2 endPos_; // �p�X�̏I���ʒu  
	int numPoints_; // �p�[�e�B�N���̐�  
	const float springStiffness_; // �o�l�̍d��  
	const float dt_; // �f���^����  
	Vector2 gravity_; // Gravity vector  
	std::vector<std::vector<PBD::Points>> points_; // �p�[�e�B�N���̃��X�g  
	std::vector<std::vector<PBD::Constraint>> constraints_; // �ڑ��̃��X�g  
};
