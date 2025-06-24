#include "PBDMesh.h"
#include "MassFunction.h"
void nuno::Initialize(Vector2 startPos, Vector2 endPos, int numPoints, Vector2 gravity)
{

	points_.clear();
	constraints_.clear();
	startPos_ = startPos;// �J�n�ʒu��ݒ�
	endPos_ = endPos;// �I���ʒu��ݒ�
	numPoints_ = numPoints;// �_�̐���ݒ�
	gravity_ = gravity;// �d�͂�ݒ�

	if (numPoints_ < 2) return; // 2�_�����͖���

	points_.resize(numPoints_);// �_�̐������|�C���g���m��
	constraints_.resize(numPoints_ - 1);// ����̐��͓_�̐� - 1

	for (int i = 0; i < numPoints_; i++) {

		float t = static_cast<float>(i) / static_cast<float>((numPoints_ - 1));



		points_[i].position = Lerp(startPos_, endPos_, t);// �����ʒu����`��ԂŐݒ�

		points_[i].estimationPosition = points_[i].position; // �����ʒu��ݒ�	
		points_[i].velocity = Vector2(0.0f, 0.0f);// �������x���[���ɐݒ�
		points_[i].mass = 1.0f;// ���ʂ�1.0�ɐݒ�

	}
	points_[0][0].isFixed = true;
	//points_[1].isFixed = true;
	points_[numPoints_ - 1][numPoints_-1].isFixed = true;

	for (int i = 0; i < numPoints_ - 1; i++) {
		constraints_[i].prevIndex = i;
		constraints_[i].nextIndex = i + 1;
		constraints_[i].distance = Length(Subtract(points_[i].position, points_[i + 1].position)); // ����������ݒ� 

	}
}

void nuno::Update(){
	//�O�͂ɂ������x���v�Z
	for (int i = 0; i < numPoints_; i++)
	{
		for (int j = 0; j <numPoints_ ; j++)
		{
			points_[i][j].velocity += gravity_ * dt_;
		if (points_[i][j].isFixed)
		{
			points_[i][j].velocity = Vector2(0.0f, 0.0f);
		}

		}
		

	}

	
	// �ʒu�̍X�V
	for (int i = 0; i < numPoints_; i++)
	{
		for (int j = 0; j < numPoints_; j++)
		{
			points_[i][j].estimationPosition = points_[i][j].position + (points_[i][j].velocity * dt_);
			points_[i][j].position = points_[i][j].estimationPosition;
		}
		
	}

	points_[0][0].position = startPos_;
	//points_[0].isFixed = true;
	// �Ō�̓_��endPos�ɌŒ�
	points_[numPoints_ - 1][numPoints_ - 1].position = endPos_;
	//points_[numPoints_ - 1].isFixed = true;

	//���x�̍X�V
	for (int i = 0; i < constraints_.size(); i++)
	{
		Constraint c = constraints_[i];
		Points p1 = points_[c.prevIndex];
		Points p2 = points_[c.nextIndex];
		float w1 = 1.0f / p1.mass;// ���ʂ̋t�����v�Z
		float w2 = 1.0f / p2.mass;// ���ʂ̋t�����v�Z

		float diff = Length(Subtract(p1.position , p2.position));
		Vector2 dp1 = Multiply((-springStiffness_ * w1 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));
		Vector2 dp2 = Multiply((springStiffness_ * w2 / (w1 + w2) * (diff - c.distance)), Normalize(Subtract(p1.position , p2.position)));

		points_[c.prevIndex].velocity += dp1 / dt_;
		points_[c.nextIndex].velocity += dp2 / dt_;

	}
	for (int i = 0; i < numPoints_; i++) {
		if (points_[i].isFixed)
		{
			points_[i].velocity = Vector2(0.0f, 0.0f);
		}
	}
}
void nuno::Draw(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix)
{
}
