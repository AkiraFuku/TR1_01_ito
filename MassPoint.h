#pragma once
#include <KamataEngine.h>
using namespace KamataEngine;

class MassPoint {
public:
    Vector3 position;
    Vector3 prevPosition;
    bool isFixed = false; // �Œ�_���ǂ���
};