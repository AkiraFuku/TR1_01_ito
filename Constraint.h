#pragma once
#include <KamataEngine.h>
using namespace KamataEngine;

class Constraint
{
public:
    int i, j;      // 質点のインデックス
    float restLen; // 目標距離
};

