#include "Yarm.h"

void Yarm::Update()
{
	// 物理演算（重力・慣性）
for (int i = 0; i < points_.size(); ++i) {
    if (!points_[i].isFixed) {
        Vector3 temp = points_[i].position;
        Vector3 velocity = points_[i].position - points_[i].prevPosition;
        p.position += velocity; // 慣性
        p.position.y += 0.98f;  // 重力（適宜調整）
        p.prevPosition = temp;
    }
}

// 拘束解決
for (int iter = 0; iter < 5; ++iter) {
    for (const auto& c : constraints) {
        auto& p1 = points_[c.i];
        auto& p2 = points_[c.j];
        Vector2 delta = p2.position - p1.position;
        float dist = sqrt(delta.x * delta.x + delta.y * delta.y);
        float diff = (dist - c.restLen) / dist;
        if (!p1.isFixed)
            p1.position.x += 0.5f * diff * delta.x;
            p1.position.y += 0.5f * diff * delta.y;
        if (!p2.isFixed)
            p2.position.x -= 0.5f * diff * delta.x;
            p2.position.y -= 0.5f * diff * delta.y;
    }
}
}

void Yarm::draw()
{
	for (int i = 0; i < points_.size() - 1; ++i) {
   Novice:: DrawLine(
	   static_cast<int>(points_[i].position.x),
	   static_cast<int>(points_[i].position.y),
	   static_cast<int>(points_[i+1].position.x),
	   static_cast<int>(points_[i+1].position.y),
	   WHITE
	   
	   );
	  
}
}

