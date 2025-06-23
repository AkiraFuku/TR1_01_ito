#include"DrawFunction.h"

void DrawSphere(const Sphere& sphere, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	const uint32_t kSubdivision=24;//分割数
	const float kLonEvery= 2.0f * PI / static_cast<float>(kSubdivision);
	const float kLatEvery=PI / static_cast<float>(kSubdivision);
	///緯度に分割
	for (uint32_t latIndex = 0; latIndex < kSubdivision; ++latIndex)
	{
		float lat = -PI/2.0f+kLatEvery*latIndex;//現在の緯度
		float latD = PI / static_cast<float>(kSubdivision);//緯度の分割幅
		//経度に分割
		for (uint32_t lonIndex = 0; lonIndex < kSubdivision; lonIndex++)
		{
			float lon=lonIndex*kLonEvery;//現在の経度
			float lonD = PI * 2.0f / static_cast<float>(kSubdivision);//経度の分割幅
			//world
			Vector3 a,b,c;
			a={
				 sphere.center.x + sphere.radius * cosf(lat)*cosf(lon),
				 sphere.center.y + sphere.radius *sinf(lat),
				 sphere.center.z + sphere.radius *cosf(lat)*sinf(lon)
			};
			b={
				 sphere.center.x + sphere.radius * cosf(lat+latD)*cosf(lon),
				 sphere.center.y + sphere.radius *sin(lat+latD),
				 sphere.center.z + sphere.radius * cosf(lat + latD) * sinf(lon)
			};
			c={ 
				 sphere.center.x + sphere.radius * cosf(lat) * cosf(lon + lonD),
				sphere.center.y + sphere.radius *sinf(lat),
				sphere.center.z + sphere.radius *cosf(lat) * sinf(lon + lonD) };
			//緯度線
			Vector3 screen[3] = {};
			for (uint32_t i = 0; i < 3; i++)
			{
				Vector3 nbc = Transform(i==0?a:i==1?b:c, viewProjectionMatrix);
				screen[i] = Transform(nbc, viewportMatrix);
			}
			//描画
			Novice::DrawLine(static_cast<int>(screen[0].x), static_cast<int>(screen[0].y),
								static_cast<int>(screen[1].x), static_cast<int>(screen[1].y), color);
			Novice::DrawLine(static_cast<int>(screen[0].x), static_cast<int>(screen[0].y),
				static_cast<int>(screen[2].x), static_cast<int>(screen[2].y), color);

			
			
			
		}

	}

}

void DrawGrid(const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix)
{
	const float kGridHalfWidth=2.0f;//グリッドの半分
	const uint32_t kSubdivision=10;//分割数
	//一つ分の長さ
	const float kGridEvery =(kGridHalfWidth*2.0f)/static_cast<float>(kSubdivision);

	//奥から引いていく
	for (uint32_t xIndex = 0; xIndex <= kSubdivision; ++xIndex)
	{
		Vector3 kGridLocalVertices[2]={};
			kGridLocalVertices[0]={-kGridHalfWidth+static_cast<float>(xIndex)*static_cast<float>(kGridEvery),0.0f,kGridHalfWidth};
			kGridLocalVertices[1]={-kGridHalfWidth+static_cast<float>(xIndex)*static_cast<float>(kGridEvery),0.0f,-kGridHalfWidth};

		Vector3 screen[2]={};
		for (uint32_t i = 0; i < 2; i++)
		{


			Vector3 nbc = Transform(kGridLocalVertices[i], viewProjectionMatrix);
			screen[i] = Transform(nbc, viewportMatrix);
		}

		

		if (xIndex==kSubdivision/2)
		{
			Novice::DrawLine(static_cast<int>(screen[0].x),static_cast<int>(screen[0].y),static_cast<int>(screen[1].x),static_cast<int>(screen[1].y),0x000000FF);

		}else{
			Novice::DrawLine(static_cast<int>(screen[0].x),static_cast<int>(screen[0].y),static_cast<int>(screen[1].x),static_cast<int>(screen[1].y),0xAAAAAAFF);
		
		}
	}
	for (uint32_t zIndex = 0; zIndex <= kSubdivision; zIndex++)
	{
		Vector3 kGridLocalVertices[2]={};
			kGridLocalVertices[0]={kGridHalfWidth,0.0f,-kGridHalfWidth+static_cast<float>(zIndex)*static_cast<float>(kGridEvery)};
			kGridLocalVertices[1]={-kGridHalfWidth,0.0f,-kGridHalfWidth+static_cast<float>(zIndex)*static_cast<float>(kGridEvery)};

		Vector3 screen[2]={};
		for (uint32_t i = 0; i < 2; i++)
		{


			Vector3 nbc = Transform(kGridLocalVertices[i], viewProjectionMatrix);
			screen[i] = Transform(nbc, viewportMatrix);
		}

		
		if (zIndex==kSubdivision/2)
		{
			Novice::DrawLine(static_cast<int>(screen[0].x),static_cast<int>(screen[0].y),static_cast<int>(screen[1].x),static_cast<int>(screen[1].y),0x000000FF);

		}else{
			Novice::DrawLine(static_cast<int>(screen[0].x),static_cast<int>(screen[0].y),static_cast<int>(screen[1].x),static_cast<int>(screen[1].y),0xAAAAAAFF);
		
		}
		

	}
}

void DrawPlane(const Plane& plane, const Matrix4x4& viewProjectionMatrix, const Matrix4x4& viewportMatrix, uint32_t color)
{
	Vector3 center = Multiply(plane.distance, plane.normal);
	Vector3 perpendiculars[4];
	perpendiculars[0] = Normalize(Perpendicular(plane.normal));
	perpendiculars[1]={-perpendiculars[0].x ,-perpendiculars[0].y,-perpendiculars[0].z};
	perpendiculars[2] = Cross(plane.normal, perpendiculars[0]);
	perpendiculars[3]={-perpendiculars[2].x ,-perpendiculars[2].y,-perpendiculars[2].z};

	Vector3 points[4];
	for (int i = 0; i < 4; ++i){
		Vector3 extend =Multiply(2.0f,perpendiculars[i]);
		Vector3 point = Add(center, extend);
		points[i] = Transform(Transform(point, viewProjectionMatrix), viewportMatrix);
		Novice::ScreenPrintf(0,15*i,"point%d:x%0.0f:y%0.0f:z%0.0f",i,points[i].x,points[i].y,points[i].z);
	}
	/*for (int i = 0; i < 4; i++)
	{
		Novice::DrawLine(
			static_cast<int>(points[i].x),
			static_cast<int>(points[i].y),
			static_cast<int>(points[(i+1)%4].x),
			static_cast<int>(points[(i+1)%4].y),
			color
			);
	}*/
	Novice::DrawLine (
		static_cast<int>(points[0].x),
		static_cast<int>(points[0].y),
		static_cast<int>(points[2].x),
		static_cast<int>(points[2].y),
			color);
	Novice::DrawLine (
		static_cast<int>(points[1].x),
		static_cast<int>(points[1].y),
		static_cast<int>(points[2].x),
		static_cast<int>(points[2].y),
			color);
	Novice::DrawLine (
		static_cast<int>(points[0].x),
		static_cast<int>(points[0].y),
		static_cast<int>(points[3].x),
		static_cast<int>(points[3].y),
			color);
	Novice::DrawLine (
		static_cast<int>(points[1].x),
		static_cast<int>(points[1].y),
		static_cast<int>(points[3].x),
		static_cast<int>(points[3].y),
			color);

}

/// <summary>
/// 正射影ベクトル
/// </summary>
/// <param name="v1"></param>
/// <param name="v2"></param>
/// <returns></returns>
Vector3 Project(const Vector3& v1, const Vector3& v2)
{
	float dot = Dot(v1, v2);
	float length = Length(v2);
	if (length == 0.0f) {return Vector3();} // v2がゼロベクトルの場合は投影できない

	
	return Multiply((dot / powf(length ,2)),v2);
}

Vector3 ClosestPoint(const Vector3& point, const Segment& segment)
{
	Vector3 project = Project(Subtract(point, segment.origin),segment.diff);
	return Add(segment.origin,project);
}

bool IsCollision(const Sphere& s1, const Sphere& s2)
{
	float distance= Length(Subtract(s2.center,s1.center));
	if (distance<=s1.radius+s2.radius)
	{
		return true;
	}
	return false;
}

bool IsCollision(const Sphere& sphere, const Plane& plane)
{
	float distance = Dot(sphere.center, plane.normal) - plane.distance;
	if (fabs(distance) <= sphere.radius)
	{
		return true;
	}
	return false;
}
