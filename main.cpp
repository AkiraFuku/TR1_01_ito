#include <Novice.h>
#include <KamataEngine.h>
#include <imgui.h>
#include "PBD.h"
#include "MassFunction.h"
const char kWindowTitle[] = "学籍番号";
using namespace KamataEngine;
const int kwindowWidth = 1280;
const int kwindowHight = 720;


// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Vector2 mousePosition = { 0.0f, 0.0f };

	Vector3 startPosition = { 0.0f, 0.0f,0.0f };
	Vector3 endPosition = { 50.0f, 0.0f,0.0f };

	PBD* pbd = new PBD;
	int pointCount = 10;// 点の数
	float k = 0.1f; // バネの定数
	const float dt = 1.f / 60.f; // デルタタイム
	Vector3 gravity = { 0.0f, -9.8f ,0.0f}; // 重力ベクトル

	float kDamping = 0.05f; // 減衰率
	float m = 1.0f;

	Vector3 rotate = { 0.0f, 0.0f, 0.0f };
	Vector3 traslate = { 0.0f, 0.0f, 0.0f };
	Vector3 cameraTranslate = { 0.0f, 1.9f, -6.49f };
	Vector3 cameraRotate = { 0.26f, 0.0f, 0.0f };

	// PBDの初期化
	pbd->Initialize(startPosition, endPosition, pointCount, k, dt, kDamping, gravity);
	//pbd->InitPoints();

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// マウスの位置を取得


		// 修正: static_cast を削除し、適切な型変換を行うために一時変数を使用します。
		int mouseX = static_cast<int>(mousePosition.x);
		int mouseY = static_cast<int>(mousePosition.y);
		Novice::GetMousePosition(&mouseX, &mouseY);
		mousePosition.x = static_cast<float>(mouseX);
		mousePosition.y = static_cast<float>(mouseY);

		ImGui::Begin("PBD Control Panel");
		//ImGui::SliderFloat("Spring Constant (k)", &k, 0.0f, 1.0f, "%.3f");
		//ImGui::SliderFloat("Mass", &m, 0.01f, 10.0f, "%.3f");
		ImGui::SliderFloat3("startPos", &startPosition.x, 0.01f, 1280.0f, "%.3f");
		ImGui::SliderFloat3("EndPos", &endPosition.x, 0.01f, 1280.0f, "%.3f");
		ImGui::SliderFloat3("Camera Pos", &cameraTranslate.x, -1280.0f, 1280.0f, "%.2f");

		ImGui::End();
		pbd->setK(k);
		pbd->SetDt(dt);
		pbd->SetMass(m);
		pbd->SetStartPos(startPosition);
		pbd->SetEndPos(endPosition);
		pbd->Update();

		Matrix4x4 worldMatrix = MakeAfineMatrix({ 1.0f, 1.0f, 1.0f }, rotate, traslate);
		////MatrixScreenPrintf(0, kRowHeight, worldMatrix, "worldMatrix");
		Matrix4x4 cameraMatrix = MakeAfineMatrix({ 1.0f, 1.0f, 1.0f },cameraRotate,cameraTranslate );
		////MatrixScreenPrintf(0, kRowHeight * 20, cameraMatrix, "cameraMatrix");
		Matrix4x4 viewMatrix = Inverse(cameraMatrix);
		////MatrixScreenPrintf(0, kRowHeight * 30, viewMatrix, "viewMatrix");
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, static_cast<float>(kwindowWidth)/static_cast <float>(kwindowHight) , 0.1f, 100.0f);
		////MatrixScreenPrintf(kColumnWidth*10, kRowHeight  , projectionMatrix, "projectionMatrix");
		Matrix4x4 worldViewProjectionMatrix = Multiply( worldMatrix,Multiply(viewMatrix,projectionMatrix));
		Matrix4x4 viewProjectionMatrix = Multiply(viewMatrix, projectionMatrix);
		////MatrixScreenPrintf(kColumnWidth*10, kRowHeight * 20, worldViewProjectionMatrix, "worldViewProjectionMatrix");
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, static_cast<float>(kwindowWidth), static_cast<float>(kwindowHight), 0.0f, 1.0f);
		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		pbd->Draw(viewProjectionMatrix,viewportMatrix);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
