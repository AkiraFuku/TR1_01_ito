#include <Novice.h>
#include <KamataEngine.h>
#include "PBD.h"
const char kWindowTitle[] = "学籍番号";
using namespace KamataEngine;

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	Vector2 mousePosition = {0.0f, 0.0f };

	Vector2 startPosition = {800.0f, 700.0f };
	Vector2 endPosition = { 1200.0f, 700.0f };

    PBD* pbd = new PBD;
	int pointCount = 5;// 点の数
	float k = 0.01f; // バネの定数
	float dt = 0.01f; // デルタタイム
	Vector2 gravity = { 0.0f, 0.0f }; // 重力ベクトル
	float kDamping = 0.03f; // 減衰率
	


	// PBDの初期化
	pbd->Initialize(startPosition,endPosition,pointCount ,k,dt,kDamping,gravity);

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
		//Novice::GetMousePosition(mousePosition.x, mousePosition.y);

		pbd->Update();

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		pbd->Draw();

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
