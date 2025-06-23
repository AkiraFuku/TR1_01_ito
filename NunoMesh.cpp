#include "NunoMesh.h"
#include "DrawFunction.h" // 3D描画関数がここにあると仮定
#include <imgui.h>
#include "PBD.h"


void NunoMesh::Initialize() {
    const float kGridHalfWidth = 2.0f;
    const float kGridEvery = (kGridHalfWidth * 2.0f) / static_cast<float>(kSubdivision_);

    meshPoints_.resize(kSubdivision_ + 1);
    for (int y = 0; y <= kSubdivision_; ++y) {
        meshPoints_[y].resize(kSubdivision_ + 1);
        for (int x = 0; x <= kSubdivision_; ++x) {
            float px = -kGridHalfWidth + kGridEvery * x;
            float py = 0.0f; // 高さ
            float pz = -kGridHalfWidth - kGridEvery * y;
            meshPoints_[y][x] = Vector3{ px, py, pz };
        }
    }
}

void NunoMesh::Update() {
    ImGui::SliderFloat3 ("Point[0][0]",&meshPoints_[0][0].x,0.01f, 1.0f, "%.3f");
    ImGui::SliderFloat3 ("Point[kSubdivision_][kSubdivision_]",&meshPoints_[kSubdivision_][kSubdivision_].x,0.01f, 1.0f, "%.3f");

    
}

void NunoMesh::Draw(
    const Matrix4x4& viewProjectionMatrix,
    const Matrix4x4& viewportMatrix
) {
    for (int y = 0; y <= kSubdivision_; ++y) {
        for (int x = 0; x <= kSubdivision_; ++x) {
            // 3D→2D変換
            Vector3 pos3d = meshPoints_[y][x];
            Vector3 projected = Transform(pos3d, viewProjectionMatrix);
            Vector3 screen = Transform(projected, viewportMatrix);

            // 頂点を描画


            if (x==0&&y==0)
            {
                Novice::DrawEllipse(
                static_cast<int>(screen.x),
                static_cast<int>(screen.y),
                5, 5, 0.0f, RED, kFillModeSolid
            );
            } else if(x==kSubdivision_&&y==kSubdivision_)
            {
                Novice::DrawEllipse(
                 static_cast<int>(screen.x),
                 static_cast<int>(screen.y),
                 5, 5, 0.0f, BLUE, kFillModeSolid
                );
            } else
            {
                 Novice::DrawEllipse(
                 static_cast<int>(screen.x),
                 static_cast<int>(screen.y),
                 5, 5, 0.0f, WHITE, kFillModeSolid
            );
            }
           

            

            // 右隣と線
            if (x + 1 <= kSubdivision_) {
                Vector3 right = Transform(Transform(meshPoints_[y][x + 1], viewProjectionMatrix), viewportMatrix);
                Novice::DrawLine(
                    static_cast<int>(screen.x), static_cast<int>(screen.y),
                    static_cast<int>(right.x), static_cast<int>(right.y),
                    WHITE
                );
            }
            // 下隣と線
            if (y + 1 <= kSubdivision_) {
                Vector3 down = Transform(Transform(meshPoints_[y + 1][x], viewProjectionMatrix), viewportMatrix);
                Novice::DrawLine(
                    static_cast<int>(screen.x), static_cast<int>(screen.y),
                    static_cast<int>(down.x), static_cast<int>(down.y),
                    WHITE
                );
            }
        }
    }
}

void NunoMesh::SyncNunoMeshWithPBD(NunoMesh& mesh, const PBD& pbd) {
    int idx = 0;
    for (int y = 0; y < mesh.meshPoints_.size(); ++y) {
        for (int x = 0; x < mesh.meshPoints_[y].size(); ++x) {
            if (idx < pbd.) {
                mesh.meshPoints_[y][x].x = pbd.points_[idx].position.x;
                mesh.meshPoints_[y][x].y = pbd.points_[idx].position.y;
                // mesh.meshPoints_[y][x].z = 0.0f; // 2Dならzは0
                ++idx;
            }
        }
    }
}
