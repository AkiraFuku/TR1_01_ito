class PBD;
struct PBDLinkConstraint {
    PBD* pbdA;
    int indexA;
    PBD* pbdB;
    int indexB;
    float restDistance; // 通常は0（同じ位置にしたい場合）
};