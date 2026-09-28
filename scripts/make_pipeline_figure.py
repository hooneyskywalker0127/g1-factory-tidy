"""범용 파이프라인 그림: 말 → 보기 → 이동 → 파지 → 전신 도달 → 물리 검증. (g1-motion-tracking/scripts/make_pipeline_figure.py 의 틀)"""
import os, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.font_manager as fm, matplotlib.image as mpimg, matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch
FONT = "/usr/share/fonts/truetype/nanum/NanumSquareRoundB.ttf"; FONT_R = "/usr/share/fonts/truetype/nanum/NanumSquareRoundR.ttf"
HERE = os.path.dirname(os.path.abspath(__file__))
GREEN, GREEN_FILL = "#5aa02c", "#e8f2dc"; BLUE, BLUE_FILL = "#1f5fa9", "#dce8f7"; ORANGE, ORANGE_FILL = "#c8721c", "#fbe9d6"
STAGES = [
 dict(no="1단계", name="보기 · 언어 접지", tool="C-RADIO", kind="see", thumb="stage_1.png",
      body=["문장 → 물체·잡을 부위·목적지", "머리 카메라 RGB-D에서 찾음", "3D 중심 · 설 자리 계산"]),
 dict(no="2단계", name="이동 · 자세", tool="GR00T-WholeBodyControl", kind="plan", thumb="stage_2.png",
      body=["둘러보기 → 물체 앞으로 걷기", "쪼그리기 / 무릎 꿇기", "운반: 일어서서 목적지로 걷기"]),
 dict(no="3단계", name="파지 후보", tool="GraspGen-X", kind="plan", thumb="stage_3.png",
      body=["부위 점군 → 손 자세 후보 수백 개", "충돌 필터 · 신뢰도 순", "손 종류별 (5지 Inspire 등)"]),
 dict(no="4단계", name="전신 도달", tool="cuRobo", kind="plan", thumb="stage_4.png",
      body=["후보마다 허리·다리·팔 궤적 (IK)", "접근 → 쥐기 → 들기 → 놓기", "양손 작업도 같은 틀"]),
 dict(no="5단계", name="물리 검증 · 렌더", tool="Isaac Lab (PhysX)", kind="phys", thumb="stage_5.png",
      body=["후보를 실제로 쥐고 들어 봄", "든 것만 통과 → 재검증", "3뷰 영상 (전체·머리·손목)"]),
]
W, GAP, X0 = 382, 22, 75; BOX_BOTTOM, BOX_H = 150, 500; HEAD_BOTTOM, HEAD_H = 520, 130
def _thumb(ax, path, left, width):
    if not os.path.isfile(path): return
    img = mpimg.imread(path); iw, ih = img.shape[1], img.shape[0]
    tw = width - 44; th = tw * ih / iw; top = 500; max_h = 200
    if th > max_h: th = max_h; tw = th * iw / ih
    cx = left + width / 2; ax.imshow(img, extent=(cx - tw / 2, cx + tw / 2, top - th, top), aspect="auto", zorder=3)
def main(out):
    for f in (FONT, FONT_R):
        if os.path.isfile(f): fm.fontManager.addfont(f)
    plt.rcParams["font.family"] = fm.FontProperties(fname=FONT).get_name()
    n = len(STAGES); total_w = X0 + n * W + (n - 1) * GAP + 75
    fig, ax = plt.subplots(figsize=(total_w / 100.0, 8.9), dpi=100); ax.set_xlim(0, total_w); ax.set_ylim(0, 890); ax.axis("off"); fig.patch.set_facecolor("white")
    ax.text(65, 790, "말 한마디로 정리하는 Unitree G1 파이프라인", fontsize=33, color="#222222")
    ax.text(68, 745, "\"바닥의 물체를 책상 위 크레이트에 넣어\"  같은 문장 하나가 입력 · 좌표는 어디에도 주지 않음", fontsize=16, color="#666666")
    spans = [("see", "#f4f1e8", ORANGE, "카메라로 찾는 구간"), ("plan", "#f2f2f2", "#666666", "물리 없음 · 자세와 궤적을 계산하는 구간"), ("phys", "#eaf1fa", BLUE, "물리 있음 · 실제로 되는지 확인하는 구간")]
    for kind, fill, col, label in spans:
        idx = [i for i, s in enumerate(STAGES) if s["kind"] == kind]; l = X0 + idx[0] * (W + GAP) - 20; r = X0 + idx[-1] * (W + GAP) + W + 20
        ax.add_patch(FancyBboxPatch((l, 95), r - l, 585, boxstyle="round,pad=6,rounding_size=8", linewidth=0, facecolor=fill))
        ax.text((l + r) / 2, 120, label, fontsize=15, color=col, ha="center")
    for i, st in enumerate(STAGES):
        color, fill = {"see": (ORANGE, ORANGE_FILL), "plan": (GREEN, GREEN_FILL), "phys": (BLUE, BLUE_FILL)}[st["kind"]]
        left = X0 + i * (W + GAP)
        ax.add_patch(FancyBboxPatch((left, BOX_BOTTOM), W, BOX_H, boxstyle="round,pad=4,rounding_size=6", linewidth=2, edgecolor=color, facecolor="white"))
        ax.add_patch(FancyBboxPatch((left, HEAD_BOTTOM), W, HEAD_H, boxstyle="round,pad=4,rounding_size=6", linewidth=2, edgecolor=color, facecolor=fill))
        ax.text(left + 24, 612, st["no"], fontsize=15, color=color)
        ax.text(left + 22, 566, st["name"], fontsize=21, color="#222222")
        ax.text(left + 22, 534, st["tool"], fontsize=14, color="#555555")
        _thumb(ax, os.path.join(HERE, "..", "docs", "pipeline_thumbs", st["thumb"]), left, W)
        for j, line in enumerate(st["body"]): ax.text(left + 22, 255 - j * 44, line, fontsize=14.5, color="#333333")
        if i < n - 1: ax.add_patch(FancyArrowPatch((left + W + 3, 400), (left + W + GAP - 3, 400), arrowstyle="-|>", mutation_scale=20, linewidth=3, color="#9a9a9a", zorder=4))
    ax.text(65, 55, "운반은 2단계(일어서서 걷기)와 4단계(목적지 위에서 놓기)를 한 번 더 지난다.  모든 단계는 NVIDIA 공개 저장소를 그대로 쓴다: C-RADIO · GR00T-WholeBodyControl · GraspGen-X · cuRobo · Isaac Lab.", fontsize=14.5, color="#666666")
    fig.savefig(out, facecolor="white", bbox_inches="tight", pad_inches=0.12); print("saved", out)
if __name__ == "__main__": main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "docs", "pipeline.png"))
