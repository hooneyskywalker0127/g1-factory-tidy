"""후보 선택. 체인이 BEST 를 고를 때 쓴다.

예전 규칙은 `max(object followed)` 였다. 그런데 stand-up 에서 물체가 "따라온 양"은
물체가 손에 있을 때만 의미가 있다. 이미 바닥에 떨어진 물체는 0.000 을 기록하고,
실제로 들려 있다가 떨어진 후보(-0.272)를 이긴다. 260929 rg54 에서 이 규칙이
c1(물체가 f585 이전에 이미 바닥) 을 골랐고, wrap 까지 살아남은 유일한 후보 c0 을
버렸다. 로그로 확인된 오선택이다.

새 규칙: hold2 시점에 물체가 손바닥에 얼마나 가까운가. 손에 있으면 작고, 바닥에
있으면 크다. HELD 가 있으면 그것이 항상 이긴다.
"""
import re, sys, math

def score(path):
    try:
        t = open(path, errors="ignore").read()
    except OSError:
        return None
    if "-> HELD" in t:
        return 1e6
    m = re.findall(r"after hold2\s+\(frame\s+\d+\): object \[([^\]]+)\].*?right palm \[([^\]]+)\]", t)
    if m:
        o = [float(x) for x in m[-1][0].split()]
        p = [float(x) for x in m[-1][1].split()]
        return -math.dist(o, p)          # 가까울수록 높은 점수
    f = re.findall(r"object followed ([+-]?[0-9.]+) m", t)   # hold2 줄이 없을 때만
    return max((float(x) for x in f), default=-1e9) - 1e3

if __name__ == "__main__":
    best, bv = 0, -1e18
    for k, path in enumerate(sys.argv[1:]):
        v = score(path)
        if v is None:
            continue
        print(f"  c{k} {path.split('/')[-1]}: score {v:.4f}", file=sys.stderr)
        if v > bv:
            best, bv = k, v
    print(best)
