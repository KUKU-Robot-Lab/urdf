# 로봇 자산 생성 규격 — `openarm_dg5f-m_bi_rl` 기준

> 2026-09-09 작성. **새 로봇 자산을 만들 때 이 문서를 기준으로 한다.**
> 파이프라인: `tools/generate_rl_urdf.py` → `tools/build_usd.py --sync-hdgp`
> 이 문서는 "왜 이렇게 하는가"를 담는다. "무엇을 하는가"는 각 도구의 상수 주석에 있다.

---

## 0. 지배 원칙 — 모든 값의 기준은 벤더

로봇에 들어가는 값은 **전부** 벤더·공식 레포에서 온다. PD 게인뿐 아니라 effort 한계,
관절 한계, 질량, collider 종류, URDF→USD 변환 설정까지 전부다. 우리 값이 벤더와 다르면
**그 차이 자체가 먼저 정당화**되어야 하고, 정당화가 "가드나 테스트를 통과시키려고"이면
그건 정당화가 아니다.

값을 정하는 순서: ① 벤더 레포 → ② 공식 문서 → ③ 실측 → ④ 그 외(기록 필수).

**★벤더가 Isaac Sim 변환 설정을 직접 주는 경우가 있다.** 자산 논쟁은 여기서 먼저 확인한다:

```
urdf/vendor/delto_m_ros2/dg_isaacsim/dg5f_telop/rb10_1300e_dg5f_{left,right}/config.yaml
  collider_type: convex_hull        merge_fixed_joints: false     self_collision: false
  convert_mimic_joints_to_normal_joints: false                    make_instanceable: true
  joint_drive: force/position, stiffness 100.0 / damping 1.0      link_density: 0.0
```

**다른 레포의 수치를 우리 관절에 매핑하지 말 것.** 구조·방법론은 참고하되 숫자는 이식 금지다
(SimToolReal 의 `HAND_JOINT_ARMATURE` 는 SHARPA 값이다). 벤더가 안 주는 양이면 **넣지 않는 것이
기본**이다 — DG-5F 는 rotor inertia·기어비를 제공하지 않으므로 armature 를 넣지 않는다.

**벤더 사본이 둘이면 어느 쪽인지 명시한다.** DG-5F 는 CAD 릴리스(`vendor/tesollo_model`)와
드라이버 사본(`vendor/delto_m_ros2/dg_description`)의 **질량과 관절한계 9곳이 다르다**
(엄지 상한 CAD 1.3439 vs 드라이버 0.8901). `openarm_dg5f-m_bi_rl` 은 드라이버 사본을 쓴다
(조립 xacro 가 `$(find dg_description)/urdf/dg5f_right.xacro` 를 문다).

---

## 1. collision 근사 — 기본은 `convex_hull`

```python
DEFAULT_COLLIDER = "convex_hull"
COLLIDER_DEVIATIONS = { "<asset>": ("convex_decomposition", "<사유 — 빈 문자열이면 빌드 에러>") }
```

**새 로봇은 아무것도 선언하지 않으면 hull 이다.** 근거는 §0 의 벤더 Isaac config.

09.05 에 우리가 고른 `convex_decomposition` 은 비벤더 선택이었고 그 대가가 컸다:

| | decomposition | hull |
|---|---|---|
| 로봇 collision shape (런타임 실측) | **731** | **75** |
| 부팅 PhysX contact-buffer overflow | 3런 각 1회 | 0회 |
| 자기충돌 ON 가능 여부 | 불가 | **가능** |

조각 경계의 깊은 관통이 큰 depenetration 임펄스를 만들고, 그게 관절 한계 구속을 뚫었다.

**예외를 등록할 때는 사유를 반드시 적는다.** 현재 예외 둘:
- `openarm_rh56f1_bi` — `self_collision_allowlist.yaml` 의 `accept_raw` 항목들이
  decomposition 임포트를 전제로 수용된 것이다. hull 로 바꾸려면 그 전제부터 재측정.
- `openarm_gripper_bi` — 그리퍼 조는 **파지 도구**라 hull 이면 개구(84.5 mm 실측)와
  파지 대역(10~85 mm)이 무효가 된다. 09.09 미검증이라 보류.

---

## 2. 관절 한계 — 자산마다 **명시적 선언 필수**

```python
JOINT_LIMIT_RESTRICTIONS = { "<asset>": { r"<joint regex>": (lower, upper) } }   # 빈 dict 허용, 미등록은 빌드 에러
```

**벤더 URDF 한계는 자기관통 없는 가동 범위를 보장하지 않는다.** 09.09 에 확인된 두 가지:

- `_3`/`_4`(PIP·DIP)는 CAD·드라이버 두 사본이 **똑같이 ±1.5708**(대칭 ±90°)이다. 실제
  손가락이 뒤로 90° 젖혀질 리 없으므로 **placeholder** 다. 20관절 전부 플랫인 effort 7.5 와
  같은 성격이고, 두 사본이 같아서 벤더 안에서 교차검증도 안 된다.
- `thumb_1` 하한 −0.384(−22.9°)는 엄지가 손바닥/베이스를 **0.67 mm 파고드는** 각도다.

그 한계를 그대로 쓰면 정책이 도달 불가능한 자세를 명령하고, 접촉이 관절을 한계 **밖으로**
밀어낸다 — 실측 `r_hj_thumb_1` **−4.075 rad**(표본 93% 한계 밖), `index_3` −1.16 rad.

빈 dict 도 반드시 등록한다. "아직 안 쟀다"와 "잴 필요가 없다"를 구분해서 남기기 위해서다.

### 2-1. 측정 방법 (이대로 하지 않으면 답이 뒤집힌다)

- **다른 관절은 0(편 상태)** 으로 두고 하나씩 굽힌다. 이웃 관절을 극단에 고정해 놓고 재면
  그건 **그 조합**을 재는 것이지 그 관절을 재는 게 아니다. 09.09 에 `_2` 를 벤더 상한에
  둔 채 `_3/_4` 를 굽혀 "네 손가락 굴곡을 제한해야 한다"는 결론을 냈는데, 분리해서 다시
  재니 **네 손가락은 ±90° 까지 5.2~8.4 mm 간극이 남았다** — 답이 뒤집혔다.
- **같은 손가락 안쪽 2-hop 쌍을 반드시 포함한다**(`_1↔_3`, `_1↔_4`, `_2↔_4`, `_2↔tip`).
  PhysX 자동 필터는 **직접 연결된 부모-자식뿐**이고, 굴곡 한계를 정하는 건 그 2-hop 쌍이다.
- 스윕 값이 각도에 따라 **안 변하면** 최근접 쌍이 스윕하지 않는 곳에 있다는 뜻이다 —
  측정이 아무것도 재고 있지 않다는 신호다.
- 도구: `tools/audit_self_collision.py` 의 `UrdfModel` / `penetration()` API.
  ⚠`link_parts()` 는 전 링크 메시를 적재하므로 필요한 링크만 직접 `local_geometry()` 로
  읽을 것(80링크 전부 읽으면 메모리에서 죽는다).

### 2-2. DG-5F-M 확정값 (28관절 축소)

| 관절 | 이전(벤더) | 확정 | 근거 |
|---|---|---|---|
| `_1` × 5 (벌림/대향) | 폭 0.44~1.22 | **±0.16** | 사용자 확정 — 벌림은 무조건 옆 손가락을 침범한다. 엄지는 실측으로도 하한이 0.67 mm 관통 |
| `thumb_3`, `thumb_4` | ±1.5708 | **[0, 1.05]** | 실측 — 1.05 rad 까지 간극 0.15 mm 유지, **1.20 rad(68.8°)부터 tip↔palm 4.95 mm 관통** |
| `_3`, `_4` × 8 | ±1.5708 | **[0, 1.5708]** | 상한은 벤더 유지(간극 5.2~8.4 mm). 하한만 0 |

**하한 0 을 물리 한계에도 넣는 이유**: 액션 범위는 이미 override 로 0 이었지만 물리 한계가
±1.5708 이라 **접촉이 관절을 음수로 밀 수 있었다**(실측 `index_3` −1.16 rad = 66° 뒤로 꺾임).
정책이 명령할 수 없는 자세를 물리도 만들지 못하게 한다.

⚠**한계를 좁히면 프로필의 기본 자세를 같이 고쳐야 한다.** IsaacLab 은 런타임 clamp **앞에서**
기본자세를 한계와 대조해 거부한다. 09.09 에 `thumb_3` 기본자세 −0.5 를 안 고쳐 서버 3런이
동시에 죽었다. hdgp `test_profile_poses_are_inside_asset_joint_limits` 가 이제 막는다.

---

## 3. 링크 구조 — 주소용 프레임은 규칙으로 제거

지오메트리가 없고, 들어오는 고정관절이 1개이며, 앞뒤 조인트 중 하나가 **항등변환**인 링크는
자동 제거된다(`find_addressing_frames()`). 기구학적으로 아무 일도 하지 않으면서 PhysX 강체만
하나 늘리기 때문이다.

**왜 질량을 키우는 걸로는 안 되나** — PhysX 는 무질량 강체를 만들 수 없다(무질량이면 1 kg
유령이 된다). 그래서 토큰 질량 1e-5 kg 을 줄 수밖에 없고, 팔(0.47 kg)↔손(1.76 kg) 체인에
10⁵ 질량비가 낀다. **"sim 에서 무질량·무접촉"을 얻는 유일한 방법은 링크를 없애는 것**이다.

⚠**링크는 소비처가 없어도 조인트 이름은 쓰인다.** `r_hj_mount` 를 fabric 생성기와 기하 계약
테스트가 이름으로 찾고 있었고, 09.09 에 40건이 깨졌다. 그래서 **항등인 조인트를 버리고
변환을 가진 조인트 이름을 남긴다.** 앞뒤가 둘 다 비항등이면 어느 이름을 남길지 정할 수 없으므로
빌드가 죽는다.

이름으로 참조되는 프레임은 `KEEP_ADDRESSING_FRAMES` 에 등록해 보존한다
(`*_hl_palm_ee` — `robots.py` 의 `palm_ee_body`, `head_cam_view` — 매니페스트 `camera_view_frame`).

DG-5F-M 결과: 손 링크 **31 → 29** (벤더 28 + `palm_ee`), 전체 84 → 80.

**`merge_fixed_joints` 는 `false` 로 둔다** — 벤더 Isaac config 도 false 다. 벤더 고정관절
(base·palm·tip)은 병합하지 않는다. hdgp 가 `palm_body`·`fingertip_bodies` 를 이름으로 쓴다.

---

## 4. 외부 참조는 **상대경로**

`build_usd.relativize_head_reference()` 가 `convert()` 의 **마지막**에 head_v1 참조를
hdgp 배치 기준 상대경로(`../../simulation_setting/head_v1/usd/head_v1.usda`)로 다시 쓴다.

여태 빌드 머신의 절대경로(`/home/user/rl_ws/...`)가 박혀 학습 서버(`/home/oem/...`)에서
열리지 않았다. 참조 대상은 서버에도 **있었다** — 경로만 틀렸다. 치명적이지 않아 경고로만
남았고 env 하나당 3줄씩 나온다: 4096 env 에 12,288줄, **24576 env 에 73,728줄**.
⚠그동안 **머리 지오메트리가 서버 학습에서 빠진 채**였다.

graft 시점에는 참조를 **읽어서** 물리를 벗겨내야 하므로 절대경로로 걸고, 모든 편집이 끝난 뒤
상대화한다. 그 시점 이후로는 빌드 디렉터리에서 해석되지 않으므로 head graft 검증은 sync 후
hdgp 사본에서 하고, `--sync-hdgp` 없이 빌드하면 **"검증되지 않은 자산"이라며 죽는다.**

**새 로봇이 외부 자산을 참조하면 같은 처리를 해야 한다.** 절대경로는 자산을 빌드한 사람의
머신에 묶는다.

---

## 5. 자기충돌 — 켜는 것이 기본, 조건이 있다

**켜야 하는 이유**: 꺼두면 손가락이 서로와 손바닥을 **통과**한다. 실기에 없는 자세를 정책이
학습하므로 그 자체가 sim2real 격차다. c 시리즈 영상의 "손가락이 엉킨다"가 그것이고, 엄지가
손바닥을 뚫고 −4.07 rad 로 돌아간 것도 기하가 안 막은 것이 한 축이었다.

**켤 수 있는 조건**: `tools/audit_self_collision.py` 의 **영(리셋) 자세 감사 PASS, FAIL 0**.
DG-5F-M 은 §1·§2 를 마친 뒤 이 조건을 만족했고, 09.01 에 껐던 이유("손 hull 초기 겹침 ×
자기충돌 = 폭주")의 전제가 사라졌다.

설정은 **leaf cfg 에만** 둔다(hdgp `GraspFJTesolloRightEnvCfg.enable_self_collisions`).
base 에 두면 형제 트랙(RH56F1 등)이 조용히 끌려간다.

⚠**완전 폐쇄에서 손가락이 손바닥·서로에 닿는 것은 정상이다** — 실제 손도 그렇다. 그건 결함이
아니라 자기충돌이 막아야 할 바로 그 접촉이다. `audit_poses.yaml` 은 **"task home"**(로봇이
머무는 자세) 용이므로 폐쇄 자세를 등록하면 정상 접촉을 결함으로 신고하게 된다(09.09 시행착오).

⚠자기충돌은 충돌 스택 사용량을 크게 키운다. 24576 env 에서 PhysX 가
`collisionStackSize buffer overflow … at least 994,438,608` 로 죽었다 →
`gpu_collision_stack_size` 2²⁹ → **2³⁰**.

---

## 6. 새 로봇 추가 체크리스트

1. `SOURCES` 에 소스 URDF 등록, `build_usd.ASSET_BUILDS` 에 `hand_drive`·게인 출처 선언
   (미등록은 빌드 에러 — 기본값으로 조용히 넘어가지 않는다).
2. `JOINT_LIMIT_RESTRICTIONS` 에 항목 추가. **안 쟀으면 `{}`** 로 등록.
3. collision 근사는 손대지 않는다(hull 기본). decomposition 이 필요하면 **사유와 함께** 등록.
4. `python3 tools/generate_rl_urdf.py <name>` → 자기충돌 감사 PASS 확인.
   FAIL 이 나오면 그건 자산 결함이다 — 필터로 덮지 말고 기하를 고치거나 한계를 좁힌다.
5. §2-1 방법으로 관절별 가동 범위 측정 → 한계 축소 → 재생성.
6. `isaaclab.sh -p tools/build_usd.py <asset> --sync-hdgp`.
7. hdgp 프로필 자세(`init_joint_pos`·`hand_open_pose`)가 새 한계 안인지 확인
   (계약 테스트가 잡지만, 자산과 프로필을 같이 고치는 습관이 낫다).
8. 런타임 확인: 부팅 표(손 액션한계·시작거리·`action=/obs=/state=`) · collision shape 수
   (`root_physx_view.max_shapes`) · PhysX overflow 0 · head 참조 경고 0.

---

## 7. 흔한 함정 (전부 09.09 에 실제로 밟았다)

| 함정 | 증상 | 규칙 |
|---|---|---|
| 파생 구조에 오버라이드를 얹음 | 확인 로그는 찍히는데 물리가 안 바뀜. A/B 두 팔이 **비트 단위 동일** | 재조립이 읽는 **입력**(cfg 필드)에 넣는다. `finalize_after_overrides()`·`_apply_object_bank()`·`restore_run_cfg` 가 파생을 다시 만든다 |
| 조건당 1런으로 A/B | 노이즈를 효과로 보고 | baseline 을 **2런 이상** 먼저 돌려 편차를 박는다. 차이가 편차 안이면 "효과 미검출" |
| 이웃 관절을 극단에 고정하고 측정 | 조합을 재고 그 관절 탓으로 돌림 | 다른 관절은 **0(편 상태)** |
| 자동 필터를 믿고 쌍을 제외 | 한계를 정하는 쌍이 측정에서 빠짐 | PhysX 자동 필터는 **부모-자식뿐**. 2-hop 은 직접 넣는다 |
| 링크 이름만 grep 하고 제거 | 조인트 이름 소비처가 깨짐(40 errors) | 링크·조인트 **양쪽** 이름을 찾는다 |
| `pkill -f <패턴>` | 자기 셸의 명령줄이 매칭돼 스스로 죽음 | PID 로 종료한다 |
