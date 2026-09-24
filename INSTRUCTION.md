제부터는 각 부품을 따로 parameterize하는 단계가 아니라, CLEAR Hand 전체 design vector를 코드에서 어떻게 표현할지를 정하면 돼.
내가 보기엔 지금 제일 먼저 해야 할 건 현재 HandMorphology라는 이름의 의미를 좁히는 것이야. 지금 HandMorphology는 사실 전체 hand morphology가 아니라 joint layout / reference frame graph만 표현하고 있거든.
나는 구조를 이렇게 잡는 게 제일 깔끔하다고 봐.
HandMorphologyParameters        <- optimizer가 보는 전체 design
│
├── fingertip                  <- shared morphology
│   ├── shared shape
│   └── per-digit total length
│
├── linkage                    <- shared ordinary-finger linkage
│
└── layout                     <- MCP placement + thumb layout

            ↓ construct

HandMorphology                 <- derived physical hand
│
├── fingertips[5]
├── linkage
└── layout / joint transforms
즉 현재의:
HandMorphologyParameters
HandMorphology
는 각각
HandLayoutParameters
HandLayout
로 바꾸고,
진짜 전체 hand를 나타내는 이름을 HandMorphology*가 가져가는 것이 의미적으로 맞아.
1. Hand-level aggregate
예를 들면 최종적으로 optimizer-facing API는 이렇게.
@dataclass(frozen=True)
class HandMorphologyParameters:
    fingertip: FingertipFamilyParameters
    fingertip_lengths: FingertipLengths
    linkage: FingerLinkageParameters
    layout: HandLayoutParameters
그리고:
@dataclass(frozen=True)
class FingertipLengths:
    index_mm: float
    middle_mm: float
    ring_mm: float
    little_mm: float
    thumb_mm: float
그러면 optimizer 입장에서는 정말 우리가 방금 정리한
\[
d=
[d_\text{tip}, L_I,L_M,L_R,L_L,L_T,
 d_\text{linkage},d_\text{layout}]
\]가 그대로 코드 구조로 드러남.
이게 좋은 이유는 sharing relationship 자체가 type structure에 박힌다는 것이야.
잘못해서:
index_linkage
middle_linkage
ring_linkage
같은 걸 만들 여지가 없어.
2. Fingertip은 지금 parameterization을 한 번 손보는 게 좋음
현재 코드를 보니까:
PadParameters
    pad_length_mm
    tip_length_mm

PhalanxParameters
    carrier_length_mm
로 되어 있어.
그런데 이제 우리가 원하는 의미는:
distal morphology는 모든 손가락이 같고, 손가락별로 전체 길이만 다르다.

잖아.
그러면 pad_length와 carrier_length를 각각 per-digit로 주는 것보다 한 개의 total length에서 둘 다 derive하는 게 훨씬 좋음.
현재 기본값을 보면:
pad length     = 40 mm
carrier length = 35 mm
tip length     = 15 mm
즉 carrier가 distal tip 끝에서 5 mm 안쪽에서 끝나.
이걸
\[
d_\text{carrier-inset}
=
L_\text{tip}-L_\text{carrier}
\]로 정의하면 현재는:
\[
d_\text{carrier-inset}=5\text{ mm}
\]이야.
그러면 각 digit에 대해 total length \(L_i\)만 주고:
\[
L_{\text{pad},i}=L_i
\]\[
L_{\text{carrier},i}
=
L_i-d_\text{carrier-inset}
\]로 만들면 돼.
이 방식이 특히 예쁜 이유가 하나 있는데, 지금 fingertip validation에 들어가는:
\[
\frac{
L_{\text{carrier}}-L_{\text{straight}}
}{
L_{\text{tip}}
}
\]에서
\[
L_{\text{straight}}=L_i-L_{\text{tip}}
\]니까
\[
L_{\text{carrier}}-L_{\text{straight}}
=
(L_i-d)-(L_i-L_{\text{tip}})
=
L_{\text{tip}}-d
\]가 돼.
전체 길이 \(L_i\)가 사라짐.
즉 손가락 길이를 늘려도 distal ellipsoid/carrier penetration morphology가 정확히 동일하게 유지돼.
이게 우리가 말하는:
같은 fingertip 구조인데 길이만 다르다

를 수학적으로도 가장 깔끔하게 구현하는 방식이야.
실제로 바뀌는 건 proximal straight section 길이뿐.
그래서 fingertip parameter는 이렇게 생각할 수 있음
Shared Fingertip Morphology
├── carrier width
├── carrier height
├── stem width
├── stem height
├── vertical pad width
├── vertical pad height
├── elliptical pad width
├── elliptical pad height
├── tip length
└── carrier distal inset

Per-digit
└── total length
즉 기존의
pad_length_mm
carrier_length_mm
를 independent shared parameter로 두지 않는 거지.
carrier_distal_inset_mm가 shared morphology parameter가 되고.
3. Linkage는 지금 구조 거의 그대로
여기는 별로 손댈 게 없어.
FingerLinkageParameters
    main_chain
    coupling
하나가 전체 ordinary finger에 공유.
그리고:
FingerLinkage(parameters)
하나만 construct.
Hand-level object에서는:
@property
def linkage(self) -> FingerLinkage:
    return FingerLinkage(self.parameters.linkage)
정도로 끝.
굳이:
index_linkage
middle_linkage
ring_linkage
little_linkage
객체 네 개를 만들 필요도 없어. geometry definition은 하나니까.
실제 pose를 계산할 때만 같은 linkage model을 각 finger transform에 attach하면 됨.
4. 지금 HandMorphology는 HandLayout으로
현재:
HandMorphologyParameters
HandMorphology
가 하는 일은 정확히:
MCP placement
thumb axis placement
joint transforms
actuator transforms
이니까 나는 이름을:
HandLayoutParameters
HandLayout
로 바꾸는 걸 추천해.
그러면 layout variable은:
@dataclass(frozen=True)
class HandLayoutParameters:
    middle_mcp_aa: HandPlanePoint
    ring_mcp_aa: HandPlanePoint
    little_mcp_aa: HandPlanePoint

    thumb_cmc_ro: HandPlanePoint
    theta_rad: float
    ro_raa_distance_mm: float
    phi_rad: float
딱 이것만 남음.
현재 있는:
raa_mcp_fe_distance_mm
는 삭제.
5. Fixed architecture는 HandLayout 내부 canonical constant
여기에는 이제 확정된 값들이 들어감.
ORDINARY_MCP_AA_TO_MCP_FE = T(23.5, 0.0, 16.75)

THUMB_CMC_RAA_TO_MCP_FE_DISTANCE_MM = 39.5
THUMB_MCP_FE_TO_IP_FE_DISTANCE_MM = 35.5
그리고 chain:
thumb_cmc_ro = (
    T(x, y, 0)
    @ Rz(theta)
)

thumb_cmc_raa = (
    thumb_cmc_ro
    @ Tx(ro_raa_distance)
)

thumb_mcp_fe = (
    thumb_cmc_raa
    @ Tx(39.5)
    @ Rz(phi)
)

thumb_ip_fe = (
    thumb_mcp_fe
    @ Tx(35.5)
)
그래서 최종 joint graph에 이제:
thumb_cmc_ro
thumb_cmc_raa
thumb_mcp_fe
thumb_ip_fe
까지 들어감.
thumb_ip_fe에도 XM335가 붙으니까 actuator transforms / packaging도 자연스럽게 하나 더 생기고.
6. 최종 HandMorphology
그러면 진짜 전체 object는 꽤 단순해.
개념적으로:
@dataclass(frozen=True)
class HandMorphology:
    parameters: HandMorphologyParameters

    @property
    def layout(self) -> HandLayout:
        return HandLayout(self.parameters.layout)

    @property
    def linkage(self) -> FingerLinkage:
        return FingerLinkage(self.parameters.linkage)

    @property
    def fingertips(self) -> dict[str, Fingertip]:
        ...
예를 들면:
hand.fingertips["index"]
hand.fingertips["thumb"]

hand.linkage

hand.layout.joint_transforms
hand.layout.actuator_transforms
이렇게.
즉 HandMorphology 자체는 뭔가 거대한 solver가 아니라 shared morphology pieces를 하나의 physical design으로 materialize하는 composition root 역할만 하면 돼.
이게 꽤 중요해.
7. Constraint도 자연스럽게 정리됨
현재:
check_actuator_packaging(morphology)
는 실제로 layout만 필요하잖아.
그래서 당장은:
check_actuator_packaging(hand.layout)
가 더 정확해.
나중에 전체 constraint evaluator를 만들면:
evaluate_design(hand)
안에서:
actuator packaging
linkage validity
fingertip validity
finger spacing
manufacturing clearance
...
를 종합하면 됨.
지금부터 HandMorphology에 constraint logic까지 집어넣지는 않는 게 좋아.
8. 코드 ownership은 이렇게
현재 directory structure를 유지하면서 충분히 가능해.
morphology/
├── fingertip_parameter.py
│   shared fingertip morphology
│   concrete/per-digit length representation
│
├── fingertip.py
│   concrete fingertip geometry
│
├── linkage_parameter.py
├── linkage.py
├── linkage_kinematics.py
├── linkage_seed.py
│
├── hand_parameter.py
│   HandLayoutParameters
│   FingertipLengths
│   HandMorphologyParameters   <- whole hand aggregate
│
└── hand.py
    HandLayout
    HandMorphology
    fixed architectural transforms
새 파일 안 만들어도 됨.
지금 폴더를 다시 깔끔하게 만든 직후니까 나도 이게 좋아 보여.
9. 구현 순서
나는 한 번에 다 뜯지 않고 이 순서로 갈 것 같아.
1. HandLayout 정리
   - current HandMorphology rename
   - raa_mcp_fe_distance_mm 제거
   - fixed 39.5 mm 적용
   - IP-FE 35.5 mm 추가
   - thumb_ip_fe actuator까지 추가
2. Fingertip length semantics 정리
   - pad_length / carrier_length independent 구조 제거
   - shared distal morphology + per-digit total length
   - carrier_distal_inset 도입
3. whole-hand aggregate 추가
   - FingertipLengths
   - top-level HandMorphologyParameters
   - top-level HandMorphology
4. consumer 변경
   - packaging → hand.layout
   - URDF → hand.layout
   - 향후 optimizer → top-level HandMorphologyParameters
이렇게 하는 게 가장 안전해.
그리고 지금 이 설계에서 제일 중요한 결정은 fingertip length를 바꿀 때 무엇을 고정할지인데, 나는 강하게:
\[
\boxed{
\text{tip geometry fixed}
+
\text{carrier distal inset fixed}
+
\text{straight section만 길이 변화}
}
\]를 추천해.
이러면 “모든 fingertip은 같은 morphology이고 전체 길이만 다르다”는 말이 코드에서도 정말 정확하게 성립해.