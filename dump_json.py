import json
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/workspace/automatic_calibration/generated/verify_20260818/calibration_result.json'
with open(path) as f:
    d = json.load(f)

if 'candidate_results' in d:
    print('=== CANDIDATE RESULTS count:', len(d['candidate_results']))
    for i, c in enumerate(d['candidate_results']):
        ang = c.get('angles_deg', {})
        m = c.get('metrics', {})
        print(f"cand {i} ({c.get('search_stage')}): yaw={ang.get('yaw')}, pitch={ang.get('pitch')}, roll={ang.get('roll')}, success={c.get('success')}, state={c.get('state')}, reason={c.get('reason_code')}, gate={c.get('internal_gate_pass')}, conf={c.get('multi_criteria_confidence_score')}, edge={m.get('visible_edge_points')}, nid={m.get('nid_projected_points')}, tesl={m.get('total_explained_structural_length')}, asym={m.get('asymmetric_structural_weight')}")
