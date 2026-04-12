Reviewed by Opus agent with no production code context — only test files read.

## Overall verdict
Integration test suite with a critical structural flaw: ordered sequence masquerading as independent tests. Functional but brittle.

## Key findings

**Critical — Isolation (POOR)**
Tests are an intentionally ordered sequence. `test_list_objects_held` only passes because `test_pick_blue_cylinder` ran before it. One mid-sequence failure cascades and fails everything downstream — hard to tell what actually broke.

**High — Flakiness risk**
90s startup timeout, 60s per-command timeout, `ros2 topic echo` with 30s timeout, fragile `pkill -f` teardown. All timing-dependent.

**Mixed — Assertions**
- Good: `_exact` position checks with tolerances, confidence checks in vision tests.
- Bad: several tests only assert `rc == 0` (did it not crash?), fragile substring matches like `"-0.100" in out` instead of using the already-available `parse_object_position()` helper.

**Adequate — Error case coverage**
Error section is decent but missing: double-pick, pick-while-holding, place-at-collision, invalid args.

## Specific tests to fix

| Test | Issue |
|---|---|
| `test_list_objects_held` | depends entirely on prior test |
| `test_verify_blue_on_basket` | fragile `"-0.100" in out` substring match — use `parse_object_position` |
| `test_verify_red_on_basket` | same issue: `"-0.050" in out` |
| `test_verify_positions_after_reset` | weaker duplicate of `_exact` variant — remove it |
| `test_move_to` | exact duplicate of `test_move_to_negative_y` — remove one |
| `test_detected_positions_match_scene` | fragile raw `ros2 topic echo` output parsing, high flakiness risk |
| `test_pick_emits_feedback` | does a restore place internally — silent failure risk |

## Recommended refactor scope
1. Replace ordered sequence with fixtures that establish preconditions per-test (or explicit scenario test).
2. Replace all substring position checks with `parse_object_position()` + tolerance.
3. Remove `test_verify_positions_after_reset` (redundant with `_exact` variant).
4. Remove duplicate `test_move_to`.
5. Error tests: assert on error message content, not just non-zero exit code.
6. Add negative tests: double-pick, pick-while-holding, invalid argument types.
