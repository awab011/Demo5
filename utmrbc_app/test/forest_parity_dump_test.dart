// Forest-planner parity dumper (NOT a real unit test — a corpus generator).
//
// Reads $PARITY_DIR/inputs.json, runs the REAL on-device planner
// (CalPath.calculate + buildCanPath) over every case, and writes the results
// to $PARITY_DIR/dart_out.json. tools/forest_parity_check.py (in Demo5_laptop)
// then runs the Python port over the same inputs and diffs the two — that is
// how Step 3's "output must match cal_path.dart" is actually verified.
//
//   PARITY_DIR=/tmp/forest_parity flutter test test/forest_parity_dump_test.dart
import 'dart:convert';
import 'dart:io';

import 'package:flutter_test/flutter_test.dart';
import 'package:utmrbc_mf/services/cal_path.dart';

void main() {
  test('dump forest parity corpus', () {
    final dir = Platform.environment['PARITY_DIR'] ?? '/tmp/forest_parity';
    final inputs =
        json.decode(File('$dir/inputs.json').readAsStringSync()) as List;

    final out = <Map<String, dynamic>>[];
    for (final raw in inputs) {
      final c = raw as Map<String, dynamic>;
      final states = (c['states'] as List).cast<int>();
      final team = c['team'] as String;
      final kfs = c['kfs_count'] as int;
      final picked = (c['already_picked'] as List).cast<int>();

      final result = CalPath.calculate(
        states,
        team,
        kfsCount: kfs,
        alreadyPicked: picked.isEmpty ? null : picked,
      );
      out.add(CalPath.buildCanPath(result, states).toJson());
    }

    File('$dir/dart_out.json').writeAsStringSync(json.encode(out));
    expect(out.length, inputs.length);
  });
}
