//not complete no need use also juz ignore
import 'package:flutter/material.dart';
import 'dart:async';
import '../services/ros2_client.dart';
import '../widgets/camera_feed.dart';
import 'swipe_page.dart';

const String _assemblyTopic = '/assembly_status';
const String _positionTopic = '/robot_position';
const String _mcServerUrl   = 'http://10.161.8.61:5000';

class MCPage extends StatefulWidget {
  const MCPage({super.key});
  @override
  State<MCPage> createState() => _MCPageState();
}

class _MCPageState extends State<MCPage> {
  String _selectedTeam = 'red';
  bool   _assembled    = false;
  String _robotStatus  = 'Waiting...';
  double _robotX       = -1;
  double _robotY       = -1;
  Timer? _rosSubTimer;

  @override
  void initState() 
  {
    super.initState();
    ROS2Client.onMessageReceived = (topic, msg) {
      if (!mounted) return;
      if (topic == _assemblyTopic) {
        final data = msg['data']?.toString() ?? '';
        setState(() {
          _robotStatus = data;
          if (data.toLowerCase().contains('assembled') ||
              data.toLowerCase().contains('done') || data == '1') {
            _assembled = true;
          }
        });
      } else if (topic == _positionTopic) {
        _parsePosition(msg['data']?.toString() ?? '');
      }
    };
    _rosSubTimer = Timer.periodic(const Duration(seconds: 2), (_) {
      if (ROS2Client.isConnected) {
        ROS2Client.subscribe(_assemblyTopic, 'std_msgs/String');
        ROS2Client.subscribe(_positionTopic, 'std_msgs/String');
        _rosSubTimer?.cancel();
      }
    });
  }

  void _parsePosition(String data) 
  {
    try {
      final parts = data.split(',');
      final x = double.parse(parts[0].split(':')[1]);
      final y = double.parse(parts[1].split(':')[1]);
      setState(() { _robotX = x; _robotY = y; });
    } catch (_) {}
  }

  @override
  void dispose() 
  {
    _rosSubTimer?.cancel();
    ROS2Client.unsubscribe(_assemblyTopic);
    ROS2Client.unsubscribe(_positionTopic);
    ROS2Client.onMessageReceived = null;
    super.dispose();
  }

  void _onManualAssembled() => setState(() 
  {
    _assembled   = true;
    _robotStatus = 'Assembled (manual)';
  });

  void _resetAssembled() => setState(() 
  {
    _assembled   = false;
    _robotStatus = 'Waiting...';
  });

  void _showConfirmDialog() 
  {
    showDialog(
      context: context,
      barrierDismissible: false,
      builder: (_) => AlertDialog(
        backgroundColor: const Color(0xFF1e1e2e),
        shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(16)),
        title: const Row(children: [
          Icon(Icons.check_circle, color: Colors.green, size: 26),
          SizedBox(width: 8),
          Text('Assembled! Go to MF?',
              style: TextStyle(color: Colors.white, fontSize: 17)),
        ]),
        content: Text(
          '${_selectedTeam.toUpperCase()} TEAM',
          style: const TextStyle(color: Colors.white60, fontSize: 13)),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('Stay',
                style: TextStyle(color: Colors.grey))),
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.green,
              shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8))),
            onPressed: () {
              Navigator.pop(context);
              Navigator.pushReplacement(context, MaterialPageRoute(
                  builder: (_) => const SwipePage(initialPage: 1)));
            },
            child: const Text('Go to MF →',
                style: TextStyle(
                    color: Colors.white,
                    fontWeight: FontWeight.bold))),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) 
  {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: const Color.fromARGB(255, 101, 240, 250),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back),
          onPressed: () => Navigator.pop(context),
        ),
        title: const Text('MC Zone'),
        actions: [
          // Team dropdown in appbar
          Padding(
            padding: const EdgeInsets.only(right: 8),
            child: _dropTeam(),
          ),
          // ROS2 status
          Padding(
            padding: const EdgeInsets.only(right: 12),
            child: Center(
              child: Row(mainAxisSize: MainAxisSize.min, children: [
                Icon(
                  ROS2Client.isConnected ? Icons.wifi : Icons.wifi_off,
                  color: ROS2Client.isConnected
                      ? Colors.white : Colors.red.shade200,
                  size: 16),
                const SizedBox(width: 4),
                Text(
                  ROS2Client.isConnected ? 'ROS2 ✓' : 'ROS2 ✕',
                  style: TextStyle(
                    color: ROS2Client.isConnected
                        ? Colors.white : Colors.red.shade200,
                    fontSize: 12,
                    fontWeight: FontWeight.w600)),
              ]),
            ),
          ),
        ],
      ),

      backgroundColor: const Color(0xFF12121f),
      body: SafeArea(
        child: LayoutBuilder(builder: (ctx, box) {
          final pad = (box.maxWidth * 0.020).clamp(6.0, 12.0);
          final fs  = (box.maxWidth * 0.026).clamp(10.0, 13.0);

          return Column(children: [
            Expanded(
              flex: 65,
              child: Padding(
                padding: EdgeInsets.fromLTRB(pad, pad * 0.8, pad, pad * 0.4),
                child: Row(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    // Camera
                    Expanded(child: _cameraPanel()),
                    SizedBox(width: pad * 0.7),
                    // Field diagram
                    Expanded(child: _MCZoneDiagram(
                      team: _selectedTeam,
                      assembled: _assembled,
                      robotX: _robotX,
                      robotY: _robotY,
                    )),
                  ],
                ),
              ),
            ),

            Expanded(
              flex: 35,
              child: Padding(
                padding: EdgeInsets.fromLTRB(pad, 0, pad, pad),
                child: _bottomPanel(fs, pad),
              ),
            ),
          ]);
        }),
      ),
    );
  }

  Widget _cameraPanel() => Container(
    decoration: BoxDecoration(
      borderRadius: BorderRadius.circular(10),
      border: Border.all(
        color: _selectedTeam == 'red'
            ? const Color(0xFFc0392b) : const Color(0xFF2980b9),
        width: 2.5)),
    child: ClipRRect(
      borderRadius: BorderRadius.circular(8),
      child: Stack(children: [
        const CameraFeedWidget(
          serverUrl: _mcServerUrl,
          width: double.infinity,
          height: double.infinity,
          showControls: true,
        ),
        Positioned(top: 8, left: 8,
          child: Container(
            padding: const EdgeInsets.symmetric(
                horizontal: 7, vertical: 3),
            decoration: BoxDecoration(
              color: Colors.red.withOpacity(0.85),
              borderRadius: BorderRadius.circular(4)),
            child: const Row(mainAxisSize: MainAxisSize.min, children: [
              Icon(Icons.circle, color: Colors.white, size: 6),
              SizedBox(width: 4),
              Text('LIVE', style: TextStyle(color: Colors.white,
                  fontSize: 9, fontWeight: FontWeight.bold)),
            ]))),
        if (_assembled)
          Positioned.fill(
            child: Container(
              decoration: BoxDecoration(
                border: Border.all(color: Colors.green, width: 3),
                borderRadius: BorderRadius.circular(8)),
              child: Align(
                alignment: Alignment.bottomCenter,
                child: Container(
                  margin: const EdgeInsets.only(bottom: 10),
                  padding: const EdgeInsets.symmetric(
                      horizontal: 14, vertical: 6),
                  decoration: BoxDecoration(
                    color: Colors.green.withOpacity(0.92),
                    borderRadius: BorderRadius.circular(8)),
                  child: const Text('ASSEMBLED ✓',
                      style: TextStyle(color: Colors.white,
                          fontWeight: FontWeight.bold,
                          fontSize: 12)))))),
      ]),
    ),
  );

  Widget _bottomPanel(double fs, double pad) => Row(
    crossAxisAlignment: CrossAxisAlignment.stretch,
    children: [
      Expanded(
        flex: 55,
        child: Container(
          padding: EdgeInsets.all(pad * 0.9),
          decoration: BoxDecoration(
            color: const Color(0xFF1e1e2e),
            borderRadius: BorderRadius.circular(10),
            border: Border.all(color: Colors.white12)),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('ROBOT POSITION', style: TextStyle(
                  color: Colors.white38, fontSize: fs * 0.78,
                  fontWeight: FontWeight.bold, letterSpacing: 1.2)),
              SizedBox(height: pad * 0.5),

              Row(children: [
                Container(width: 10, height: 10,
                  decoration: BoxDecoration(
                    color: _robotX >= 0 ? Colors.green : Colors.orange,
                    shape: BoxShape.circle,
                    boxShadow: [BoxShadow(
                      color: (_robotX >= 0
                          ? Colors.green : Colors.orange).withOpacity(0.5),
                      blurRadius: 5)])),
                SizedBox(width: pad * 0.5),
                Text(
                  _robotX >= 0
                      ? 'x: ${_robotX.toStringAsFixed(3)}   '
                        'y: ${_robotY.toStringAsFixed(3)}'
                      : 'No position data',
                  style: TextStyle(
                    color: _robotX >= 0
                        ? Colors.white70 : Colors.white30,
                    fontSize: fs * 0.9,
                    fontFamily: 'monospace')),
              ]),
              SizedBox(height: pad * 0.4),

              Row(children: [
                Container(width: 7, height: 7,
                  decoration: BoxDecoration(
                    color: _assembled ? Colors.green : Colors.orange,
                    shape: BoxShape.circle)),
                SizedBox(width: pad * 0.4),
                Flexible(child: Text(_robotStatus,
                    style: TextStyle(
                        color: _assembled
                            ? Colors.green : Colors.white54,
                        fontSize: fs * 0.85,
                        fontWeight: _assembled
                            ? FontWeight.bold : FontWeight.normal),
                    maxLines: 2,
                    overflow: TextOverflow.ellipsis)),
              ]),
            ],
          ),
        ),
      ),

      SizedBox(width: pad * 0.7),

      Expanded(
        flex: 45,
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // Manual button (only when not assembled)
            if (!_assembled) ...[
              Expanded(
                child: OutlinedButton.icon(
                  onPressed: _onManualAssembled,
                  icon: const Icon(Icons.touch_app, size: 14),
                  label: Text('MANUAL ASSEMBLED',
                      style: TextStyle(fontSize: fs * 0.82)),
                  style: OutlinedButton.styleFrom(
                    foregroundColor: Colors.white54,
                    side: const BorderSide(color: Colors.white24),
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(8)),
                  ),
                ),
              ),
              SizedBox(height: pad * 0.4),
            ],

            Expanded(
              flex: 2,
              child: GestureDetector(
                onTap: _assembled ? _showConfirmDialog : null,
                onLongPress: _assembled ? _resetAssembled : null,
                child: AnimatedContainer(
                  duration: const Duration(milliseconds: 300),
                  decoration: BoxDecoration(
                    color: _assembled
                        ? Colors.green : const Color(0xFF1e1e2e),
                    borderRadius: BorderRadius.circular(10),
                    border: Border.all(
                      color: _assembled
                          ? Colors.green : Colors.white24,
                      width: 2),
                    boxShadow: _assembled
                        ? [BoxShadow(
                            color: Colors.green.withOpacity(0.4),
                            blurRadius: 12, spreadRadius: 2)]
                        : [],
                  ),
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Icon(
                        _assembled
                            ? Icons.check_circle
                            : Icons.radio_button_unchecked,
                        color: _assembled
                            ? Colors.white : Colors.white24,
                        size: 22),
                      const SizedBox(height: 4),
                      Text(
                        _assembled ? 'COMPLETE' : 'NOT YET',
                        style: TextStyle(
                          color: _assembled
                              ? Colors.white : Colors.white24,
                          fontWeight: FontWeight.bold,
                          fontSize: fs,
                          letterSpacing: 1.5)),
                      if (_assembled)
                        Text('long press to reset',
                            style: TextStyle(
                                color: Colors.white38,
                                fontSize: fs * 0.68)),
                    ],
                  ),
                ),
              ),
            ),
          ],
        ),
      ),
    ],
  );

  Widget _dropTeam() => Container(
    padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 2),
    decoration: BoxDecoration(
      color: _selectedTeam == 'red'
          ? const Color(0xFF3a0a0a) : const Color(0xFF0a1a3a),
      borderRadius: BorderRadius.circular(5),
      border: Border.all(
        color: _selectedTeam == 'red'
            ? const Color(0xFFc0392b) : const Color(0xFF2980b9),
        width: 2)),
    child: DropdownButtonHideUnderline(
      child: DropdownButton<String>(
        value: _selectedTeam,
        dropdownColor: const Color(0xFF1e1e2e),
        isDense: true,
        style: TextStyle(
          color: _selectedTeam == 'red'
              ? const Color(0xFFe74c3c) : const Color(0xFF3498db),
          fontWeight: FontWeight.bold, fontSize: 13),
        items: const [
          DropdownMenuItem(value: 'red',  child: Text('RED')),
          DropdownMenuItem(value: 'blue', child: Text('BLUE')),
        ],
        onChanged: (v) => setState(() => _selectedTeam = v!),
      ),
    ),
  );
}

class _MCZoneDiagram extends StatelessWidget 
{
  final String team;
  final bool assembled;
  final double robotX, robotY;

  const _MCZoneDiagram({
    required this.team,
    required this.assembled,
    required this.robotX,
    required this.robotY,
  });

  @override
  Widget build(BuildContext context) 
  {
    final isRed = team == 'red';
    return Container(
      decoration: BoxDecoration(
        color: const Color(0xFF1a1a2e),
        borderRadius: BorderRadius.circular(10),
        border: Border.all(
          color: isRed
              ? const Color(0xFFc0392b).withOpacity(0.6)
              : const Color(0xFF2980b9).withOpacity(0.6),
          width: 2,
        ),
      ),
      child: Column(children: [
        Container(
          padding: const EdgeInsets.symmetric(vertical: 5),
          decoration: BoxDecoration(
            color: isRed
                ? const Color(0xFFc0392b).withOpacity(0.25)
                : const Color(0xFF2980b9).withOpacity(0.25),
            borderRadius:
                const BorderRadius.vertical(top: Radius.circular(8)),
          ),
          child: Center(
            child: Text(
              'MC ZONE — ${team.toUpperCase()} TEAM',
              style: const TextStyle(
                color: Colors.white60,
                fontSize: 9.5,
                fontWeight: FontWeight.bold,
                letterSpacing: 1.1,
              ),
            ),
          ),
        ),
        Expanded(
          child: ClipRRect(
            borderRadius:
                const BorderRadius.vertical(bottom: Radius.circular(8)),
            child: CustomPaint(
              painter: _MCZonePainter(
                team: team,
                assembled: assembled,
                robotX: robotX,
                robotY: robotY,
              ),
              size: Size.infinite,
            ),
          ),
        ),
      ]),
    );
  }
}

class _MCZonePainter extends CustomPainter 
{
  final String team;
  final bool assembled;
  final double robotX, robotY;

  const _MCZonePainter({
    required this.team,
    required this.assembled,
    required this.robotX,
    required this.robotY,
  });

  Paint _fill(Color c) =>
      Paint()..color = c..style = PaintingStyle.fill;

  Paint _stroke(Color c, double lw) => Paint()..color = c..style = PaintingStyle.stroke..strokeWidth = lw;

  void _label(Canvas c, String t, Offset o, double sz, Color col,
      {FontWeight fw = FontWeight.bold}) {
    final tp = TextPainter(
      text: TextSpan(
          text: t, style: TextStyle(color: col, fontSize: sz, fontWeight: fw)),
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    )..layout();
    tp.paint(c, Offset(o.dx - tp.width / 2, o.dy - tp.height / 2));
  }

  void _box(Canvas canvas, Rect r, Color fill, Color border, String label,
      {double fontSize = 8.5}) {
    canvas.drawRRect(
        RRect.fromRectAndRadius(r, const Radius.circular(4)),
        _fill(fill.withOpacity(0.35)));
    canvas.drawRRect(
        RRect.fromRectAndRadius(r, const Radius.circular(4)),
        _stroke(border.withOpacity(0.85), 1.8));
    // multi-line label
    final lines = label.split('\n');
    final lineH = fontSize * 1.4;
    final totalH = lines.length * lineH;
    for (int i = 0; i < lines.length; i++) {
      final dy = r.center.dy - totalH / 2 + i * lineH + lineH / 2;
      _label(canvas, lines[i], Offset(r.center.dx, dy), fontSize,
          Colors.white);
    }
  }

  void _robotDot(Canvas canvas, Offset pos, Color color, String label,
      double dotR) {
    // glow
    canvas.drawCircle(
        pos,
        dotR + 4,
        Paint()
          ..color = color.withOpacity(0.18)
          ..maskFilter =
              const MaskFilter.blur(BlurStyle.normal, 6));
    canvas.drawCircle(pos, dotR, _fill(color));
    canvas.drawCircle(
        pos, dotR, _stroke(Colors.white.withOpacity(0.45), 1.5));
    _label(canvas, label, pos + Offset(0, dotR + 9), 8, color);
  }

  @override
  void paint(Canvas canvas, Size s) 
  {
    final w = s.width;
    final h = s.height;
    final isRed = team == 'red';

    final teamColor =
        isRed ? const Color(0xFFc0392b) : const Color(0xFF2980b9);
    final bgColor = isRed
        ? const Color(0xFFf4a0a0).withOpacity(0.08)
        : const Color(0xFF8ecae6).withOpacity(0.08);

    canvas.drawRRect(
        RRect.fromRectAndRadius(
            Rect.fromLTWH(0, 0, w, h), const Radius.circular(6)),
        _fill(bgColor));

    final gridP = Paint()
      ..color = Colors.white.withOpacity(0.04)
      ..strokeWidth = 0.5;
    for (double x = 0; x < w; x += w / 8) {
      canvas.drawLine(Offset(x, 0), Offset(x, h), gridP);
    }
    for (double y = 0; y < h; y += h / 6) {
      canvas.drawLine(Offset(0, y), Offset(w, y), gridP);
    }

    final topH = h * 0.26;
    final r1W = w * 0.15;
    final staffW = w * 0.38;
    final r2W = w * 0.15;
    final spearW = w * 0.14;
    final pathW = w * 0.09;

    if (isRed) 
    {

      final r1Rect = Rect.fromLTWH(0, 0, r1W, topH);
      canvas.drawRect(r1Rect, _fill(teamColor.withOpacity(0.45)));
      canvas.drawRect(r1Rect, _stroke(teamColor, 1.2));
      canvas.drawCircle(
          Offset(r1W / 2, topH * 0.28), 5,
          _fill(Colors.green.withOpacity(0.85)));
      _label(canvas, 'R1', Offset(r1W / 2, topH * 0.58), 8.5,
          Colors.white);
      _label(canvas, 'START', Offset(r1W / 2, topH * 0.78), 7,
          Colors.white70,
          fw: FontWeight.normal);

      final staffRect =
          Rect.fromLTWH(r1W, 0, staffW, topH);
      _box(canvas, staffRect, const Color(0xFF8B6914),
          const Color(0xFFc8961e), 'STAFF RACK',
          fontSize: 9.0);

      final r2Left = r1W + staffW;
      final r2Rect = Rect.fromLTWH(r2Left, 0, r2W, topH);
      canvas.drawRect(r2Rect,
          _fill(teamColor.withOpacity(0.22)));
      canvas.drawRect(
          r2Rect, _stroke(teamColor.withOpacity(0.6), 0.8));
      canvas.drawCircle(
          Offset(r2Left + r2W / 2, topH * 0.28), 5,
          _fill(Colors.green.withOpacity(0.85)));
      _label(canvas, 'R2',
          Offset(r2Left + r2W / 2, topH * 0.58), 8.5,
          const Color(0xFFffaaaa));
      _label(canvas, 'START',
          Offset(r2Left + r2W / 2, topH * 0.78), 7,
          Colors.white54,
          fw: FontWeight.normal);

      final spearLeft = r2Left + r2W;
      final spearRect =
          Rect.fromLTWH(spearLeft, 0, spearW, topH * 1.7);
      _box(canvas, spearRect, const Color(0xFF7a4510),
          const Color(0xFFc8961e), 'SPEAR\nHEAD\nRACK',
          fontSize: 8.0);

      final pathRect =
          Rect.fromLTWH(0, topH, pathW, h - topH);
      canvas.drawRect(
          pathRect, _fill(teamColor.withOpacity(0.06)));
      canvas.drawRect(
          pathRect,
          _stroke(teamColor.withOpacity(0.25), 0.5));
      _rotatedLabel(canvas, 'R1 PATHWAY',
          Offset(pathW / 2, topH + (h - topH) / 2), 7.5,
          teamColor.withOpacity(0.5));

      final entranceRect =
          Rect.fromLTWH(pathW, topH, w - pathW - spearW, h * 0.14);
      canvas.drawRect(entranceRect,
          _fill(teamColor.withOpacity(0.06)));
      canvas.drawRect(
          entranceRect,
          _stroke(teamColor.withOpacity(0.25), 0.5));
      _label(
          canvas,
          'R2 ENTRANCE / R1 PATHWAY',
          Offset(pathW + (w - pathW - spearW) / 2,
              topH + h * 0.07),
          7.0,
          teamColor.withOpacity(0.55),
          fw: FontWeight.normal);

      final retryRect =
          Rect.fromLTWH(0, h - h * 0.13, w * 0.28, h * 0.13);
      canvas.drawRect(
          retryRect, _fill(teamColor.withOpacity(0.2)));
      canvas.drawRect(retryRect,
          _stroke(teamColor.withOpacity(0.45), 0.5));
      _label(canvas, 'RETRY ZONE',
          Offset(w * 0.14, h - h * 0.065), 7.5,
          teamColor.withOpacity(0.8),
          fw: FontWeight.normal);

      final r1DotPos = Offset(w * 0.42, h * 0.65);
      _robotDot(canvas, r1DotPos,
          assembled ? Colors.green : const Color(0xFF4ae6e6),
          'R1', 9);
    } else {

      final spearLeft = 0.0;
      final spearRect = Rect.fromLTWH(
          spearLeft, 0, spearW, topH * 1.7);
      _box(canvas, spearRect, const Color(0xFF1565C0),
          const Color(0xFF42A5F5), 'SPEAR\nHEAD\nRACK',
          fontSize: 8.0);

      final r2Left = spearW;
      final r2Rect = Rect.fromLTWH(r2Left, 0, r2W, topH);
      canvas.drawRect(r2Rect,
          _fill(teamColor.withOpacity(0.22)));
      canvas.drawRect(
          r2Rect, _stroke(teamColor.withOpacity(0.6), 0.8));
      canvas.drawCircle(
          Offset(r2Left + r2W / 2, topH * 0.28), 5,
          _fill(Colors.green.withOpacity(0.85)));
      _label(canvas, 'R2',
          Offset(r2Left + r2W / 2, topH * 0.58), 8.5,
          const Color(0xFFaaccff));
      _label(canvas, 'START',
          Offset(r2Left + r2W / 2, topH * 0.78), 7,
          Colors.white54,
          fw: FontWeight.normal);

      final staffLeft = spearW + r2W;
      final staffRect =
          Rect.fromLTWH(staffLeft, 0, staffW, topH);
      _box(canvas, staffRect, const Color(0xFF8B6914),
          const Color(0xFFc8961e), 'STAFF RACK',
          fontSize: 9.0);

      final r1Left = staffLeft + staffW;
      final r1Rect = Rect.fromLTWH(r1Left, 0, r1W, topH);
      canvas.drawRect(
          r1Rect, _fill(teamColor.withOpacity(0.45)));
      canvas.drawRect(r1Rect, _stroke(teamColor, 1.2));
      canvas.drawCircle(
          Offset(r1Left + r1W / 2, topH * 0.28), 5,
          _fill(Colors.green.withOpacity(0.85)));
      _label(canvas, 'R1',
          Offset(r1Left + r1W / 2, topH * 0.58), 8.5,
          Colors.white);
      _label(canvas, 'START',
          Offset(r1Left + r1W / 2, topH * 0.78), 7,
          Colors.white70,
          fw: FontWeight.normal);

      final pathLeft = w - pathW;
      final pathRect =
          Rect.fromLTWH(pathLeft, topH, pathW, h - topH);
      canvas.drawRect(
          pathRect, _fill(teamColor.withOpacity(0.06)));
      canvas.drawRect(
          pathRect,
          _stroke(teamColor.withOpacity(0.25), 0.5));
      _rotatedLabel(canvas, 'R1 PATHWAY',
          Offset(pathLeft + pathW / 2, topH + (h - topH) / 2),
          7.5, teamColor.withOpacity(0.5));

      final entranceRect = Rect.fromLTWH(
          spearW, topH, w - spearW - pathW, h * 0.14);
      canvas.drawRect(entranceRect,
          _fill(teamColor.withOpacity(0.06)));
      canvas.drawRect(
          entranceRect,
          _stroke(teamColor.withOpacity(0.25), 0.5));
      _label(
          canvas,
          'R2 ENTRANCE / R1 PATHWAY',
          Offset(spearW + (w - spearW - pathW) / 2,
              topH + h * 0.07),
          7.0,
          teamColor.withOpacity(0.55),
          fw: FontWeight.normal);

      final retryRect = Rect.fromLTWH(
          w - w * 0.28, h - h * 0.13, w * 0.28, h * 0.13);
      canvas.drawRect(
          retryRect, _fill(teamColor.withOpacity(0.2)));
      canvas.drawRect(retryRect,
          _stroke(teamColor.withOpacity(0.45), 0.5));
      _label(canvas, 'RETRY ZONE',
          Offset(w - w * 0.14, h - h * 0.065), 7.5,
          teamColor.withOpacity(0.8),
          fw: FontWeight.normal);

      final r1DotPos = Offset(w * 0.58, h * 0.65);
      _robotDot(canvas, r1DotPos,
          assembled ? Colors.green : const Color(0xFF4ae6e6),
          'R1', 9);
    }

    if (robotX >= 0 && robotY >= 0) {
      final livePos = Offset(robotX * w, robotY * h);
      canvas.drawCircle(
          livePos,
          15,
          Paint()
            ..color = Colors.green.withOpacity(0.15)
            ..maskFilter =
                const MaskFilter.blur(BlurStyle.normal, 10));
      canvas.drawCircle(
          livePos, 9, _fill(Colors.green.withOpacity(0.9)));
      canvas.drawCircle(livePos, 9,
          _stroke(Colors.white.withOpacity(0.7), 2));
      _label(canvas, 'LIVE',
          livePos + const Offset(0, 18), 8, Colors.green);
    }

    if (assembled) {
      final cx = w * 0.50;
      final cy = h * 0.82;
      canvas.drawRRect(
          RRect.fromRectAndRadius(
              Rect.fromLTWH(
                  cx - w * 0.24, cy - h * 0.07, w * 0.48, h * 0.14),
              const Radius.circular(6)),
          _fill(Colors.green.withOpacity(0.88)));
      _label(canvas, '✓  ASSEMBLED', Offset(cx, cy), 11,
          Colors.white);
    }
  }
  void _rotatedLabel(Canvas canvas, String text, Offset center,
      double fontSize, Color color) {
    canvas.save();
    canvas.translate(center.dx, center.dy);
    canvas.rotate(-3.14159 / 2);
    _label(canvas, text, Offset.zero, fontSize, color,
        fw: FontWeight.normal);
    canvas.restore();
  }

  @override
  bool shouldRepaint(_MCZonePainter old) =>
      old.team != team ||
      old.assembled != assembled ||
      old.robotX != robotX ||
      old.robotY != robotY;
}