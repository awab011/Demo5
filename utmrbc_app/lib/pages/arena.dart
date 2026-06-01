import 'dart:async';
import 'dart:convert';
import 'package:flutter/material.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

enum _BTS { success, failure }

abstract class _BTNode 
{
  _BTS tick();
}

class _Sel extends _BTNode 
{
  final List<_BTNode> c;
  _Sel(this.c);
  @override
  _BTS tick() {
    for (final n in c) {
      if (n.tick() == _BTS.success) return _BTS.success;
    }
    return _BTS.failure;
  }
}

class _Seq extends _BTNode 
{
  final List<_BTNode> c;
  _Seq(this.c);
  @override
  _BTS tick() {
    for (final n in c) {
      if (n.tick() == _BTS.failure) return _BTS.failure;
    }
    return _BTS.success;
  }
}

class _Cond extends _BTNode 
{
  final bool Function() f;
  _Cond(this.f);
  @override
  _BTS tick() => f() ? _BTS.success : _BTS.failure;
}

class _Act extends _BTNode 
{
  final void Function() f;
  _Act(this.f);
  @override
  _BTS tick() {
    f();
    return _BTS.success;
  }
}

class ArenaPage extends StatefulWidget 
{
  const ArenaPage({super.key});
  @override
  State<ArenaPage> createState() => _ArenaPageState();
}

class _ArenaPageState extends State<ArenaPage> with TickerProviderStateMixin 
{
  List<int> boardState = List.filled(9, 0); // 0=empty 1=own 2=opp

  int staffCount = 2;
  int pushesUsed = 0;
  int r2Max = 3;
  int r1Max = 3;
  String teamTheme = 'red';
  String inputMode = 'opp';
  Map<String, dynamic>? currentDecision;

  static const bool useRos2 = false; 

  WebSocketChannel? _wsChannel;
  StreamSubscription? _wsSub;
  bool _rosConnected = false;
  //check if robot is moving
  bool _robotBusy = false;

  String _serverIp = '172.20.10.2';
  static const String _visTopic = '/internal/vision_data';

  late AnimationController _pulseCtrl, _flashCtrl;
  late Animation<double> _pulseAnim, _flashAnim;
  late TextEditingController _staffCtrl, _r2Ctrl, _r1Ctrl;

  static const List<List<int>> _lines = 
  [
    [0, 3, 6], [1, 4, 7], [2, 5, 8], [0, 4, 8], [2, 4, 6],
  ];

  @override
  void initState() 
  {
    super.initState();
    _staffCtrl = TextEditingController(text: '$staffCount');
    _r2Ctrl = TextEditingController(text: '$r2Max');
    _r1Ctrl = TextEditingController(text: '$r1Max');
    _pulseCtrl = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 1500),
    )..repeat(reverse: true);
    _pulseAnim = Tween<double>(begin: 1.0, end: 1.05)
        .animate(CurvedAnimation(parent: _pulseCtrl, curve: Curves.easeInOut));
        
    _flashCtrl = AnimationController(
      vsync: this,
      duration: const Duration(milliseconds: 900),
    )..repeat(reverse: true);
    _flashAnim = Tween<double>(begin: 1.0, end: 0.3)
        .animate(CurvedAnimation(parent: _flashCtrl, curve: Curves.easeInOut));

    WidgetsBinding.instance.addPostFrameCallback((_) {
      _toggleNetworkConnection();
    });

    currentDecision = _formatDecision(4, 'Opening: Take Center!', null);
  }

  @override
  void dispose() 
  {
    _pulseCtrl.dispose();
    _flashCtrl.dispose();
    _staffCtrl.dispose();
    _r2Ctrl.dispose();
    _r1Ctrl.dispose();
    _wsSub?.cancel();
    _wsChannel?.sink.close();
    super.dispose();
  }

  Future<void> _changeIpDialog() async {
    final ctrl = TextEditingController(text: _serverIp);
    final newIp = await showDialog<String>(
      context: context,
      builder: (_) => AlertDialog(
        backgroundColor: const Color(0xFF2c2c2c),
        title: const Text('Set Robot IP', style: TextStyle(color: Colors.white, fontSize: 14)),
        content: TextField(
          controller: ctrl,
          autofocus: true,
          style: const TextStyle(color: Colors.white),
          decoration: const InputDecoration(
            hintText: 'e.g. 172.20.10.2', //hotspot ip
            hintStyle: TextStyle(color: Colors.white38),
            enabledBorder: UnderlineInputBorder(borderSide: BorderSide(color: Colors.green)),
            focusedBorder: UnderlineInputBorder(borderSide: BorderSide(color: Colors.green, width: 2)),
          ),
        ),
        actions: [
          TextButton(
            onPressed: () => Navigator.pop(context),
            child: const Text('CANCEL', style: TextStyle(color: Colors.red)),
          ),
          TextButton(
            onPressed: () => Navigator.pop(context, ctrl.text.trim()),
            child: const Text('CONNECT', style: TextStyle(color: Colors.green)),
          ),
        ],
      ),
    );
    if (newIp != null && newIp.isNotEmpty) 
    {
      setState(() => _serverIp = newIp);
      if (_rosConnected) 
      {
        await _toggleNetworkConnection(); // Disconnect
        _toggleNetworkConnection(); // Reconnect with new IP
      } 
      else 
      {
        _toggleNetworkConnection();
      }
    }
  }

  void _syncTeamColor() 
  {
    if (_wsChannel != null && _rosConnected) 
    {
      dynamic colorPayload;
      if (useRos2) 
      {
        colorPayload = json.encode({
          'op': 'publish',
          'topic': '/team_config',
          'msg': { 'data': teamTheme }
        });
      } 
      else 
      {
        colorPayload = json.encode({
          'type': 'team_config',
          'color': teamTheme
        });
      }
      _wsChannel!.sink.add(colorPayload);
    }
  }

  Future<void> _toggleNetworkConnection() async {
    if (_rosConnected) 
    {
      await _wsSub?.cancel();
      await _wsChannel?.sink.close();
      if (mounted) setState(() => _rosConnected = false);
      return;
    }
    try 
    {
      final targetUrl = 'ws://$_serverIp:9090';
      _wsChannel = WebSocketChannel.connect(Uri.parse(targetUrl));
      await _wsChannel!.ready.timeout(const Duration(seconds: 5));

      if (useRos2) 
      {
        //subscribe to incoming data
        _wsChannel!.sink.add(json.encode({
          'op': 'subscribe',
          'topic': _visTopic,
          'type': 'utmrbc_interfaces/msg/ArenaArray',
        }));

        _wsChannel!.sink.add(json.encode({
          'op': 'advertise',
          'topic': '/team_config',
          'type': 'std_msgs/msg/String',
        }));
        _wsChannel!.sink.add(json.encode({
          'op': 'advertise',
          'topic': '/robot_paths',
          'type': 'std_msgs/msg/UInt8MultiArray',
        }));
      }

      _wsSub = _wsChannel!.stream.listen(
        (data) {
          try {
            //listen to mainboard
            if (data.toString().contains("KFS_DONE")) 
            {
                if (mounted) 
                {
                    setState(() 
                    {
                        _robotBusy = false; //continue use cam see
                        _calculateDecision();
                    });
                }
                return;
            }

            final msg = json.decode(data as String) as Map<String, dynamic>;
            if (useRos2) {
              if (msg['op'] == 'publish' && msg['topic'] == _visTopic) 
              {
                _handleVisionData(msg['msg'] as Map<String, dynamic>);
              }
            } 
            else 
            {
              if (msg['topic'] == _visTopic) 
              {
                _handleVisionData(msg);
              }
            }
          } catch (_) {}
        },
        onError: (_) { if (mounted) setState(() => _rosConnected = false); },
        onDone: () { if (mounted) setState(() => _rosConnected = false); },
      );
      
      if (mounted) 
      {
        setState(() => _rosConnected = true);
        Future.delayed(const Duration(milliseconds: 500), () {
          _syncTeamColor();
        });
      }
    } catch (_) 
    {
      if (mounted) setState(() => _rosConnected = false);
    }
  }

  void _handleVisionData(Map<String, dynamic> msg) {
    if (!mounted) return;
    if (_robotBusy) return;
    
    setState(()
    {
      final blocks = msg['blocks'] as List<dynamic>;
      for (final b in blocks) {
        final idx = (b['block_id'] as int) - 1;
        final bool isOurTeam = b['is_our_team'] ?? b['has_r1_kfs'] ?? false;
        final bool isOppTeam = b['is_opp_team'] ?? b['has_r2_kfs'] ?? false;
        
        if (isOurTeam) 
        {
          boardState[idx] = 1;
        } 
        else if (isOppTeam) 
        {
          boardState[idx] = 2;
        } 
        else 
        {
          boardState[idx] = 0;
        }
      }
      _calculateDecision();
    });
  }

  Color get teamColor => teamTheme == 'red' ? Colors.red : Colors.blue;
  Color get oppColor => teamTheme == 'red' ? Colors.blue : Colors.red;
  bool get _canPush => pushesUsed < staffCount;

  int get _r1Col 
  {
    for (int col = 0; col < 3; col++) 
    {
      if (boardState[6 + col] == 1) return col;
    }
    return -1;
  }

  int? get _r1AboveTarget 
  {
    final col = _r1Col;
    if (col < 0) return null;
    final above = 3 + col;
    return boardState[above] == 0 ? above : null;
  }

  List<int> _validMoves() 
  {
    final topOwn = boardState.sublist(0, 3).where((e) => e == 1).length;
    final midOwn = boardState.sublist(3, 6).where((e) => e == 1).length;
    final totalOwn = topOwn + midOwn;
    if (totalOwn >= r2Max) return [];

    final rowLimit = r2Max == 1 ? 1 : 2;
    final moves = <int>[];

    for (int i = 0; i < 6; i++) 
    {
      final rowFull = i < 3 ? topOwn >= rowLimit : midOwn >= rowLimit;
      if (rowFull) continue;
      if (boardState[i] == 0) 
      {
        moves.add(i);
      } 
      else if (boardState[i] == 2 && _canPush) 
      {
        moves.add(i); 
      }
    }
    return moves;
  }

  int? wanwinliao() 
  {
    if (!_canPush) return null;
    final valid = _validMoves();
    for (final l in _lines) 
    {
      final vals = l.map((i) => boardState[i]).toList();
      final ownCount = vals.where((v) => v == 1).length;
      final oppCount = vals.where((v) => v == 2).length;
      final emptyIdx = vals.indexWhere((v) => v == 0);
      if (ownCount == 1 && oppCount == 1 && emptyIdx >= 0) 
      {
        final emptyCell = l[emptyIdx];
        if (valid.contains(emptyCell)) 
        {
          return l[vals.indexWhere((v) => v == 2)];
        }
      }
    }
    return null;
  }

  int? _blockma() 
  {
    for (final l in _lines) 
    {
      final vals = l.map((i) => boardState[i]).toList();
      if (vals.where((v) => v == 2).length == 2 && vals.contains(0)) 
      {
        return l[vals.indexOf(0)];
      }
    }
    return null;
  }

  int? _pushma() 
  {
    for (final l in _lines) 
    {
      final vals = l.map((i) => boardState[i]).toList();
      if (vals.where((v) => v == 2).length == 2 && vals.contains(0)) 
      {
        return l.firstWhere((i) => boardState[i] == 2);
      }
    }
    return null;
  }

  int? _findWin(List<int> validMoves) 
  {
    for (final l in _lines) 
    {
      final vals = l.map((i) => boardState[i]).toList();
      if (vals.where((v) => v == 1).length == 2 && vals.contains(0)) 
      {
        final t = l[vals.indexOf(0)];
        if (validMoves.contains(t)) return t;
      }
    }
    return null;
  }

  void _calculateDecision() 
  {
    Map<String, dynamic>? result;
    final bt = _Sel([
      _Seq([
        _Cond(() => boardState.every((v) => v == 0)),
        _Act(() { result = _formatDecision(4, 'Take Center', null); }),
      ]),
      _Seq([
        _Cond(() => _validMoves().isNotEmpty),
        _Sel([
          _Seq([
            _Cond(() => _findWin(_validMoves()) != null),
            _Act(() { result = _formatDecision(_findWin(_validMoves())!, 'Kung Fu Master', null); }),
          ]),
          _Seq([
            _Cond(() => wanwinliao() != null),
            _Act(() {
              final pushOpp = wanwinliao()!;
              final best = _findWin(_validMoves()) ?? bestmove(_validMoves());
              result = _formatDecision(best, 'Push to Win!', pushOpp);
            }),
          ]),
          _Seq([
            _Cond(() => _blockma() != null),
            _Sel([
              _Seq([
                _Cond(() => _canPush && _pushma() != null),
                _Act(() {
                  final best = bestmove(_validMoves());
                  final pushIdx = _pushma()!;
                  result = _formatDecision(best, 'Push Opponent KFS', pushIdx);
                }),
              ]),
              _Seq([
                _Cond(() {
                  final t = _blockma();
                  return t != null && _validMoves().contains(t);
                }),
                _Act(() { result = _formatDecision(_blockma()!, 'Block', null); }),
              ]),
            ]),
          ]),
          _Seq([
            _Cond(() {
              final t = _r1AboveTarget;
              return t != null && _validMoves().contains(t);
            }),
            _Act(() {
              final above = _r1AboveTarget;
              if (above != null) result = _formatDecision(above, 'Above R1', null);
            }),
          ]),
          _Act(() {
            final best = bestmove(_validMoves());
            int? pushTarget;
            if (_canPush) {
              for (final l in _lines) {
                if (!l.contains(best)) continue;
                final vals = l.map((i) => boardState[i]).toList();
                if (vals.where((v) => v == 1).length == 1 && vals.where((v) => v == 2).length == 1) {
                  pushTarget = l.firstWhere((i) => boardState[i] == 2);
                  break;
                }
              }
            }
            result = _formatDecision(best, pushTarget != null ? 'Build' : 'Decision Making', pushTarget);
          }),
        ]),
      ]),
      _Act(() { result = _formatDecision(null, 'Idle.', null); }),
    ]);

    bt.tick();
    currentDecision = result;
  }

  int bestmove(List<int> validMoves) 
  {
    int best = validMoves.first;
    double maxScore = -100;
    for (final move in validMoves) 
    {
      double score = 0;
      for (final l in _lines) {
        if (!l.contains(move)) continue;
        final vals = l.map((i) => boardState[i]).toList();
        final own = vals.where((v) => v == 1).length;
        final opp = vals.where((v) => v == 2).length;
        if (own == 1 && opp == 0) 
        {
          score += 5;
        } 
        else if (own == 1 && opp == 1 && _canPush) 
        {
          score += 3;
        }
        else if (own == 0 && opp == 0) 
        {
          score += 1;
        }
      }
      if (move == 4) score += 10;
      if (move >= 3 && move <= 5) score += 2;
      if (move == 1 || move == 3 || move == 5) score += 3;
      if (score > maxScore) { maxScore = score; best = move; }
    }
    return best;
  }

  Map<String, dynamic> _formatDecision(int? slot, String msg, int? pushIdx) 
  {
    const cols = ['Left', 'Center', 'Right'];
    final colName = slot != null ? cols[slot % 3] : '—';
    final r2Act = slot == null ? 'IDLE' : (slot < 3 ? 'Wait @ $colName (Top)' : 'Place @ $colName (Mid)');
    var r1Act = (slot != null && slot < 3) ? 'Lift R2 @ $colName' : 'Free Task';
    if (pushIdx != null) r1Act = 'PUSH opp col ${pushIdx % 3}  →  $r1Act';
    return 
    {
      'slot': slot, 'push_idx': pushIdx, 'r2_action': r2Act, 'r1_action': r1Act, 'message': msg,
    };
  }

  void _cellClicked(int index) 
  {
    setState(() {
      if (index >= 6) {
        boardState[index] = (boardState[index] + 1) % 3;
      } 
      else 
      {
        if (inputMode == 'remove') 
        {
          boardState[index] = 0;
        } 
        else 
        {
          boardState[index] = boardState[index] == 2 ? 0 : 2;
        }
      }
      _calculateDecision();
    });
  }

  void _confirmR2() 
  {
    setState(() 
    {
      final slot = currentDecision?['slot'] as int?;
      if (slot == null || slot >= 6) return;
      
      if (_wsChannel != null && _rosConnected) 
      {
        dynamic payload;
        if (useRos2) {
          payload = json.encode({
            'op': 'publish', 
            'topic': '/robot_paths',
            'msg': { 'data': [slot, 0, 1] }
          });
        } 
        else 
        {
          payload = json.encode({ 'type': 'robot_paths', 'payload': [slot, 0, 1] });
        }
        _wsChannel!.sink.add(payload);
      }

      // NEW: Lock the app until STM32 finishes
      _robotBusy = true;

      if (boardState[slot] == 2 && _canPush) 
      {
        boardState[slot] = 0; pushesUsed++;
      }
      if (boardState[slot] == 0) 
      {
        boardState[slot] = 1; _calculateDecision();
      }
    });
  }

  void _confirmPush() 
  {
    setState(() {
      final pushIdx = currentDecision?['push_idx'] as int?;
      final savedSlot = currentDecision?['slot'];
      final savedMessage = currentDecision?['message'];
      final savedR2 = currentDecision?['r2_action'];
      final savedR1 = currentDecision?['r1_action'];

      if (pushIdx != null && _canPush) {
        if (_wsChannel != null && _rosConnected) 
        {
          dynamic payload;
          if (useRos2) 
          {
            payload = json.encode({
              'op': 'publish', 
              'topic': '/robot_paths',
              'msg': { 'data': [pushIdx, 1, 0] }
            });
          } 
          else 
          {
            payload = json.encode({ 'type': 'robot_paths', 'payload': [pushIdx, 1, 0] });
          }
          _wsChannel!.sink.add(payload);
        }

        // NEW: Lock the app until STM32 finishes
        _robotBusy = true;

        boardState[pushIdx] = 0;
        pushesUsed++;
        _calculateDecision();
        currentDecision = 
        {
          ...currentDecision!, 'slot': savedSlot, 'r2_action': savedR2, 
          'r1_action': savedR1, 'message': savedMessage, 'push_idx': null,
        };
      }
    });
  }

  void _resetBoard() => setState(() 
  {
    boardState = List.filled(9, 0); pushesUsed = 0;
    currentDecision = _formatDecision(4, 'Start: Take Center!', null);
  });

  String get _gameStatus 
  {
    if (boardState.every((v) => v == 0)) return 'start';
    final occupied = boardState.sublist(0, 6).where((e) => e == 1).length;
    if (occupied >= r2Max) return 'end game';
    return 'decision';
  }

  @override
  Widget build(BuildContext context) {
    final hasPush = currentDecision?['push_idx'] != null;

    return Scaffold(
      backgroundColor: const Color(0xFF1e1e1e),
      appBar: AppBar(
        title: const Text('Arena'),
        backgroundColor: const Color.fromARGB(255, 101, 240, 250),
        leading: IconButton(
          icon: const Icon(Icons.arrow_back, color: Colors.black),
          onPressed: () => Navigator.pop(context),
        ),
        actions: [
          Tooltip(
            message: useRos2 ? 'Toggle ROS2 (Long press to set IP)' : 'Toggle websocket (Long press to set IP)',
            child: InkWell(
              onTap: _toggleNetworkConnection,
              onLongPress: _changeIpDialog, // Triggers the popup
              child: Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16.0),
                child: Icon(
                  _rosConnected ? Icons.sensors : Icons.sensors_off,
                  color: _rosConnected ? Colors.green : Colors.red,
                ),
              ),
            ),
          ),
        ],
      ),
      body: SafeArea(
        child: LayoutBuilder(
          builder: (ctx, constraints) 
          {
            final sw = constraints.maxWidth;
            final sh = constraints.maxHeight;
            final gridH = (sh * 0.40).clamp(160.0, 340.0);
            final gridW = gridH;
            final fs = (sw * 0.030).clamp(10.0, 13.0);
            final pad = (sw * 0.03).clamp(7.0, 14.0);

            return Padding(
              padding: EdgeInsets.symmetric(horizontal: pad, vertical: pad * 0.6),
              child: SingleChildScrollView( 
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    _statPanel(fs),
                    SizedBox(height: pad * 0.3),
                    
                    GestureDetector(
                      onLongPress: _changeIpDialog, // Triggers the popup here too
                      child: Container(
                        width: double.infinity,
                        color: _robotBusy 
                            ? Colors.orange.withOpacity(0.2)
                            : (_rosConnected ? Colors.green.withOpacity(0.15) : Colors.red.withOpacity(0.10)),
                        padding: const EdgeInsets.symmetric(vertical: 3),
                        child: Text(
                          _robotBusy
                              ? 'ROBOT BUSY - WAITING KFS_DONE'
                              : (_rosConnected
                                  ? (useRos2 ? 'Connected w Cam (ROS2) - $_serverIp' : 'Connected w Cam (WS) - $_serverIp')
                                  : 'Not connected ($_serverIp) - Long press to set IP'),
                          textAlign: TextAlign.center,
                          style: TextStyle(
                            color: _robotBusy 
                                ? Colors.orangeAccent 
                                : (_rosConnected ? Colors.greenAccent : Colors.redAccent),
                            fontSize: 10, fontWeight: FontWeight.bold, letterSpacing: 0.5,
                          ),
                        ),
                      ),
                    ),
                    SizedBox(height: pad * 0.4),
                    
                    _inputModeRow(fs),
                    SizedBox(height: pad * 0.5),
                    
                    Center(
                      child: Container(
                        width: gridW + pad * 2.2,
                        padding: EdgeInsets.fromLTRB(pad, pad * 0.4, pad, pad * 0.6),
                        decoration: BoxDecoration(
                          color: teamTheme == 'red' ? const Color(0xFFf2aeb0) : const Color(0xFF8ecae6),
                          borderRadius: BorderRadius.circular(12),
                          border: Border.all(
                            color: teamTheme == 'red' ? const Color(0xFFc0392b) : const Color(0xFF2980b9),
                            width: 4,
                          ),
                        ),
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Text(
                              'RACK',
                              style: TextStyle(
                                fontSize: fs * 0.85, fontWeight: FontWeight.w900, color: Colors.white,
                                shadows: const [Shadow(color: Colors.black, offset: Offset(1, 1))],
                              ),
                            ),
                            SizedBox(height: pad * 0.2),
                            SizedBox(width: gridW, height: gridH, child: _buildGrid(gridW, gridH, fs)),
                          ],
                        ),
                      ),
                    ),
                    SizedBox(height: pad * 0.4),
                    
                    ElevatedButton(
                      onPressed: _robotBusy ? null : _confirmR2,
                      style: ElevatedButton.styleFrom(
                        backgroundColor: _robotBusy ? Colors.grey[800] : const Color(0xFF27ae60),
                        padding: EdgeInsets.symmetric(vertical: pad * 0.7),
                        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                      ),
                      child: Text('EXECUTE R2 TARGET', style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold, fontSize: fs * 1.1)),
                    ),
                    
                    if (hasPush) ...[
                      SizedBox(height: pad * 0.3),
                      ElevatedButton(
                        onPressed: (_canPush && !_robotBusy) ? _confirmPush : null,
                        style: ElevatedButton.styleFrom(
                          backgroundColor: (_canPush && !_robotBusy) ? const Color(0xFFe67e22) : Colors.grey[700],
                          padding: EdgeInsets.symmetric(vertical: pad * 0.7),
                          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(8)),
                        ),
                        child: Text(
                          _canPush ? 'PUSH KFS  (${staffCount - pushesUsed} remaining)' : 'PUSH FINISH',
                          style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold, fontSize: fs * 1.1),
                        ),
                      ),
                    ],
                    SizedBox(height: pad * 0.4),
                    _hud(fs),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );
  }

  Widget _inputModeRow(double fs) 
  {
    const modes = [('opp', 'OPP (R2)', Icons.adjust), ('remove', 'REMOVE', Icons.delete_outline)];
    return Row(
      children: modes.map((m) {
        final active = inputMode == m.$1;
        return Expanded(
          child: GestureDetector(
            onTap: () => setState(() => inputMode = m.$1),
            child: Container(
              margin: const EdgeInsets.symmetric(horizontal: 3),
              padding: const EdgeInsets.symmetric(vertical: 7),
              decoration: BoxDecoration(
                color: active ? teamColor : const Color(0xFF2c3e50),
                borderRadius: BorderRadius.circular(6),
                border: Border.all(color: active ? teamColor : Colors.white24, width: 1.5),
              ),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  Icon(m.$3, color: Colors.white, size: fs * 1.3),
                  const SizedBox(height: 2),
                  Text(m.$2, style: TextStyle(color: Colors.white, fontSize: fs * 0.75, fontWeight: FontWeight.bold)),
                ],
              ),
            ),
          ),
        );
      }).toList(),
    );
  }

  Widget _statPanel(double fs) => Container(
    padding: const EdgeInsets.symmetric(vertical: 10, horizontal: 8),
    decoration: BoxDecoration(color: const Color(0xFF2c3e50), borderRadius: BorderRadius.circular(8)),
    child: Row(
      mainAxisAlignment: MainAxisAlignment.spaceAround,
      children: [
        _statBox('Staff', _staffCtrl, fs, (v) => setState(() { staffCount = v; if (pushesUsed > staffCount) pushesUsed = staffCount; _calculateDecision(); })),
        _statBox('R2 KFS', _r2Ctrl, fs, (v) => setState(() { r2Max = v; _calculateDecision(); })),
        _statBox('R1 KFS', _r1Ctrl, fs, (v) => setState(() { r1Max = v; _calculateDecision(); })),
        Column(
          children: [
            Text('Pushes', style: TextStyle(color: Colors.white, fontSize: fs * 0.9)),
            const SizedBox(height: 4),
            Container(
              width: 52, height: 34,
              decoration: BoxDecoration(color: pushesUsed >= staffCount ? Colors.red[900] : Colors.grey[800], borderRadius: BorderRadius.circular(4)),
              alignment: Alignment.center,
              child: Text('$pushesUsed/$staffCount', style: TextStyle(color: Colors.white, fontWeight: FontWeight.bold, fontSize: fs * 0.95)),
            ),
          ],
        ),
        Column(
          children: [
            Text('Team', style: TextStyle(color: Colors.white, fontSize: fs * 0.9)),
            const SizedBox(height: 6),
            Row(
              children: [
                _colorDot('red', Colors.red),
                const SizedBox(width: 6),
                _colorDot('blue', Colors.blue),
              ],
            ),
          ],
        ),
        Column(
          children: [
            Text('Reset', style: TextStyle(color: Colors.white, fontSize: fs * 0.9)),
            const SizedBox(height: 4),
            GestureDetector(
              onTap: _resetBoard,
              child: Container(
                width: 34, height: 34,
                decoration: BoxDecoration(color: Colors.red[900], borderRadius: BorderRadius.circular(4)),
                alignment: Alignment.center,
                child: const Text('↺', style: TextStyle(color: Colors.white, fontSize: 18, fontWeight: FontWeight.bold)),
              ),
            ),
          ],
        ),
      ],
    ),
  );

  Widget _colorDot(String theme, Color color) => GestureDetector(
    onTap: () => setState(() {
      teamTheme = theme;
      _calculateDecision();
      _syncTeamColor(); 
    }),
    child: Container(
      width: 26, height: 26,
      decoration: BoxDecoration(
        color: color, shape: BoxShape.circle,
        border: teamTheme == theme ? Border.all(color: Colors.white, width: 3) : null,
      ),
    ),
  );

  Widget _statBox(String label, TextEditingController ctrl, double fs, Function(int) onChange) => Column(
    children: [
      Text(label, style: TextStyle(color: Colors.white, fontSize: fs * 0.9)),
      const SizedBox(height: 4),
      Container(
        width: 52, height: 34,
        decoration: BoxDecoration(color: Colors.white, borderRadius: BorderRadius.circular(4)),
        child: TextField(
          controller: ctrl, textAlign: TextAlign.center, keyboardType: TextInputType.number,
          style: TextStyle(color: Colors.black, fontWeight: FontWeight.bold, fontSize: fs * 1.1),
          onSubmitted: (s) => onChange(int.tryParse(s) ?? 0),
          onEditingComplete: () { onChange(int.tryParse(ctrl.text) ?? 0); FocusScope.of(context).unfocus(); },
          decoration: const InputDecoration(border: InputBorder.none, contentPadding: EdgeInsets.symmetric(vertical: 7)),
        ),
      ),
    ],
  );

  Widget _buildGrid(double w, double h, double fs) => GridView.builder(
    physics: const NeverScrollableScrollPhysics(),
    gridDelegate: const SliverGridDelegateWithFixedCrossAxisCount(crossAxisCount: 3, mainAxisSpacing: 4, crossAxisSpacing: 4, childAspectRatio: 1),
    itemCount: 9,
    itemBuilder: (_, i) => _cell(i, w, fs),
  );

  Widget _cell(int i, double gridW, double fs) {
    final val = boardState[i];
    final isTarget = currentDecision?['slot'] == i;
    final isPush = currentDecision?['push_idx'] == i && val == 2;
    final isBottom = i >= 6;
    final col = i % 3;
    final r1InCol = boardState[6 + col] == 1;
    final isRemove = inputMode == 'remove';

    Color bgColor;
    if (val == 1) 
    {
      bgColor = teamColor;
    } 
    else if (val == 2)
    {
      bgColor = oppColor;
    } 
    else 
    {
      bgColor = isBottom ? const Color(0xFF1a2a1a) : const Color(0xFF34495e);
    }

    String cellLabel = '';
    if (val == 1)
    {
      cellLabel = isBottom ? 'R1\nOWN' : 'R2\nOWN';
    } 
    else if (val == 2)
    {
      cellLabel = isBottom ? 'R1\nOPP' : 'OPP';
    }

    final rowBadge = i < 3 ? 'TOP' : i < 6 ? 'MID' : 'R1';

    Widget content = Stack(
      alignment: Alignment.center,
      children: [
        if (val == 0) Text(rowBadge, style: TextStyle(color: Colors.white24, fontSize: fs * 0.7, fontWeight: FontWeight.w600)),
        if (val != 0) Text(cellLabel, textAlign: TextAlign.center, style: TextStyle(color: Colors.white, fontSize: fs, fontWeight: FontWeight.bold, shadows: const [Shadow(color: Colors.black, blurRadius: 4)])),
        if (isTarget) Positioned(bottom: 3, child: Text('TARGET', style: TextStyle(color: Colors.greenAccent, fontSize: fs * 0.65, fontWeight: FontWeight.bold))),
        if (isPush) AnimatedBuilder(
          animation: _flashAnim,
          builder: (_, __) => Opacity(opacity: _flashAnim.value, child: Text('✕', style: TextStyle(color: Colors.yellow, fontSize: gridW * 0.12, fontWeight: FontWeight.w900, shadows: const [Shadow(color: Colors.black, blurRadius: 8)]))),
        ),
        if (!isBottom && val == 0 && r1InCol) Positioned(top: 3, child: Text('▲ R1', style: TextStyle(color: Colors.amberAccent, fontSize: fs * 0.6, fontWeight: FontWeight.bold))),
      ],
    );

    if (isTarget) 
    {
      content = AnimatedBuilder(animation: _pulseAnim, builder: (_, child) => Transform.scale(scale: _pulseAnim.value, child: child), child: content);
    }

    return GestureDetector(
      onTap: () => _cellClicked(i),
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 150),
        decoration: BoxDecoration(
          color: bgColor,
          borderRadius: BorderRadius.circular(6),
          border: isRemove ? Border.all(color: Colors.red, width: 2) : Border.all(
            color: isTarget ? Colors.greenAccent : isPush ? Colors.orange : isBottom ? Colors.white24 : const Color(0xFF7f8c8d),
            width: isTarget || isPush ? 3 : 1.5,
          ),
          boxShadow: isTarget ? [BoxShadow(color: Colors.greenAccent.withOpacity(0.5), blurRadius: 10, spreadRadius: 2)] : null,
        ),
        child: content,
      ),
    );
  }

  Widget _hud(double fs) {
    final r2Action = currentDecision?['r2_action'] ?? 'NULL';
    final r1Action = currentDecision?['r1_action'] ?? 'NULL';
    final msg = currentDecision?['message'] ?? 'WAITING';

    return Container(
      width: double.infinity, padding: const EdgeInsets.all(10),
      decoration: BoxDecoration(color: const Color(0xFF0D0D0D), border: Border.all(color: const Color(0xFF33FF33).withOpacity(0.3)), borderRadius: BorderRadius.circular(4)),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text('mode: ${inputMode.toUpperCase()}', style: TextStyle(color: Colors.grey[600], fontFamily: 'monospace', fontSize: fs * 0.7, fontWeight: FontWeight.bold)),
          const SizedBox(height: 8),
          _serialLine('R2', r2Action, const Color(0xFF33FF33), fs),
          _serialLine('R1', r1Action, const Color(0xFF33FF33), fs),
          _serialLine('Game state', _gameStatus, Colors.orangeAccent, fs),
          const Divider(color: Color(0xFF222222), height: 12),
          Text('msg: $msg', style: TextStyle(color: Colors.white70, fontFamily: 'monospace', fontSize: fs * 0.85)),
        ],
      ),
    );
  }

  Widget _serialLine(String key, String val, Color valColor, double fs) => Padding(
    padding: const EdgeInsets.symmetric(vertical: 2),
    child: RichText(text: TextSpan(style: TextStyle(fontFamily: 'monospace', fontSize: fs * 0.9), children: [TextSpan(text: '$key: ', style: const TextStyle(color: Colors.white54)), TextSpan(text: val, style: TextStyle(color: valColor, fontWeight: FontWeight.bold))])),
  );
}