//use hotspot to connect better
//
// OPERATOR-CONSOLE rosbridge client. The app no longer computes or sends a
// path: r2_brain is the single planner/controller. This client only publishes
// OPERATOR INTENT (grid, mode, e-stop, mission command) and relays incoming
// telemetry. The retired /robot_paths binary-CAN publisher (publishPaths +
// _packToBinary + CRC) used to live here — it is gone. See cal_path.dart
// (RETIRED, offline/preview only).
import 'dart:async';
import 'dart:convert';
import 'package:flutter/foundation.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as ws_status;

class ROS2Client {
  // ── Intent topic contract with r2_brain (single source of truth) ──────────
  static const String topicManualGrid = '/r2/forest/manual_grid'; // String (JSON)
  static const String topicArenaMode  = '/r2/arena/mode';         // String "attack"|"defense"
  static const String topicEstop      = '/r2/estop';              // Bool
  static const String topicMissionCmd = '/r2/mission/cmd';        // String "start"|"stop"|"retry"

  static const String _typeString = 'std_msgs/String';
  static const String _typeBool   = 'std_msgs/Bool';

  // Topics advertised on connect — topic -> rosbridge type.
  static const Map<String, String> _advertisedTopics = {
    topicManualGrid: _typeString,
    topicArenaMode:  _typeString,
    topicEstop:      _typeBool,
    topicMissionCmd: _typeString,
  };

  // Allowed enum values (guard against publishing garbage to the brain).
  static const Set<String> _validModes       = {'attack', 'defense'};
  static const Set<String> _validMissionCmds = {'start', 'stop', 'retry'};

  static WebSocketChannel?   _channel;
  static StreamSubscription? _sub;
  static bool _connected  = false;
  static bool _advertised = false;
  static bool get isConnected => _connected;

  static void Function(bool connected)? onConnectionChanged;
  static void Function(String topic, dynamic msg)? onMessageReceived;

  static void _setConnected(bool value)
  {
    if (_connected == value) return;
    _connected = value;
    if (!value) _advertised = false;
    onConnectionChanged?.call(value);
  }

  static Future<bool> connect({
    String host = 'localhost',
    int    port = 9090,
  }) async {
    await _safeClose();

    try {
      debugPrint('[ROS2] Connecting to ws://$host:$port ...');

      final uri = Uri.parse('ws://$host:$port');
      _channel = WebSocketChannel.connect(uri);

      await _channel!.ready.timeout(const Duration(seconds: 5));

      _sub = _channel!.stream.listen(
        (data)
        {
          debugPrint('[ROS2] ← $data');
          _onMessage(data);
        },
        onError: (e)
        {
          debugPrint('[ROS2] Error: $e');
          _setConnected(false);
        },
        onDone: ()
        {
          debugPrint('[ROS2] Connection closed by server');
          _setConnected(false);
        },
        cancelOnError: false,
      );

      _setConnected(true);

      // Advertise every intent topic the operator console publishes.
      for (final entry in _advertisedTopics.entries) {
        _rawSend({'op': 'advertise', 'topic': entry.key, 'type': entry.value});
      }

      await Future.delayed(const Duration(milliseconds: 500));
      _advertised = true;

      debugPrint('[ROS2] Connected & advertised ${_advertisedTopics.keys.join(", ")}');
      return true;
    } catch (e) {
      debugPrint('[ROS2] Connect failed: $e');
      await _safeClose(); //reset
      _setConnected(false);
      return false;
    }
  }

  static Future<void> disconnect() async {
    if (_connected)
    {
      for (final topic in _advertisedTopics.keys) {
        _rawSend({'op': 'unadvertise', 'topic': topic});
      }
    }
    await _safeClose();
    debugPrint('[ROS2] Disconnected');
  }

  static void subscribe(String topic, String msgType) {
    if (!_connected) return;
    _rawSend({'op': 'subscribe', 'topic': topic, 'type': msgType});
  }

  static void unsubscribe(String topic) {
    if (!_connected) return;
    _rawSend({'op': 'unsubscribe', 'topic': topic});
  }

  // ── Intent publishers ─────────────────────────────────────────────────────

  /// Raw grid intent → /r2/forest/manual_grid. r2_brain runs the real planner.
  static bool publishManualGrid(Map<String, dynamic> grid) =>
      _publishJsonString(topicManualGrid, grid);

  /// "attack" | "defense" → /r2/arena/mode.
  static bool publishArenaMode(String mode) {
    final m = mode.toLowerCase().trim();
    if (!_validModes.contains(m)) {
      debugPrint('[ROS2] ignoring invalid arena mode: "$mode"');
      return false;
    }
    return _publishString(topicArenaMode, m);
  }

  /// e-stop engaged/cleared → /r2/estop.
  static bool publishEstop(bool engaged) => _publishBool(topicEstop, engaged);

  /// "start" | "stop" | "retry" → /r2/mission/cmd.
  static bool publishMissionCmd(String cmd) {
    final c = cmd.toLowerCase().trim();
    if (!_validMissionCmds.contains(c)) {
      debugPrint('[ROS2] ignoring invalid mission cmd: "$cmd"');
      return false;
    }
    return _publishString(topicMissionCmd, c);
  }

  // ── Publish helpers ─────────────────────────────────────────────────────

  /// std_msgs/String whose `data` is a JSON-encoded object.
  static bool _publishJsonString(String topic, Map<String, dynamic> obj) {
    String encoded;
    try {
      encoded = json.encode(obj);
    } catch (e) {
      debugPrint('[ROS2] JSON encode failed for $topic: $e');
      return false;
    }
    return _publishString(topic, encoded);
  }

  static bool _publishString(String topic, String data) =>
      _publishMsg(topic, {'data': data});

  static bool _publishBool(String topic, bool data) =>
      _publishMsg(topic, {'data': data});

  static bool _publishMsg(String topic, Map<String, dynamic> msg) {
    if (!_connected || _channel == null) return false;

    if (!_advertised)
    {
      // advertise() hasn't settled yet — retry shortly.
      Future.delayed(const Duration(milliseconds: 500),
          () => _publishMsg(topic, msg));
      return true;
    }

    try {
      _channel!.sink.add(json.encode({
        'op':    'publish',
        'topic': topic,
        'msg':   msg,
      }));
      debugPrint('[ROS2] → $topic $msg');
      return true;
    } catch (e) {
      debugPrint('[ROS2] publish $topic failed: $e');
      _setConnected(false);
      return false;
    }
  }

  static void _rawSend(Map<String, dynamic> payload) {
    if (_channel == null) return;
    try
    {
      _channel!.sink.add(json.encode(payload));
    } catch (e) {
      debugPrint('[ROS2] _rawSend error: $e');
    }
  }

  static Future<void> _safeClose() async {
    _advertised = false;
    _connected = false;

    if (_sub != null)
    {
      await _sub!.cancel();
      _sub = null;
    }

    if (_channel != null)
    {
      try
      {
        _channel!.sink.close(ws_status.goingAway);
      }
      catch (_) {}
      _channel = null;
    }
  }

  static void _onMessage(dynamic raw) {
    try
    {
      final msg = json.decode(raw as String) as Map<String, dynamic>;
      if (msg['op'] == 'publish' && msg['topic'] != null) {
        onMessageReceived?.call(msg['topic'] as String, msg['msg']);
      }
    } catch (_) {}
  }
}
