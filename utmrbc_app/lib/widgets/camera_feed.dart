import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';
import 'dart:typed_data';

class CameraFeedWidget extends StatefulWidget {
  final String serverUrl;
  final double width;
  final double height;
  final bool showControls;
  
  const CameraFeedWidget({
    super.key,
    this.serverUrl = 'http://10.161.8.61:5000',
    this.width = 320,
    this.height = 240,
    this.showControls = true,
  });

  @override
  State<CameraFeedWidget> createState() => _CameraFeedWidgetState();
}

class _CameraFeedWidgetState extends State<CameraFeedWidget> {
  bool _isConnected = false;
  String _status = 'Disconnected';
  int _fps = 0;
  Timer? _statusTimer;
  String? _errorMessage;
  String _streamUrl = '';
  
  //vid streaming
  StreamSubscription<List<int>>? _streamSubscription;
  Uint8List? _currentFrame;
  bool _isStreaming = false;

  @override
  void initState() {
    super.initState();
    _updateStreamUrl();
    _checkServerStatus();
    
    // status
    _statusTimer = Timer.periodic(
      const Duration(seconds: 2),
      (_) => _checkServerStatus(),
    );
  }

  void _updateStreamUrl() {
    _streamUrl = '${widget.serverUrl}/video';
    print('YOLO Stream URL: $_streamUrl');
  }

  @override
  void didUpdateWidget(CameraFeedWidget oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (oldWidget.serverUrl != widget.serverUrl) {
      _stopStream();
      _updateStreamUrl();
      if (_isConnected) {
        _startStream();
      }
    }
  }

  Future<void> _checkServerStatus() async {
    try {
      final response = await http
          .get(Uri.parse('${widget.serverUrl}/status'))
          .timeout(const Duration(seconds: 2));
      
      if (response.statusCode == 200) {
        final data = json.decode(response.body);
        final wasConnected = _isConnected;
        
        if (mounted) {
          setState(() {
            _isConnected = data['receiving_frames'] ?? false;
            _fps = data['fps'] ?? 0;
            _status = _isConnected ? 'Connected' : 'No YOLO Data';
            _errorMessage = null;
          });
          
          if (_isConnected && !wasConnected && !_isStreaming) {
            _startStream();
          }
        }
      } else {
        _setDisconnected();
      }
    } catch (e) {
      _setDisconnected(e.toString());
    }
  }

  void _setDisconnected([String? error]) {
    if (mounted) {
      setState(() {
        _isConnected = false;
        _status = 'Disconnected';
        _errorMessage = error;
      });
      _stopStream();
    }
  }

  void _startStream() async {
    if (_isStreaming) return;
    
    print('🎬 Starting MJPEG stream...');
    _isStreaming = true;
    
    try {
      final request = http.Request('GET', Uri.parse(_streamUrl));
      final response = await request.send();
      
      if (response.statusCode == 200) {
        List<int> buffer = [];
        const jpegStart = [0xFF, 0xD8]; // JPEG start marker
        const jpegEnd = [0xFF, 0xD9];   // JPEG end marker
        
        _streamSubscription = response.stream.listen(
          (chunk) {
            buffer.addAll(chunk);
            
            while (true) {

              int startIdx = -1;
              for (int i = 0; i < buffer.length - 1; i++) {
                if (buffer[i] == jpegStart[0] && buffer[i + 1] == jpegStart[1]) {
                  startIdx = i;
                  break;
                }
              }
              
              if (startIdx == -1) {
                buffer.clear();
                break;
              }
              
              int endIdx = -1;
              for (int i = startIdx + 2; i < buffer.length - 1; i++) {
                if (buffer[i] == jpegEnd[0] && buffer[i + 1] == jpegEnd[1]) {
                  endIdx = i + 2;
                  break;
                }
              }
              
              if (endIdx == -1) {
                buffer = buffer.sublist(startIdx);
                break;
              }
              
              final frameData = Uint8List.fromList(buffer.sublist(startIdx, endIdx));
              
              if (mounted) {
                setState(() {
                  _currentFrame = frameData;
                });
              }
              
              buffer = buffer.sublist(endIdx);
            }
          },
          onError: (error) {
            print('Stream error: $error');
            _stopStream();
            _setDisconnected(error.toString());
          },
          onDone: () {
            print('Stream ended');
            _stopStream();
          },
          cancelOnError: true,
        );
      } else {
        print('Stream failed with status: ${response.statusCode}');
        _stopStream();
      }
    } catch (e) {
      print('Stream exception: $e');
      _stopStream();
      _setDisconnected(e.toString());
    }
  }
  void _stopStream() {
    _streamSubscription?.cancel();
    _streamSubscription = null;
    _isStreaming = false;
  }

  void _refresh() {
    _stopStream();
    setState(() {
      _currentFrame = null;
    });
    _checkServerStatus();
  }

  @override
  void dispose() {
    _statusTimer?.cancel();
    _stopStream();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Container(
      width: widget.width,
      height: widget.height + (widget.showControls ? 40 : 0),
      decoration: BoxDecoration(
        color: Colors.black,
        borderRadius: BorderRadius.circular(8),
        border: Border.all(
          color: _isConnected ? Colors.green : Colors.red,
          width: 2,
        ),
      ),
      child: Column(
        children: [
          Expanded(
            child: ClipRRect(
              borderRadius: const BorderRadius.vertical(top: Radius.circular(6)),
              child: buildVideoDisplay(),
            ),
          ),
          if (widget.showControls) buildControls(),
        ],
      ),
    );
  }

  Widget buildVideoDisplay() {
    if (!_isConnected) {
      return Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(
              Icons.videocam_off,
              color: Colors.red.withOpacity(0.6),
              size: 48,
            ),
            const SizedBox(height: 12),
            Text(
              _status,
              style: const TextStyle(
                color: Colors.white70,
                fontWeight: FontWeight.bold,
                fontSize: 14,
              ),
            ),
            if (_errorMessage != null) ...[
              const SizedBox(height: 8),
              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 16),
                child: Text(
                  _errorMessage!,
                  style: const TextStyle(
                    color: Colors.white54,
                    fontSize: 10,
                  ),
                  textAlign: TextAlign.center,
                  maxLines: 3,
                  overflow: TextOverflow.ellipsis,
                ),
              ),
            ],
            const SizedBox(height: 16),
            ElevatedButton.icon(
              onPressed: _refresh,
              icon: const Icon(Icons.refresh, size: 16),
              label: const Text('Reconnect'),
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.green,
                foregroundColor: Colors.white,
                padding: const EdgeInsets.symmetric(
                  horizontal: 16,
                  vertical: 8,
                ),
              ),
            ),
          ],
        ),
      );
    }

    //display current MJPEG frame
    if (_currentFrame == null) {
      return const Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            CircularProgressIndicator(color: Colors.green),
            SizedBox(height: 12),
            Text(
              'Loading YOLO stream...',
              style: TextStyle(color: Colors.white70, fontSize: 12),
            ),
          ],
        ),
      );
    }

    return Image.memory(
      _currentFrame!,
      fit: BoxFit.contain,
      gaplessPlayback: true, 
      errorBuilder: (context, error, stackTrace) {
        return Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(
                Icons.error_outline,
                color: Colors.red.withOpacity(0.6),
                size: 48,
              ),
              const SizedBox(height: 12),
              const Text(
                'Frame Decode Error',
                style: TextStyle(
                  color: Colors.white70,
                  fontWeight: FontWeight.bold,
                  fontSize: 14,
                ),
              ),
            ],
          ),
        );
      },
    );
  }

  Widget buildControls() {
    return Container(
      height: 40,
      padding: const EdgeInsets.symmetric(horizontal: 8),
      decoration: const BoxDecoration(
        color: Colors.black87,
        borderRadius: BorderRadius.vertical(bottom: Radius.circular(6)),
      ),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceBetween,
        children: [
          Row(
            children: [
              Container(
                width: 8,
                height: 8,
                decoration: BoxDecoration(
                  color: _isConnected ? Colors.green : Colors.red,
                  shape: BoxShape.circle,
                ),
              ),
              const SizedBox(width: 8),
              Text(
                '$_status${_isConnected && _fps > 0 ? " • ${_fps}fps" : ""}',
                style: const TextStyle(
                  color: Colors.white70,
                  fontSize: 11,
                ),
              ),
              const SizedBox(width: 8),
              Container(
                padding: const EdgeInsets.symmetric(
                  horizontal: 6,
                  vertical: 2,
                ),
                decoration: BoxDecoration(
                  color: Colors.orange.withOpacity(0.3),
                  borderRadius: BorderRadius.circular(4),
                ),
                child: const Text(
                  'YOLO',
                  style: TextStyle(
                    color: Colors.orange,
                    fontSize: 9,
                    fontWeight: FontWeight.bold,
                  ),
                ),
              ),
            ],
          ),
          IconButton(
            icon: const Icon(Icons.refresh),
            color: Colors.white70,
            iconSize: 18,
            padding: EdgeInsets.zero,
            constraints: const BoxConstraints(),
            onPressed: _refresh,
            tooltip: 'Refresh stream',
          ),
        ],
      ),
    );
  }
}