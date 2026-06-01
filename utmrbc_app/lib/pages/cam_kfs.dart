import 'package:flutter/material.dart';
import '../widgets/camera_feed.dart';
import 'package:http/http.dart' as http;
import 'dart:convert';
import 'dart:async';

class MFDetectionPage extends StatefulWidget {
  const MFDetectionPage({super.key});

  @override
  State<MFDetectionPage> createState() => _MFDetectionPageState();
}

class _MFDetectionPageState extends State<MFDetectionPage> {
  String _detectionStatus = 'Stopped';
  bool _isProcessing = false;
  bool _isConnected = false;
  int _fps = 0;
  Timer? _statusTimer;

  static const String serverUrl = 'http://10.161.8.61:5000'; 
  
  @override
  void initState() 
  {
    super.initState();
    _checkServerStatus();
    //check status every 2 seconds
    _statusTimer = Timer.periodic(
      const Duration(seconds: 2),
      (_) => _checkServerStatus(),
    );
  }

  @override
  void dispose() 
  {
    _statusTimer?.cancel();
    super.dispose();
  }

  Future<void> _checkServerStatus() async {
    try {
      final response = await http
          .get(Uri.parse('$serverUrl/status'))
          .timeout(const Duration(seconds: 2));
      
      if (response.statusCode == 200) 
      {
        final data = json.decode(response.body);
        if (mounted) {
          setState(() 
          {
            _isConnected = data['receiving_frames'] ?? false;
            _fps = data['fps'] ?? 0;
            
            if (_isConnected) 
            {
              _detectionStatus = 'Running';
              _isProcessing = true;
            } 
            else 
            {
              _detectionStatus = 'No Data';
              _isProcessing = false;
            }
          });
        }
      } 
      else 
      {
        _setDisconnected();
      }
    } 
    catch (e) 
    {
      _setDisconnected();
    }
  }

  void _setDisconnected() 
  {
    if (mounted) 
    {
      setState(() 
      {
        _isConnected = false;
        _detectionStatus = 'Disconnected';
        _isProcessing = false;
      });
    }
  }

  @override
  Widget build(BuildContext context) 
  {
    return Scaffold(
      appBar: AppBar(
        title: const Text('MF - Cam Detection'),
        backgroundColor: Colors.green,
        actions: [
          IconButton(
            icon: const Icon(Icons.grid_view),
            onPressed: () {
              Navigator.pushNamed(context, '/mf-blocks');
            },
            tooltip: 'Insert KFS',
          ),
        ],
      ),
      body: Container(
        color: const Color.fromARGB(255, 230, 245, 233),
        child: Row(
          children: [
            Expanded(
              flex: 3,
              child: _buildCameraSection(),
            ),
            Expanded(
              flex: 2,
              child: _buildDetectionInfo(),
            ),
          ],
        ),
      ),
    );
  }

  Widget _buildCameraSection() 
  {
    return Padding(
      padding: const EdgeInsets.all(16),
      child: Column(
        children: [
          Expanded(
            child: Container(
              decoration: BoxDecoration(
                borderRadius: BorderRadius.circular(8),
                border: Border.all(
                  color: _isConnected ? Colors.green : Colors.red,
                  width: 3,
                ),
                boxShadow: [
                  BoxShadow(
                    color: Colors.black.withValues(alpha: 0.2),
                    blurRadius: 8,
                    offset: const Offset(0, 4),
                  ),
                ],
              ),
              child: ClipRRect(
                borderRadius: BorderRadius.circular(6),
                child: Stack(
                  children: [
                    const CameraFeedWidget(
                      serverUrl: serverUrl,
                      width: double.infinity,
                      height: double.infinity,
                      showControls: true,
                    ),
                    // Status badge
                    Positioned(
                      top: 8,
                      right: 8,
                      child: Container(
                        padding: const EdgeInsets.symmetric(
                          horizontal: 12,
                          vertical: 6,
                        ),
                        decoration: BoxDecoration(
                          color: _isConnected 
                              ? Colors.green.withValues(alpha: 0.9)
                              : Colors.red.withValues(alpha: 0.9),
                          borderRadius: BorderRadius.circular(20),
                        ),
                        child: Row(
                          mainAxisSize: MainAxisSize.min,
                          children: [
                            Icon(
                              _isConnected ? Icons.visibility : Icons.visibility_off,
                              size: 16,
                              color: Colors.white,
                            ),
                            const SizedBox(width: 4),
                            Text(
                              _isConnected ? 'YOLO Active' : 'Waiting...',
                              style: const TextStyle(
                                color: Colors.white,
                                fontSize: 12,
                                fontWeight: FontWeight.bold,
                              ),
                            ),
                          ],
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
          ),
          const SizedBox(height: 16),
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.white,
              borderRadius: BorderRadius.circular(8),
              border: Border.all(color: Colors.green.shade300),
            ),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceAround,
              children: [
                _buildInfoChip(
                  Icons.videocam,
                  'YOLO Stream',
                  _isConnected,
                ),
                _buildInfoChip(
                  Icons.analytics,
                  'Detection ${_isProcessing ? "ON" : "OFF"}',
                  _isProcessing,
                ),
                _buildInfoChip(
                  Icons.speed,
                  _fps > 0 ? '${_fps}fps' : _detectionStatus,
                  _isConnected,
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }

  Widget _buildInfoChip(IconData icon, String label, bool isActive) 
  {
    return Row(
      children: [
        Icon(
          icon,
          size: 16,
          color: isActive ? Colors.green : Colors.grey,
        ),
        const SizedBox(width: 6),
        Text(
          label,
          style: TextStyle(
            fontSize: 12,
            fontWeight: isActive ? FontWeight.bold : FontWeight.normal,
            color: isActive ? Colors.green[700] : Colors.grey[600],
          ),
        ),
      ],
    );
  }

  Widget _buildDetectionInfo() 
  {
    return Container(
      margin: const EdgeInsets.all(16),
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(12),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withValues(alpha: 0.1),
            blurRadius: 8,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            children: [
              Icon(
                Icons.info_outline,
                color: Colors.green[700],
              ),
              const SizedBox(width: 8),
              const Text(
                'YOLO Detection Info',
                style: TextStyle(
                  fontSize: 20,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ],
          ),
          const Divider(height: 24),
          
          //connection status
          _buildStatusRow(
            'Stream',
            _isConnected ? 'Connected' : 'Disconnected',
            _isConnected ? Colors.green : Colors.red,
          ),
          const SizedBox(height: 12),
          
          //detection Status
          _buildStatusRow(
            'YOLO Status',
            _detectionStatus,
            _isProcessing ? Colors.green : Colors.grey,
          ),
          const SizedBox(height: 12),
          
          //fps
          _buildStatusRow(
            'Frame Rate',
            _fps > 0 ? '$_fps fps' : 'N/A',
            _fps > 0 ? Colors.green : Colors.grey,
          ),
          
          const SizedBox(height: 24),
          const Text(
            'System Info',
            style: TextStyle(
              fontSize: 16,
              fontWeight: FontWeight.bold,
            ),
          ),
          const SizedBox(height: 12),
          
          Container(
            padding: const EdgeInsets.all(12),
            decoration: BoxDecoration(
              color: Colors.blue.shade50,
              borderRadius: BorderRadius.circular(8),
              border: Border.all(color: Colors.blue.shade200),
            ),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Row(
                  children: [
                    Icon(Icons.info, size: 16, color: Colors.blue),
                    SizedBox(width: 8),
                    Text(
                      'How it works:',
                      style: TextStyle(
                        fontWeight: FontWeight.bold,
                        fontSize: 14,
                      ),
                    ),
                  ],
                ),
                const SizedBox(height: 8),
                Text(
                  '1. Camera_MF.launch.py captures camera\n'
                  '2. YOLO detects kfs_r1, kfs_r2, kfs_fake\n'
                  '3. Stream displays live detections\n'
                  '4. Bounding boxes show detected blocks',
                  style: TextStyle(
                    fontSize: 12,
                    color: Colors.grey[700],
                    height: 1.5,
                  ),
                ),
              ],
            ),
          ),
          
          const Spacer(),
          
          // Connection Instructions
          if (!_isConnected) ...[
            const Divider(),
            const SizedBox(height: 8),
            Container(
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.orange.shade50,
                borderRadius: BorderRadius.circular(8),
                border: Border.all(color: Colors.orange.shade200),
              ),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Row(
                    children: [
                      Icon(Icons.warning, size: 16, color: Colors.orange),
                      SizedBox(width: 8),
                      Text(
                        'Not Connected',
                        style: TextStyle(
                          fontWeight: FontWeight.bold,
                          fontSize: 14,
                        ),
                      ),
                    ],
                  ),
                  const SizedBox(height: 8),
                  Text(
                    'Make sure:\n'
                    '• Camera_MF.launch.py is running\n'
                    '• yolo_streamer.py is running\n'
                    '• Both devices on same network',
                    style: TextStyle(
                      fontSize: 11,
                      color: Colors.grey[700],
                      height: 1.5,
                    ),
                  ),
                ],
              ),
            ),
          ],
        ],
      ),
    );
  }

  Widget _buildStatusRow(String label, String value, [Color? valueColor]) 
  {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceBetween,
      children: [
        Text(
          label,
          style: TextStyle(
            color: Colors.grey[600],
            fontSize: 14,
          ),
        ),
        Text(
          value,
          style: TextStyle(
            fontWeight: FontWeight.bold,
            fontSize: 14,
            color: valueColor,
          ),
        ),
      ],
    );
  }
}