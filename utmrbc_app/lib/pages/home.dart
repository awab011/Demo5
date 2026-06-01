import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import '../widgets/camera_feed.dart';

class HomePage extends StatefulWidget {
  const HomePage({super.key});

  @override
  State<HomePage> createState() => _HomePageState();
}

class _HomePageState extends State<HomePage> {
  bool _showCamera = false;

  @override
  void initState() 
  {
    super.initState();
    SystemChrome.setPreferredOrientations([DeviceOrientation.portraitUp]);
  }

  @override
  void dispose() 
  {
    SystemChrome.setPreferredOrientations([
      DeviceOrientation.portraitUp,
      DeviceOrientation.portraitDown,
      DeviceOrientation.landscapeLeft,
      DeviceOrientation.landscapeRight,
    ]);
    super.dispose();
  }

  @override
  Widget build(BuildContext context) 
  {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Home Page'),
        backgroundColor: Colors.blue,
        actions: [
          IconButton(
            icon: Icon(_showCamera ? Icons.videocam : Icons.videocam_off),
            onPressed: () {
              setState(() {
                _showCamera = !_showCamera;
              });
            },
            tooltip: 'Toggle Camera',
          ),
        ],
      ),
      backgroundColor: const Color.fromARGB(255, 135, 204, 221),
      body: Stack(
        children: [
          _zoneSelection(),

          if (_showCamera)
            Positioned(
              top: 16,
              right: 16,
              child: const CameraFeedWidget(
                width: 320,
                height: 240,
                showControls: true,
              ),
            ),
        ],
      ),
    );
  }

  Widget _zoneSelection() 
  {
    return Center(
      child: Padding(
        padding: const EdgeInsets.all(20),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              'Zone Selection',
              style: TextStyle(
                color: Colors.black,
                fontSize: 18,
                fontWeight: FontWeight.w600,
              ),
            ),
            const SizedBox(height: 20),

            Row(
              mainAxisAlignment: MainAxisAlignment.spaceEvenly,
              children: [
                buildZoneCard(
                  'MC Zone',
                  Colors.blue,
                  '/mc',
                  Icons.precision_manufacturing,
                ),
                buildMFZoneCard(),
                buildZoneCard(
                  'Arena Zone',
                  Colors.orange,
                  '/arena',
                  Icons.grid_3x3,
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }

  //mf card
  Widget buildMFZoneCard() 
  {
    return Expanded(
      child: Container(
        margin: const EdgeInsets.symmetric(horizontal: 8),
        height: 150,
        decoration: BoxDecoration(
          color: Colors.green.withOpacity(0.3),
          borderRadius: BorderRadius.circular(16),
          border: Border.all(color: Colors.green, width: 2),
        ),
        child: InkWell(
          onTap: () => showMFOptions(context),
          borderRadius: BorderRadius.circular(16),
          child: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(Icons.forest, size: 50, color: Colors.green),
                const SizedBox(height: 10),
                const Text(
                  'MF Zone',
                  textAlign: TextAlign.center,
                  style: TextStyle(
                    color: Colors.black,
                    fontSize: 16,
                    fontWeight: FontWeight.w600,
                  ),
                ),
                const SizedBox(height: 4),
                Text(
                  'Tap for options',
                  style: TextStyle(
                    color: Colors.black.withOpacity(0.6),
                    fontSize: 10,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }

  void showMFOptions(BuildContext context) 
  {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Row(
            children: [
              Icon(Icons.forest, color: Colors.green),
              const SizedBox(width: 8),
              const Text('MF Zone'),
            ],
          ),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              ListTile(
                leading: const Icon(Icons.camera_alt, color: Colors.blue),
                title: const Text('Camera Detection'),
                subtitle: const Text('Live camera feed & detection'),
                onTap: () {
                  Navigator.pop(context);
                  Navigator.pushNamed(context, '/mf-detection');
                },
              ),

              const Divider(),

              ListTile(
                leading: const Icon(Icons.grid_on, color: Colors.purple),
                title: const Text('Block Mapping'),
                subtitle: const Text('Manual block input & positioning'),
                onTap: () {
                  Navigator.pop(context);
                  Navigator.pushNamed(context, '/mf-blocks');
                },
              ),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.pop(context),
              child: const Text('Cancel'),
            ),
          ],
        );
      },
    );
  }

  Widget buildZoneCard(
    String zoneName,
    Color color,
    String route,
    IconData icon,
  ) {
    return Expanded(
      child: Container(
        margin: const EdgeInsets.symmetric(horizontal: 8),
        height: 150,
        decoration: BoxDecoration(
          color: color.withOpacity(0.3),
          borderRadius: BorderRadius.circular(16),
          border: Border.all(color: color, width: 2),
        ),
        child: InkWell(
          onTap: () => Navigator.pushNamed(context, route),
          borderRadius: BorderRadius.circular(16),
          child: Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Icon(icon, size: 50, color: color),
                const SizedBox(height: 10),
                Text(
                  zoneName,
                  textAlign: TextAlign.center,
                  style: const TextStyle(
                    color: Colors.black,
                    fontSize: 16,
                    fontWeight: FontWeight.w600,
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
