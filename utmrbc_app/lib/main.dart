import 'package:flutter/material.dart';
import 'pages/home.dart';
import 'pages/cam_kfs.dart';
import 'pages/swipe_page.dart';  

void main() {
  WidgetsFlutterBinding.ensureInitialized();
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'UTMRBC MF',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(
          seedColor: const Color.fromARGB(255, 229, 230, 235),
        ),
        useMaterial3: true,
      ),
      debugShowCheckedModeBanner: false,
      initialRoute: '/',
      routes: {
        '/': (context) => const HomePage(),
        '/mc': (context) => const SwipePage(initialPage: 0),
        '/mf-blocks': (context) => const SwipePage(initialPage: 1),
        '/arena': (context) => const SwipePage(initialPage: 2),
        '/mf-detection': (context) => const MFDetectionPage(),
      },
    );
  }
}