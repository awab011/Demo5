import 'package:flutter/material.dart';
import 'mc.dart';
import 'mf.dart';
import 'arena.dart';

class SwipePage extends StatefulWidget {
  final int initialPage;
  const SwipePage({super.key, this.initialPage = 0});

  @override
  State<SwipePage> createState() => _SwipePageState();
}

class _SwipePageState extends State<SwipePage> {
  late PageController _pageController;
  int _currentPage = 0;

  final _pages = const [
    MCPage(),
    MFBlocksPage(),
    ArenaPage(),
  ];

  final _labels = ['MC', 'MF', 'Arena'];
  final _colors = [
    Color(0xFF4ae6e6),
    Colors.green,
    Colors.deepPurple,
  ];

  @override
  void initState() {
    super.initState();
    _currentPage = widget.initialPage;
    _pageController = PageController(initialPage: widget.initialPage);
  }

  @override
  void dispose() {
    _pageController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Stack(
        children: [
          PageView(
            controller: _pageController,
            onPageChanged: (i) => setState(() => _currentPage = i),
            children: _pages,
          ),
          Positioned(
            bottom: 12,
            left: 0, right: 0,
            child: Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: List.generate(_pages.length, (i) {
                final active = i == _currentPage;
                return AnimatedContainer(
                  duration: const Duration(milliseconds: 250),
                  margin: const EdgeInsets.symmetric(horizontal: 4),
                  width:  active ? 24 : 8,
                  height: 8,
                  decoration: BoxDecoration(
                    color: active
                        ? _colors[_currentPage]
                        : Colors.white.withOpacity(0.4),
                    borderRadius: BorderRadius.circular(4),
                  ),
                );
              }),
            ),
          ),
          Positioned(
            bottom: 28,
            left: 0, right: 0,
            child: Center(
              child: Text(
                _labels[_currentPage],
                style: TextStyle(
                  color: _colors[_currentPage],
                  fontSize: 11,
                  fontWeight: FontWeight.bold,
                  letterSpacing: 1.5,
                ),
              ),
            ),
          ),
        ],
      ),
    );
  }
}