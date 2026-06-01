import 'dart:async';
import 'dart:math';
import 'package:flutter/material.dart';
import '../services/cal_path.dart';
import '../services/ros2_client.dart';
import 'cam_kfs.dart';

const String ros2HostDefault = '172.20.10.2';
const int ros2Port = 9090;
const String serverUrl = 'http://172.20.10.2:5000'; 
const int sEmpty=0, sR1=1, sR2=2, sFake=3;

const Map<int,int> _blockHeightMm = {
  1:400,2:200,3:400,4:200,5:400,6:600,
  7:400,8:600,9:400,10:200,11:400,12:200,
};

class MFBlocksPage extends StatefulWidget {
  final int initialIndex;
  const MFBlocksPage({super.key, this.initialIndex = 1});
  @override State<MFBlocksPage> createState() => _MFBlocksPageState();
}

class _MFBlocksPageState extends State<MFBlocksPage> {
  late int _idx;
  @override void initState() { super.initState(); _idx = widget.initialIndex; }
  @override
  Widget build(BuildContext context) => Scaffold(
    appBar: _idx==1 ? AppBar(
      backgroundColor: const Color.fromARGB(255, 101, 240, 250),
      leading: IconButton(icon:const Icon(Icons.arrow_back), onPressed:()=>Navigator.pop(context)),
      title: const Text('Meihua Forest'),
      actions: [IconButton(icon:const Icon(Icons.videocam), onPressed:()=>setState(()=>_idx=0))],
    ) : null,
    body: IndexedStack(index:_idx, children:[const MFDetectionPage(), const _MFGridPage()]),
  );
}

class _MFGridPage extends StatefulWidget {
  const _MFGridPage();
  @override State<_MFGridPage> createState() => _MFGridPageState();
}

class _MFGridPageState extends State<_MFGridPage>
    with SingleTickerProviderStateMixin {
  String selectedSide = 'red';
  List<int> mfStates = List.filled(12, sEmpty);
  String _ros2Host = ros2HostDefault;
  int _kfsCount = 3;
  List<int> _alreadyPicked = [];
  bool _retryMode = false;

  RobotResult _result = RobotResult(
    r1:PathResult.empty('R1'), r2:PathResult.empty('R2'),
    priorityCells:[], blockedKFS:false, team:'red', r2Directions:[],
  );

  bool _needsCalc = false, _ros2Connected = false, _connecting = false;
  Timer? _reconnectTimer;

  late AnimationController _blinkCtrl;
  late Animation<double> _blinkAnim;

  final Map<int,String> mfHeight = {
    1:'400',2:'200',3:'400',4:'200',5:'400',6:'600',7:'400',8:'600',9:'400',10:'200',11:'400',12:'200',
  };

  List<int> get blockOrder => selectedSide=='red'? [12,11,10,9,8,7,6,5,4,3,2,1] : [10,11,12,7,8,9,4,5,6,1,2,3];
  List<int> get _plannedKfs => _result.r2.allKfs;

  @override
  void initState() 
  {
    super.initState();
    _blinkCtrl = AnimationController(vsync:this, duration:const Duration(milliseconds:700))
      ..repeat(reverse:true);
    _blinkAnim = Tween<double>(begin:0.15,end:1.0).animate(
        CurvedAnimation(parent:_blinkCtrl, curve:Curves.easeInOut));
    ROS2Client.onConnectionChanged = (ok) {
      if (mounted) setState(()=>_ros2Connected=ok);
    };
    _connectROS2();
    _reconnectTimer = Timer.periodic(const Duration(seconds:5), (_) {
      if (!ROS2Client.isConnected && !_connecting) _connectROS2();
    });
  }

  Future<void> _connectROS2() async {
    if (_connecting) return;
    _connecting = true;
    if (mounted) setState((){});
    await ROS2Client.connect(host:_ros2Host, port:ros2Port);
    _connecting = false;
    if (mounted) setState((){});
  }

  @override
  void dispose() 
  {
    _blinkCtrl.dispose();
    _reconnectTimer?.cancel();
    ROS2Client.onConnectionChanged = null;
    ROS2Client.disconnect();
    super.dispose();
  }

  void _tap(int id) 
  {
    setState(() {
      const entry = {1,2,3};
      const perimeterR1 = {1,2,3,4,6,7,9,10,11,12};
      int next = (mfStates[id-1]+1)%4;
      bool valid = false;
      while (!valid) {
        valid=true;
        if (next == sR1&&!perimeterR1.contains(id)) {next=(next+1)%4;valid=false; continue;}
        if (next == sFake&&entry.contains(id)) {next=(next+1)%4;valid=false; continue;}
        if (next != sEmpty&&!_canAddMore(next)) {next=(next+1)%4;valid=false; continue;}
      }
      mfStates[id-1] = next;
      _alreadyPicked=[]; _retryMode=false;
    });
    if (_isInputComplete()) _calculate();
  }

  bool _canAddMore(int state) 
  {
    final count = mfStates.where((s)=>s==state).length;
    if (state == sR1)   return count<2;
    if (state == sR2)   return count<4;
    if (state == sFake) return count<1;
    return true;
  }

  bool _isInputComplete() 
  {
    final r1 = mfStates.where((s)=>s==sR1).length;
    final r2 = mfStates.where((s)=>s==sR2).length;
    final fk = mfStates.where((s)=>s==sFake).length;
    return r1==2 && r2==4 && fk==1;
  }

  void _clear() => setState(()
  {
    mfStates=List.filled(12,sEmpty);
    _alreadyPicked=[]; _retryMode=false;
    _result=RobotResult(r1:PathResult.empty('R1'),r2:PathResult.empty('R2'),
        priorityCells:[],blockedKFS:false, team:selectedSide, r2Directions:[]);
    _needsCalc = false;
  });

  void _calculate()
  {
    // The local plan is computed for the OPERATOR PREVIEW ONLY (grid overlays,
    // route boxes, retry). It is NOT sent to the robot — r2_brain is the single
    // planner now. We publish the raw grid as intent on /r2/forest/manual_grid;
    // r2_brain consumes it and plans the actual path. buildCanPath/publishPaths
    // (the retired on-device CAN sender) are intentionally not called here.
    final result = CalPath.calculate(
      mfStates, selectedSide,
      kfsCount: _kfsCount,
      alreadyPicked: _retryMode ? _alreadyPicked : null,
    );
    setState((){_result=result; _needsCalc=false;});
    debugPrint('[R1] ${result.r1Instruction()}');
    debugPrint('[R2 ALL] ${result.r2.allKfs}');

    final sent = ROS2Client.publishManualGrid(_buildGridIntent());
    if (!sent) {
      _connectROS2();
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
        content:Text('ROS2 not connected — reconnecting...'),
        backgroundColor:Colors.orange, duration:Duration(seconds:2),
      ));
    }
  }

  /// Raw grid intent published to /r2/forest/manual_grid. Mirrors the operator's
  /// input verbatim; r2_brain (the single planner) does the path search.
  ///   states[i] -> block (i+1); 0=empty 1=R1_KFS 2=R2_KFS 3=FAKE_KFS
  Map<String, dynamic> _buildGridIntent() => {
    'team':           selectedSide,
    'kfs_count':      _kfsCount,
    'states':         List<int>.from(mfStates),
    'retry':          _retryMode,
    'already_picked': _retryMode ? List<int>.from(_alreadyPicked) : <int>[],
  };

  //added retry
  Future<void> _showRetry() async {
    final planned = _plannedKfs;
    if (planned.isEmpty) {
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(
          content:Text('Calculate a path first'), duration:Duration(seconds:2)));
      return;
    }
    final tempPicked = List<int>.from(_alreadyPicked);
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (ctx) => StatefulBuilder(
        builder: (ctx, setLocal) {
          final remaining = planned.where((b)=>!tempPicked.contains(b)).toList();
          return AlertDialog(
            backgroundColor: const Color(0xFF2c2c2c),
            title: Row(children:const [
              Icon(Icons.replay, color:Color(0xFFf39c12), size:18),
              SizedBox(width:8),
              Text('Retry — Mark Collected',
                  style:TextStyle(color:Colors.white,fontSize:14,fontWeight:FontWeight.bold)),
            ]),
            content: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                const Text('Tap KFS blocks collected before violation:',
                    style:TextStyle(color:Colors.white54,fontSize:12)),
                const SizedBox(height:12),
                Wrap(
                  spacing:8, runSpacing:8,
                  children: planned.map((block) {
                    final done = tempPicked.contains(block);
                    final h = _blockHeightMm[block]??400;
                    return GestureDetector(
                      onTap: ()=>setLocal(()=>done?tempPicked.remove(block):tempPicked.add(block)),
                      child: AnimatedContainer(
                        duration:const Duration(milliseconds:150),
                        padding:const EdgeInsets.symmetric(horizontal:16,vertical:10),
                        decoration:BoxDecoration(
                          color:done?const Color(0xFFe67e22):const Color(0xFF3a3a3a),
                          borderRadius:BorderRadius.circular(8),
                          border:Border.all(
                              color:done?Colors.orange:Colors.white24,width:done?2:1)),
                        child:Column(children:[
                          Text('Block $block',
                              style:TextStyle(color:done?Colors.white:Colors.white60,
                                  fontWeight:FontWeight.bold,fontSize:13)),
                          Text('${h}mm${done?" ✓":""}',
                              style:TextStyle(color:done?Colors.white70:Colors.white38,
                                  fontSize:10)),
                        ]),
                      ),
                    );
                  }).toList(),
                ),
                const SizedBox(height:12),
                if (tempPicked.isEmpty)
                  const Text('None — full retry from entry',
                      style:TextStyle(color:Colors.white38,fontSize:12))
                else
                  Text(
                    'Picked: ${tempPicked.join(", ")}   '
                    'Still need: ${remaining.isEmpty?"none":remaining.join(", ")}',
                    style:const TextStyle(color:Color(0xFFf39c12),
                        fontSize:12,fontWeight:FontWeight.w600),
                  ),
                const SizedBox(height:4),
                const Text('No pre-entry on retry.',
                    style:TextStyle(color:Colors.white38,fontSize:11,
                        fontStyle:FontStyle.italic)),
              ],
            ),
            actions:[
              TextButton(
                onPressed:()=>Navigator.pop(ctx,false),
                child:const Text('CANCEL',style:TextStyle(color:Colors.red)),
              ),
              TextButton(
                onPressed: remaining.isEmpty && tempPicked.isNotEmpty
                    ? null
                    : ()=>Navigator.pop(ctx,true),
                child:Text('RETRY',
                    style:TextStyle(
                      color:remaining.isEmpty&&tempPicked.isNotEmpty
                          ?Colors.white24:const Color(0xFFf39c12),
                      fontWeight:FontWeight.bold)),
              ),
            ],
          );
        },
      ),
    );

    if (confirmed==true) 
    {
      setState((){
        _alreadyPicked = List.from(tempPicked);
        _retryMode = true;
      });
      _calculate();
    }
  }

  @override
  Widget build(BuildContext context) {
    return LayoutBuilder(builder:(ctx,constraints) {
      final sw=constraints.maxWidth, sh=constraints.maxHeight;
      final gridH=(sh*0.50).clamp(220.0,460.0);
      final gridW=gridH*(3/4);
      final fs=(sw*0.030).clamp(10.0,13.0);
      final numFont=(gridW*0.11).clamp(20.0,38.0);
      final pad=(sw*0.03).clamp(7.0,14.0);

      return Scaffold(
        backgroundColor:const Color(0xFF2c2c2c),
        body:SafeArea(
          child:Column(
            crossAxisAlignment:CrossAxisAlignment.stretch,
            children:[
              Padding(
                padding:EdgeInsets.fromLTRB(pad,pad*0.6,pad,0),
                child:SizedBox(height:38,
                  child:Row(crossAxisAlignment:CrossAxisAlignment.stretch,
                    children:[
                      _dropTeam(fs),
                      SizedBox(width:pad*0.5),
                      ElevatedButton(
                        onPressed:_clear,
                        style:ElevatedButton.styleFrom(
                          backgroundColor:const Color(0xFFe74c3c),
                          foregroundColor:Colors.white,
                          padding:const EdgeInsets.symmetric(horizontal:14),
                          shape:RoundedRectangleBorder(borderRadius:BorderRadius.circular(5)),
                          minimumSize:Size.zero,tapTargetSize:MaterialTapTargetSize.shrinkWrap),
                        child:Text('CLEAR',style:TextStyle(fontWeight:FontWeight.bold,fontSize:fs)),
                      ),
                      SizedBox(width:pad*0.5),
                      Expanded(child:_ros2Chip(fs)),
                    ],
                  ),
                ),
              ),

              //new: retry
              Padding(
                padding:EdgeInsets.fromLTRB(pad,pad*0.5,pad,0),
                child:Row(children:[
                  Text('R2 picks:',
                      style:TextStyle(color:const Color(0xFF16a085),
                          fontSize:fs,fontWeight:FontWeight.w600)),
                  SizedBox(width:pad*0.4),
                  ...[1,2,3].map((n) {
                    final active = _kfsCount==n;
                    return Padding(
                      padding:EdgeInsets.only(right:pad*0.3),
                      child:GestureDetector(
                        onTap:(){
                          setState((){_kfsCount=n;_alreadyPicked=[];_retryMode=false;});
                          if (_isInputComplete()) _calculate();
                        },
                        child:AnimatedContainer(
                          duration:const Duration(milliseconds:150),
                          width:(fs*2.8).clamp(32.0,46.0),
                          height:(fs*2.2).clamp(26.0,36.0),
                          decoration:BoxDecoration(
                            color:active?const Color(0xFF16a085):const Color(0xFF1e3a1e),
                            borderRadius:BorderRadius.circular(6),
                            border:Border.all(
                                color:active?const Color(0xFF1abc9c):const Color(0xFF2d5a2d),
                                width:active?2:1),
                            boxShadow:active?[BoxShadow(
                                color:const Color(0xFF16a085).withValues(alpha:0.45),
                                blurRadius:8)]:[],
                          ),
                          child:Center(child:Text('$n',
                              style:TextStyle(
                                  color:active?Colors.white:const Color(0xFF558b5a),
                                  fontWeight:active?FontWeight.bold:FontWeight.normal,
                                  fontSize:fs*1.1))),
                        ),
                      ),
                    );
                  }),
                  SizedBox(width:pad*0.3),
                  Text('KFS',style:TextStyle(color:const Color(0xFF558b5a),fontSize:fs*0.85)),
                  const Spacer(),
                  GestureDetector(
                    onTap: _result.r2.isEmpty ? null : _showRetry,
                    child:AnimatedContainer(
                      duration:const Duration(milliseconds:150),
                      padding:const EdgeInsets.symmetric(horizontal:10,vertical:5),
                      decoration:BoxDecoration(
                        color:_retryMode?const Color(0xFF3a2000):const Color(0xFF2a2a2a),
                        borderRadius:BorderRadius.circular(6),
                        border:Border.all(
                          color:_retryMode?const Color(0xFFf39c12)
                              :(_result.r2.isEmpty?Colors.white12:Colors.white24),
                          width:_retryMode?2:1),
                      ),
                      child:Row(mainAxisSize:MainAxisSize.min,children:[
                        Icon(Icons.replay,
                            color:_retryMode?const Color(0xFFf39c12)
                                :(_result.r2.isEmpty?Colors.white12:Colors.white38),
                            size:14),
                        const SizedBox(width:4),
                        Text(
                          _retryMode
                              ?'RETRY (${_alreadyPicked.length}✓/${_plannedKfs.length})'
                              :'RETRY',
                          style:TextStyle(
                              color:_retryMode?const Color(0xFFf39c12)
                                  :(_result.r2.isEmpty?Colors.white12:Colors.white38),
                              fontSize:fs*0.85,fontWeight:FontWeight.w600),
                        ),
                      ]),
                    ),
                  ),
                ]),
              ),

              if (_retryMode)
                Padding(
                  padding:EdgeInsets.fromLTRB(pad,pad*0.3,pad,0),
                  child:Container(
                    padding:const EdgeInsets.symmetric(horizontal:10,vertical:4),
                    decoration:BoxDecoration(
                      color:const Color(0xFFf39c12).withValues(alpha:0.12),
                      borderRadius:BorderRadius.circular(5),
                      border:Border.all(color:const Color(0xFFf39c12),width:1)),
                    child:Row(children:[
                      const Icon(Icons.replay,color:Color(0xFFf39c12),size:12),
                      const SizedBox(width:6),
                      Flexible(child:Text(
                        'RETRY — collected: ${_alreadyPicked.isEmpty?"none":_alreadyPicked.join(",")}  '
                        '│  need: ${_plannedKfs.where((b)=>!_alreadyPicked.contains(b)).join(",")}',
                        style:const TextStyle(color:Color(0xFFf39c12),
                            fontSize:10,fontWeight:FontWeight.bold),
                      )),
                    ]),
                  ),
                ),

              Padding(
                padding:EdgeInsets.only(top:pad*0.4),
                child:Center(
                  child:Container(
                    width:gridW+pad*2.2,
                    padding:EdgeInsets.fromLTRB(pad,pad*0.4,pad,pad*0.8),
                    decoration:BoxDecoration(
                      color:selectedSide=='red'
                          ?const Color(0xFFf2aeb0):const Color(0xFF8ecae6),
                      borderRadius:BorderRadius.circular(12),
                      border:Border.all(
                          color:selectedSide=='red'
                              ?const Color(0xFFc0392b):const Color(0xFF2980b9),
                          width:4)),
                    child:Column(children:[
                      Text('MF',style:TextStyle(fontSize:fs*0.85,fontWeight:FontWeight.w900,
                          color:Colors.white,
                          shadows:const[Shadow(color:Colors.black,offset:Offset(1,1))])),
                      SizedBox(height:pad*0.3),
                      SizedBox(width:gridW,height:gridH,
                        child:Stack(children:[
                          _grid(gridW,gridH,numFont,fs),
                          if (!_result.r1.isEmpty)
                            CustomPaint(size:Size(gridW,gridH),
                                painter:R1PerimeterPainter(
                                    path:_result.r1.fullSequence.cast<String>(),
                                    order:blockOrder)),
                          if (!_result.r2.isEmpty)
                            AnimatedBuilder(animation:_blinkAnim,
                              builder:(_,__)=>CustomPaint(size:Size(gridW,gridH),
                                painter:R2GridPainter(
                                  physPath:_result.r2.fullSequence.cast<int>(),
                                  sideTargets:_result.r2.sideTargets,
                                  chosenTargets:_result.r2.chosenTarget.cast<int>(),
                                  entryKfs:_result.r2.entryKfs,
                                  order:blockOrder, w:gridW, h:gridH,
                                  blinkValue:_blinkAnim.value,
                                  alreadyPicked:_alreadyPicked))),
                        ]),
                      ),
                    ]),
                  ),
                ),
              ),

              SizedBox(height:pad*0.2),
              Padding(
                padding:EdgeInsets.symmetric(horizontal:pad),
                child:SizedBox(width:double.infinity,
                  child:ElevatedButton.icon(
                    onPressed:_calculate,
                    icon:const Icon(Icons.calculate_rounded,size:16),
                    label:const Text('CALCULATE',
                        style:TextStyle(fontWeight:FontWeight.bold,fontSize:13)),
                    style:ElevatedButton.styleFrom(
                      backgroundColor:_needsCalc
                          ?const Color(0xFF27ae60):const Color(0xFF174d30),
                      foregroundColor:Colors.white,
                      padding:const EdgeInsets.symmetric(vertical:10),
                      shape:RoundedRectangleBorder(borderRadius:BorderRadius.circular(8)),
                      elevation:_needsCalc?6:2),
                  ),
                ),
              ),

              SizedBox(height:pad*0.2),
              Padding(
                padding:EdgeInsets.fromLTRB(pad,0,pad,pad*0.4),
                child:SizedBox(
                  height:(sh*0.16).clamp(80.0,120.0),
                  child:Row(crossAxisAlignment:CrossAxisAlignment.stretch,children:[
                    Expanded(child:_routeBox('R1',_result.r1,
                        const Color(0xFFd4a017),const Color(0xFF2c2200),fs)),
                    SizedBox(width:pad*0.5),
                    Expanded(child:_routeBox('R2',_result.r2,
                        const Color(0xFF16a085),const Color(0xFF00231a),fs)),
                  ]),
                ),
              ),

              if (_result.blockedKFS)
                Padding(
                  padding:EdgeInsets.fromLTRB(pad,0,pad,pad*0.4),
                  child:Container(
                    padding:EdgeInsets.symmetric(horizontal:pad*0.7,vertical:5),
                    decoration:BoxDecoration(
                      color:const Color(0xFF3a1a00),borderRadius:BorderRadius.circular(7),
                      border:Border.all(color:const Color(0xFFf39c12),width:1.5)),
                    child:Row(children:[
                      const Icon(Icons.warning_amber_rounded,color:Color(0xFFf39c12),size:14),
                      SizedBox(width:pad*0.4),
                      Flexible(child:Text(
                          'R1 must clear cells ${_result.priorityCells.join(", ")} first',
                          style:TextStyle(color:const Color(0xFFf39c12),
                              fontSize:fs*0.85,fontWeight:FontWeight.w600))),
                    ]),
                  ),
                ),
            ],
          ),
        ),
      );
    });
  }

  Widget _routeBox(String label, PathResult result, Color accent, Color bg, double fs) 
  {
    String title='$label:';
    String body;
    if (result.isEmpty) {
      body='— no $label targets';
    } else if (label=='R1') {
      body=_result.r1Instruction();
    } else {
      body=_result.r2Instruction();
      final kfs=_plannedKfs;
      if (kfs.isNotEmpty) {
        final retryTag = _retryMode ? ' ⟳' : '';
        title='R2 ($_kfsCount KFS: ${kfs.join(",")})$retryTag:';
      }
    }
    return ClipRect(
      child:Container(
        padding:const EdgeInsets.all(8),
        decoration:BoxDecoration(color:bg,borderRadius:BorderRadius.circular(8),
            border:Border.all(color:accent.withValues(alpha:0.55),width:1.5)),
        child:Column(crossAxisAlignment:CrossAxisAlignment.start,children:[
          Row(children:[
            Container(width:8,height:8,
                decoration:BoxDecoration(color:accent,shape:BoxShape.circle)),
            const SizedBox(width:4),
            Flexible(child:Text(title,style:TextStyle(color:accent,
                fontWeight:FontWeight.bold,fontSize:fs*1.3))),
          ]),
          const SizedBox(height:3),
          Expanded(child:SingleChildScrollView(
            child:Text(body,style:TextStyle(color:accent.withValues(alpha:0.9),
                fontFamily:'monospace',fontSize:fs*1.2)),
          )),
        ]),
      ),
    );
  }

  Widget _ros2Chip(double fs) => GestureDetector(
    onTap: _connectROS2,
    onLongPress: () async {
      final ctrl=TextEditingController(text:_ros2Host);
      final newIp=await showDialog<String>(
        context:context,
        builder:(_)=>AlertDialog(
          backgroundColor:const Color(0xFF2c2c2c),
          title:const Text('ROS2 Host IP',style:TextStyle(color:Colors.white,fontSize:14)),
          content:TextField(controller:ctrl, autofocus:true,
            style:const TextStyle(color:Colors.white),
            decoration:InputDecoration(
              hintText:'e.g. 172.20.10.2',
              hintStyle:TextStyle(color:Colors.white38,fontSize:fs*1.5),
              enabledBorder:const UnderlineInputBorder(borderSide:BorderSide(color:Colors.green)),
              focusedBorder:const UnderlineInputBorder(
                  borderSide:BorderSide(color:Colors.green,width:2))),
          ),
          actions:[
            TextButton(onPressed:()=>Navigator.pop(context),
                child:const Text('CANCEL',style:TextStyle(color:Colors.red))),
            TextButton(onPressed:()=>Navigator.pop(context,ctrl.text.trim()),
                child:const Text('CONNECT',style:TextStyle(color:Colors.green))),
          ],
        ),
      );
      if (newIp!=null&&newIp.isNotEmpty) {
        setState(()=>_ros2Host=newIp);
        await ROS2Client.disconnect(); _connectROS2();
      }
    },
    child:Container(
      padding:const EdgeInsets.symmetric(horizontal:8),
      decoration:BoxDecoration(
        color:_ros2Connected?const Color(0xFF1a3a1a):const Color(0xFF3a1a1a),
        borderRadius:BorderRadius.circular(6),
        border:Border.all(color:_ros2Connected?Colors.green:Colors.red,width:1.5)),
      child:Row(mainAxisAlignment:MainAxisAlignment.center,children:[
        _connecting
            ? const SizedBox(width:12,height:12,
                child:CircularProgressIndicator(strokeWidth:2,color:Colors.orange))
            : Icon(_ros2Connected?Icons.wifi:Icons.wifi_off,
                color:_ros2Connected?Colors.green:Colors.red,size:13),
        const SizedBox(width:5),
        Text(
          _connecting?'Connecting...'
              :_ros2Connected?'ROS2 ✓  ($_ros2Host)':'ROS2 ✕  hold to set IP',
          style:TextStyle(color:_connecting?Colors.orange
              :_ros2Connected?Colors.green:Colors.red,
              fontSize:fs*0.85,fontWeight:FontWeight.w600),
          overflow:TextOverflow.ellipsis,
        ),
      ]),
    ),
  );

  Widget _dropTeam(double fs) => Container(
    padding:const EdgeInsets.symmetric(horizontal:10,vertical:2),
    decoration:BoxDecoration(color:const Color(0xFF444444),
        borderRadius:BorderRadius.circular(5),
        border:Border.all(color:const Color(0xFF555555),width:2)),
    child:DropdownButtonHideUnderline(
      child:DropdownButton<String>(
        value:selectedSide, dropdownColor:const Color(0xFF333333),
        style:TextStyle(color:Colors.white,fontWeight:FontWeight.bold,fontSize:fs),
        items:const[
          DropdownMenuItem(value:'red',child:Text('RED TEAM')),
          DropdownMenuItem(value:'blue',child:Text('BLUE TEAM')),
        ],
        onChanged:(v)=>setState((){
          selectedSide=v!;_needsCalc = mfStates.any((s)=>s != sEmpty);
        }),
      ),
    ),
  );

  Widget _grid(double w, double h, double numFont, double fs) => GridView.builder(
    physics:const NeverScrollableScrollPhysics(),
    gridDelegate:const SliverGridDelegateWithFixedCrossAxisCount(
        crossAxisCount:3,mainAxisSpacing:4,crossAxisSpacing:4,childAspectRatio:1),
    itemCount:12,
    itemBuilder:(_,i)=>_pole(blockOrder[i],mfStates[blockOrder[i]-1],w,numFont,fs),
  );

  Widget _pole(int id, int state, double gridW, double numFont, double fs) {
    final cellW   = gridW/3;
    final boxSize = (cellW*0.50).clamp(24.0,48.0);
    final badgeSz = (cellW*0.27).clamp(16.0,28.0);

    final allKfs  = _plannedKfs;
    final inR1    = _result.r1.waypoints.contains(id);
    final r1idx   = inR1?_result.r1.waypoints.indexOf(id)+1:-1;
    final inR2p   = _result.r2.fullSequence.contains(id);
    final inR2s   = _result.r2.sideTargets.contains(id);
    final inR2e   = _result.r2.entryKfs.contains(id); // entry KFS
    final inR2    = inR2p||inR2s||inR2e;

    final r2idx   = inR2 ? allKfs.indexOf(id)+1 : -1;
    final isPri   = _result.priorityCells.contains(id);
    final isDone  = _alreadyPicked.contains(id);
    final notChosen = state==sR2&&!isDone&&allKfs.isNotEmpty&&!allKfs.contains(id);

    return GestureDetector(
      onTap:()=>_tap(id),
      child:Opacity(
        opacity:isDone?0.32:(notChosen?0.42:1.0),
        child:Container(
          decoration:BoxDecoration(
            color:_bgColor(mfHeight[id]!),
            border:Border.all(
                color:isPri?const Color(0xFFf39c12):Colors.black26,
                width:isPri?3:1)),
          child:Stack(clipBehavior:Clip.none,children:[
            Positioned(top:cellW*0.08,left:cellW*0.08,
              child:Container(
                width:boxSize,height:boxSize,
                decoration:BoxDecoration(
                  color:isDone
                      ?Colors.grey.withValues(alpha:0.5):_stateColor(state),
                  border:Border.all(
                      color:state>0?Colors.white:Colors.white38,width:2.5),
                  boxShadow:state>0&&!isDone
                      ?[BoxShadow(color:_stateColor(state).withValues(alpha:0.5),
                          blurRadius:8)]:[]),
                child:Center(child:isDone
                    ?const Icon(Icons.check,color:Colors.white,size:18)
                    :Text(_stateLabel(state),style:TextStyle(
                        color:Colors.white,fontWeight:FontWeight.bold,
                        fontSize:state==sFake?fs*0.65:fs,
                        shadows:const[Shadow(color:Colors.black,blurRadius:2)]))),
              ),
            ),
            Positioned(bottom:cellW*0.02,right:cellW*0.04,
              child:Text('$id',style:TextStyle(
                  fontSize:numFont,fontWeight:FontWeight.w800,color:Colors.white,
                  shadows:[Shadow(color:Colors.black.withValues(alpha:0.9),
                      blurRadius:4,offset:const Offset(2,2))])),
            ),
            if (inR1)
              Positioned(top:-badgeSz*0.3,right:-badgeSz*0.3,
                child:_badge(r1idx==_result.r1.waypoints.length?'E':'$r1idx',
                    const Color(0xFFd4a017),badgeSz)),
            if (inR2)
              Positioned(top:-badgeSz*0.3,left:-badgeSz*0.3,
                child:_badge(
                  // 'E' for entry KFS, 'A' for arm pick, number for phys walk
                  inR2e?'E':(inR2s&&!inR2p?'A':'$r2idx'),
                  inR2s&&!inR2p?const Color(0xFF8e44ad):const Color(0xFF16a085),
                  badgeSz)),
            if (notChosen)
              Positioned.fill(child:Center(child:Icon(Icons.close_rounded,
                  color:Colors.white.withValues(alpha:0.65),size:boxSize*0.55))),
            if (isPri)
              Positioned(bottom:-badgeSz*0.3,right:-badgeSz*0.3,
                child:_badge('!',const Color(0xFFf39c12),badgeSz)),
          ]),
        ),
      ),
    );
  }

  Widget _badge(String text, Color color, double size) => Container(
    width:size,height:size,
    decoration:BoxDecoration(color:color,shape:BoxShape.circle,
        border:Border.all(color:Colors.white,width:1.5),
        boxShadow:const[BoxShadow(color:Colors.black45,blurRadius:4)]),
    child:Center(child:Text(text,style:TextStyle(color:Colors.white,
        fontWeight:FontWeight.bold,fontSize:(size*0.38).clamp(7.0,12.0)))),
  );

  Color _bgColor(String t) {
    if (t=='200') return const Color(0xFF29520F);
    if (t=='600') return const Color(0xFF98A650);
    return const Color(0xFF2A7138);
  }
  Color _stateColor(int s) {
    if (s==sR1)   return const Color(0xFFf1c40f);
    if (s==sR2)   return const Color(0xFF2ecc71);
    if (s==sFake) return const Color(0xFFe74c3c);
    return Colors.transparent;
  }
  String _stateLabel(int s) {
    if (s==sR1)   return 'R1';
    if (s==sR2)   return 'R2';
    if (s==sFake) return 'FAKE';
    return '';
  }
}


class R1PerimeterPainter extends CustomPainter {
  final List<String> path;
  final List<int>    order;
  R1PerimeterPainter({required this.path, required this.order});
  static const _color = Color(0xFFd4a017);

  Offset _pt(String node, Size size) {
    const p=22.0;
    final w=size.width, h=size.height;
    if (node=='corner-tl') return Offset(p,p);
    if (node=='corner-tr') return Offset(w-p,p);
    if (node=='corner-bl') return Offset(p,h-p);
    if (node=='corner-br') return Offset(w-p,h-p);
    final pts=node.split('-'); if(pts.length<2) return Offset.zero;
    final side=pts[0], id=int.tryParse(pts[1])??0;
    final idx=order.indexOf(id); if(idx<0) return Offset.zero;
    final row=idx~/3, col=idx%3;
    final cw=(w-2*p)/3, ch=(h-2*p)/4;
    final cx=p+(col+0.5)*cw, cy=p+(row+0.5)*ch;
    if (side=='top')   return Offset(cx,p);
    if (side=='btm')   return Offset(cx,h-p);
    if (side=='left')  return Offset(p,cy);
    if (side=='right') return Offset(w-p,cy);
    return Offset.zero;
  }

  @override
  void paint(Canvas c, Size size) {
    final lp=Paint()..color=_color..strokeWidth=6
        ..strokeCap=StrokeCap.round..strokeJoin=StrokeJoin.round
        ..style=PaintingStyle.stroke;
    final dp=Paint()..color=_color..style=PaintingStyle.fill;
    final db=Paint()..color=Colors.black38..strokeWidth=1.5..style=PaintingStyle.stroke;
    final pp=Path(); bool first=true;
    for (final n in path) {
      final pt=_pt(n,size); if(pt==Offset.zero) continue;
      first?pp.moveTo(pt.dx,pt.dy):pp.lineTo(pt.dx,pt.dy); first=false;
    }
    c.drawPath(pp,lp);
    for (final n in path) {
      if (!n.contains('corner')) {
        final pt=_pt(n,size);
        if (pt!=Offset.zero){c.drawCircle(pt,5,dp);c.drawCircle(pt,5,db);}
      }
    }
  }
  @override bool shouldRepaint(_)=>true;
}

class R2GridPainter extends CustomPainter {
  final List<int> physPath, sideTargets, chosenTargets,
                  order, alreadyPicked, entryKfs;
  final double w, h, blinkValue;

  R2GridPainter({
    required this.physPath, required this.sideTargets,
    required this.chosenTargets, required this.order,
    required this.w, required this.h, required this.blinkValue,
    this.alreadyPicked = const [],
    this.entryKfs      = const [],
  });

  static const _pathColor = Color(0xFF16a085);
  static const _sideColor = Color(0xFF8e44ad);
  static const _doneColor = Color(0xFF7f8c8d);

  Offset _center(int id) {
    final idx=order.indexOf(id); if(idx<0) return Offset.zero;
    const gap=4.0;
    final cw=(w-gap*2)/3, ch=(h-gap*3)/4;
    return Offset(idx%3*(cw+gap)+cw/2, idx~/3*(ch+gap)+ch/2);
  }

  @override
  void paint(Canvas c, Size size) {
    if (physPath.isEmpty) return;
    final lp=Paint()..strokeWidth=5..strokeCap=StrokeCap.round
        ..strokeJoin=StrokeJoin.round..style=PaintingStyle.stroke;

    for (int i=0;i<physPath.length-1;i++) {
      final a=_center(physPath[i]), b=_center(physPath[i+1]);
      if (a==Offset.zero||b==Offset.zero) continue;
      lp.color=alreadyPicked.contains(physPath[i])
          ?_doneColor.withValues(alpha:0.4):_pathColor;
      c.drawLine(a,b,lp);
    }
    for (final id in physPath) 
    {
      final pt=_center(id); if(pt==Offset.zero) continue;
      c.drawCircle(pt,5,Paint()..color=alreadyPicked.contains(id)?_doneColor:Colors.white);
      c.drawCircle(pt,5,Paint()..color=Colors.black26
          ..style=PaintingStyle.stroke..strokeWidth=1);
    }

    for (final sid in sideTargets) 
    {
      if (alreadyPicked.contains(sid)) continue;
      final sidx=order.indexOf(sid); if(sidx<0) continue;
      final srow=sidx~/3;
      int pivot=-1;
      for (final pid in physPath) 
      {
        final pidx=order.indexOf(pid);
        if(pidx>=0&&pidx~/3==srow){pivot=pid;break;}
      }
      if (pivot<0) continue;
      final pp2=_center(pivot), sp=_center(sid);
      if (pp2==Offset.zero||sp==Offset.zero) continue;
      final ap=Paint()..color=_sideColor.withValues(alpha:blinkValue)
          ..strokeWidth=4..strokeCap=StrokeCap.round..style=PaintingStyle.stroke;
      _drawDashed(c,pp2,sp,ap);
      final pr=10.0+6.0*blinkValue;
      c.drawCircle(sp,pr,Paint()
          ..color=_sideColor.withValues(alpha:blinkValue*0.35)
          ..style=PaintingStyle.fill);
      c.drawCircle(sp,pr,Paint()
          ..color=_sideColor.withValues(alpha:blinkValue)
          ..style=PaintingStyle.stroke..strokeWidth=2.5);
      _drawArrow(c,pp2,sp,_sideColor.withValues(alpha:blinkValue));
    }
  }

  void _drawDashed(Canvas c, Offset a, Offset b, Paint p)
  {
    const dl=8.0,gl=5.0;
    final tot=(b-a).distance, dir=(b-a)/tot;
    double t=0; bool draw=true;
    while(t<tot){
      final sl=draw?dl:gl, end=(t+sl).clamp(0.0,tot);
      if(draw) c.drawLine(a+dir*t,a+dir*end,p);
      t+=sl; draw=!draw;
    }
  }

  void _drawArrow(Canvas c, Offset from, Offset to, Color color) 
  {
    final dir=to-from; final len=dir.distance; if(len<1) return;
    final unit=dir/len; const aLen=14.0,angle=0.45;
    final tip=to-unit*4;
    final left=tip-Offset(unit.dx*aLen*cos(angle)-unit.dy*aLen*sin(angle),
                          unit.dx*aLen*sin(angle)+unit.dy*aLen*cos(angle));
    final right=tip-Offset(unit.dx*aLen*cos(angle)+unit.dy*aLen*sin(angle),
                           -unit.dx*aLen*sin(angle)+unit.dy*aLen*cos(angle));
    final p=Paint()..color=color..strokeWidth=3..strokeCap=StrokeCap.round;
    c.drawLine(tip,left,p); c.drawLine(tip,right,p);
  }

  @override bool shouldRepaint(R2GridPainter old) =>
      old.blinkValue!=blinkValue||old.physPath!=physPath||
      old.sideTargets!=sideTargets||old.alreadyPicked!=alreadyPicked;
}