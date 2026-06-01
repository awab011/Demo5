import 'package:web_socket_channel/web_socket_channel.dart';

void main() async {
  try {
    final channel = WebSocketChannel.connect(
      Uri.parse('ws://localhost:9090')
    );
    
    print('Connected!');
    
    channel.stream.listen(
      (message) => print('Received: $message'),
      onError: (error) => print('Error: $error'),
      onDone: () => print('Connection closed'),
    );
    
    // Subscribe to a topic
    channel.sink.add('{"op":"subscribe","topic":"/rosout","type":"rcl_interfaces/msg/Log"}');
    
    await Future.delayed(Duration(seconds: 5));
    channel.sink.close();
  } catch (e) {
    print('Connection failed: $e');
  }
}