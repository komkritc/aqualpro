import 'dart:async';
import 'dart:convert';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:mqtt_client/mqtt_client.dart';
import 'package:mqtt_client/mqtt_server_client.dart';
import 'package:shared_preferences/shared_preferences.dart';

void main() {
  runApp(const TankApp());
}

class TankApp extends StatelessWidget {
  const TankApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'AqualPro Monitor',
      theme: ThemeData(
        useMaterial3: true,
        colorSchemeSeed: const Color(0xFF2F7AF8),
        brightness: Brightness.light,
      ),
      home: const TankDashboard(),
    );
  }
}

class TankDashboard extends StatefulWidget {
  const TankDashboard({super.key});

  @override
  State<TankDashboard> createState() => _TankDashboardState();
}

class _TankDashboardState extends State<TankDashboard>
    with SingleTickerProviderStateMixin {
  // Animation
  late final AnimationController _waveController;
  double _phase = 0;

  // MQTT
  MqttServerClient? _client;
  bool _connected = false;
  bool _connecting = false;
  StreamSubscription<MqttReceivedMessage<MqttMessage>>? _subscription;
  Timer? _reconnectTimer;

  // Data
  double _level = 0.0; // 0..1
  double _volumeLiters = 0.0;
  String _timestamp = '--';
  String _ip = '-';
  String _device = '-';
  String _rawJson = 'Waiting for data...';

  // Battery data
  double _batteryVoltage = 0.0;
  int _batteryPercentage = 0;
  String _batteryStatus = '--';
  String _nextOnline = '--';
  String _uptime = '--';

  // Configuration
  String _mqttTopic = 'AquaSim800/status';
  final TextEditingController _topicController = TextEditingController();
  final _prefsKey = 'mqtt_topic';
  final _broker = 'test.mosquitto.org';
  final _port = 1883;

  @override
  void initState() {
    super.initState();
    _waveController =
        AnimationController(
            vsync: this,
            duration: const Duration(milliseconds: 2500),
          )
          ..addListener(() {
            setState(() {
              _phase = (_phase + 2 * math.pi / 120.0) % (2 * math.pi);
            });
          })
          ..repeat();

    _loadSavedTopic();
  }

  Future<void> _loadSavedTopic() async {
    try {
      final prefs = await SharedPreferences.getInstance();
      final savedTopic = prefs.getString(_prefsKey);
      if (savedTopic != null && savedTopic.isNotEmpty) {
        setState(() {
          _mqttTopic = savedTopic;
          _topicController.text = savedTopic;
        });
      }
    } catch (e) {
      print('Error loading saved topic: $e');
    }

    // Start connection in background without blocking UI
    _connectMQTT();
  }

  Future<void> _saveTopic(String topic) async {
    try {
      final prefs = await SharedPreferences.getInstance();
      await prefs.setString(_prefsKey, topic);
    } catch (e) {
      print('Error saving topic: $e');
    }
  }

  Future<void> _showTopicDialog() async {
    return showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: const Text('Configure MQTT Topic'),
          content: TextField(
            controller: _topicController,
            decoration: const InputDecoration(
              hintText: 'Enter MQTT topic',
              border: OutlineInputBorder(),
            ),
          ),
          actions: <Widget>[
            TextButton(
              child: const Text('Cancel'),
              onPressed: () {
                Navigator.of(context).pop();
              },
            ),
            TextButton(
              child: const Text('Save'),
              onPressed: () {
                final newTopic = _topicController.text.trim();
                if (newTopic.isNotEmpty) {
                  setState(() {
                    _mqttTopic = newTopic;
                  });
                  _saveTopic(newTopic);

                  // Resubscribe with new topic
                  if (_connected) {
                    _resubscribe();
                  }

                  Navigator.of(context).pop();
                }
              },
            ),
          ],
        );
      },
    );
  }

  Future<void> _connectMQTT() async {
    if (_connecting) return;

    setState(() {
      _connecting = true;
      _rawJson = 'Connecting...';
    });

    // Cancel any existing reconnect timer
    _reconnectTimer?.cancel();
    _reconnectTimer = null;

    // Disconnect existing connection
    await _disconnectMQTT();

    // Create new client
    _client = MqttServerClient(_broker, '');
    _client?.port = _port;
    _client?.keepAlivePeriod = 60;
    _client?.logging(on: false);
    _client?.onDisconnected = _onDisconnected;
    _client?.onConnected = _onConnected;

    // Set connection options
    final connMess = MqttConnectMessage()
        .withClientIdentifier(
          'flutter_tank_${DateTime.now().millisecondsSinceEpoch}',
        )
        .startClean()
        .withWillQos(MqttQos.atLeastOnce);

    _client?.connectionMessage = connMess;

    try {
      // Connect with timeout
      await _client?.connect().timeout(const Duration(seconds: 10));
    } on TimeoutException catch (_) {
      _handleConnectionError('Connection timeout');
    } on Exception catch (e) {
      _handleConnectionError('MQTT Error: $e');
    }
  }

  void _onConnected() {
    if (_client == null) return;

    setState(() {
      _connected = true;
      _connecting = false;
      _rawJson = 'Connected to $_broker';
    });

    // Subscribe to topic
    _client?.subscribe(_mqttTopic, MqttQos.atLeastOnce);

    // Listen for messages
    _subscription =
        _client?.updates?.listen(_handleMessage)
            as StreamSubscription<MqttReceivedMessage<MqttMessage>>?;
  }

  void _handleMessage(List<MqttReceivedMessage<MqttMessage>> messages) {
    if (messages.isEmpty) return;

    final recMess = messages[0].payload as MqttPublishMessage;
    final payload = MqttPublishPayload.bytesToStringAsString(
      recMess.payload.message,
    );

    // Update UI in setState to avoid async issues
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;

      setState(() {
        _rawJson = payload;
      });

      try {
        final data = jsonDecode(payload) as Map<String, dynamic>;
        final levelPct =
            double.tryParse(data['level_percent'].toString()) ?? 0.0;

        setState(() {
          _level = levelPct.clamp(0.0, 100.0) / 100.0;
          _volumeLiters =
              double.tryParse(data['volume_liters'].toString()) ?? 0.0;
          _timestamp = (data['timestamp'] ?? '--').toString();
          _ip = (data['ip_address'] ?? '-').toString();
          _device = (data['device'] ?? '-').toString();

          // Battery data
          _batteryVoltage =
              double.tryParse(data['battery_voltage'].toString()) ?? 0.0;
          _batteryPercentage =
              int.tryParse(data['battery_percentage'].toString()) ?? 0;
          _batteryStatus = (data['battery_status'] ?? '--').toString();
          _nextOnline = (data['next_online'] ?? '--').toString();
          _uptime = (data['uptime'] ?? '--').toString();
        });
      } catch (e) {
        print('Parse error: $e');
        setState(() {
          _rawJson = 'Invalid JSON: $payload';
        });
      }
    });
  }

  void _handleConnectionError(String error) {
    print(error);

    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;

      setState(() {
        _connected = false;
        _connecting = false;
        _rawJson = error;
      });

      // Schedule reconnect after 5 seconds
      _scheduleReconnect();
    });
  }

  void _scheduleReconnect() {
    _reconnectTimer?.cancel();
    _reconnectTimer = Timer(const Duration(seconds: 5), () {
      if (!_connected && !_connecting && mounted) {
        _connectMQTT();
      }
    });
  }

  void _onDisconnected() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;

      setState(() {
        _connected = false;
        _connecting = false;
        _rawJson = 'Disconnected';
      });

      // Schedule reconnect
      _scheduleReconnect();
    });
  }

  Future<void> _disconnectMQTT() async {
    _subscription?.cancel();
    _subscription = null;

    try {
      if (_client?.connectionStatus?.state == MqttConnectionState.connected) {
        _client?.disconnect();
      }
    } catch (e) {
      print('Error disconnecting: $e');
    }
  }

  void _resubscribe() {
    if (_client?.connectionStatus?.state == MqttConnectionState.connected) {
      _client?.subscribe(_mqttTopic, MqttQos.atLeastOnce);
    }
  }

  @override
  void dispose() {
    _waveController.dispose();
    _reconnectTimer?.cancel();
    _subscription?.cancel();
    _disconnectMQTT();
    _topicController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;

    return Scaffold(
      appBar: AppBar(
        title: const Text('AqualPro Monitor'),
        actions: [
          IconButton(
            tooltip: "Configure Topic",
            icon: const Icon(Icons.settings),
            onPressed: _showTopicDialog,
          ),
          IconButton(
            tooltip: "Reconnect",
            icon: Icon(_connecting ? Icons.refresh : Icons.power),
            onPressed: _connecting ? null : _connectMQTT,
          ),
          Padding(
            padding: const EdgeInsets.only(right: 12),
            child: _StatusDot(connected: _connected, connecting: _connecting),
          ),
        ],
      ),
      body: Column(
        children: [
          // Main content area - now scrollable
          Expanded(
            child: SingleChildScrollView(
              padding: const EdgeInsets.all(16),
              child: Column(
                children: [
                  // Tank visualization
                  LayoutBuilder(
                    builder: (context, constraints) {
                      final isWide = constraints.maxWidth > 720;
                      final tankWidth = isWide ? 198.0 : 162.0;
                      final tankHeight = isWide ? 378.0 : 324.0;

                      return Center(
                        child: TankCard(
                          width: tankWidth,
                          height: tankHeight,
                          level: _level,
                          phase: _phase,
                          volumeLiters: _volumeLiters,
                        ),
                      );
                    },
                  ),
                  const SizedBox(height: 15),

                  // Information card
                  InfoCard(
                    level: _level,
                    volumeLiters: _volumeLiters,
                    timestamp: _timestamp,
                    ip: _ip,
                    device: _device,
                    connected: _connected,
                    connecting: _connecting,
                    batteryVoltage: _batteryVoltage,
                    batteryPercentage: _batteryPercentage,
                    batteryStatus: _batteryStatus,
                    nextOnline: _nextOnline,
                    uptime: _uptime,
                  ),
                  const SizedBox(height: 15),

                  // Raw JSON data
                  Container(
                    padding: const EdgeInsets.all(10),
                    decoration: BoxDecoration(
                      color: cs.surfaceVariant,
                      borderRadius: BorderRadius.circular(12),
                    ),
                    child: SingleChildScrollView(
                      scrollDirection: Axis.horizontal,
                      child: Text(
                        _rawJson,
                        style: const TextStyle(
                          fontFamily: 'monospace',
                          fontSize: 10,
                          letterSpacing: -0.2,
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),

          // Bottom status bar
          Container(
            margin: const EdgeInsets.fromLTRB(16, 0, 16, 10),
            padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
            decoration: BoxDecoration(
              color: cs.surface,
              borderRadius: BorderRadius.circular(16),
              boxShadow: [
                BoxShadow(
                  color: cs.shadow.withOpacity(0.08),
                  blurRadius: 20,
                  offset: const Offset(0, 10),
                ),
              ],
            ),
            child: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Expanded(
                  child: Row(
                    children: [
                      Icon(Icons.sensors, color: cs.primary, size: 16),
                      const SizedBox(width: 8),
                      Expanded(
                        child: Text(
                          'Topic: $_mqttTopic',
                          style: TextStyle(
                            color: cs.onSurfaceVariant,
                            fontSize: 11,
                          ),
                          overflow: TextOverflow.ellipsis,
                        ),
                      ),
                    ],
                  ),
                ),
                const SizedBox(width: 16),
                Row(
                  children: [
                    if (_connecting)
                      SizedBox(
                        width: 16,
                        height: 16,
                        child: CircularProgressIndicator(
                          strokeWidth: 2,
                          valueColor: AlwaysStoppedAnimation<Color>(cs.primary),
                        ),
                      )
                    else
                      Icon(
                        _connected ? Icons.cloud_done : Icons.cloud_off,
                        color: _connected ? cs.primary : cs.error,
                        size: 16,
                      ),
                    const SizedBox(width: 6),
                    Text(
                      _connecting
                          ? 'Connecting...'
                          : (_connected ? 'Online' : 'Offline'),
                      style: TextStyle(
                        color: cs.onSurfaceVariant,
                        fontSize: 11,
                      ),
                    ),
                  ],
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}

// ----------------- TankCard -----------------

class TankCard extends StatelessWidget {
  final double width;
  final double height;
  final double level;
  final double phase;
  final double volumeLiters;

  const TankCard({
    super.key,
    required this.width,
    required this.height,
    required this.level,
    required this.phase,
    required this.volumeLiters,
  });

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    return Container(
      width: width + 32,
      height: height + 32,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: cs.surface,
        borderRadius: BorderRadius.circular(24),
        boxShadow: [
          BoxShadow(
            color: cs.shadow.withOpacity(0.08),
            blurRadius: 30,
            offset: const Offset(0, 16),
          ),
        ],
      ),
      child: CustomPaint(
        painter: CylindricalTankPainter(
          level: level,
          phase: phase,
          volumeLiters: volumeLiters,
          glassColor: cs.primaryContainer.withOpacity(0.18),
          liquidColor: cs.primary,
          frameColor: cs.primary,
        ),
        size: Size(width, height),
      ),
    );
  }
}

// ----------------- InfoCard -----------------

class InfoCard extends StatelessWidget {
  final double level;
  final double volumeLiters;
  final String timestamp;
  final String ip;
  final String device;
  final bool connected;
  final bool connecting;
  final double batteryVoltage;
  final int batteryPercentage;
  final String batteryStatus;
  final String nextOnline;
  final String uptime;

  const InfoCard({
    super.key,
    required this.level,
    required this.volumeLiters,
    required this.timestamp,
    required this.ip,
    required this.device,
    required this.connected,
    required this.connecting,
    required this.batteryVoltage,
    required this.batteryPercentage,
    required this.batteryStatus,
    required this.nextOnline,
    required this.uptime,
  });

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    final pctStr = (level * 100).toStringAsFixed(1);

    // Determine battery color based on percentage
    Color getBatteryColor(int percentage) {
      if (percentage >= 70) return Colors.green;
      if (percentage >= 30) return Colors.orange;
      return Colors.red;
    }

    // Determine battery icon based on status
    IconData getBatteryIcon(int percentage) {
      if (percentage >= 90) return Icons.battery_full;
      if (percentage >= 60) return Icons.battery_6_bar;
      if (percentage >= 30) return Icons.battery_4_bar;
      if (percentage >= 10) return Icons.battery_2_bar;
      return Icons.battery_alert;
    }

    return Container(
      width: double.infinity,
      padding: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: cs.surface,
        borderRadius: BorderRadius.circular(24),
        boxShadow: [
          BoxShadow(
            color: cs.shadow.withOpacity(0.08),
            blurRadius: 30,
            offset: const Offset(0, 16),
          ),
        ],
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // First row of chips
          Wrap(
            spacing: 8,
            runSpacing: 8,
            children: [
              Chip(
                avatar: Icon(Icons.water_drop, color: cs.onSecondaryContainer),
                label: Text('$pctStr%'),
                side: BorderSide.none,
                backgroundColor: cs.secondaryContainer,
                labelStyle: TextStyle(
                  fontWeight: FontWeight.w600,
                  color: cs.onSecondaryContainer,
                ),
              ),
              Chip(
                avatar: Icon(Icons.local_drink, color: cs.onTertiaryContainer),
                label: Text('${volumeLiters.toStringAsFixed(1)} L'),
                side: BorderSide.none,
                backgroundColor: cs.tertiaryContainer,
                labelStyle: TextStyle(
                  fontWeight: FontWeight.w600,
                  color: cs.onTertiaryContainer,
                ),
              ),
              Chip(
                avatar: Icon(
                  getBatteryIcon(batteryPercentage),
                  color: getBatteryColor(batteryPercentage),
                ),
                label: Text('$batteryPercentage%'),
                side: BorderSide.none,
                backgroundColor: getBatteryColor(
                  batteryPercentage,
                ).withOpacity(0.2),
                labelStyle: TextStyle(
                  fontWeight: FontWeight.w600,
                  color: getBatteryColor(batteryPercentage),
                ),
              ),
            ],
          ),
          const SizedBox(height: 12),

          // Device and basic info
          _kv('Device', device),
          const SizedBox(height: 10),
          _kv('Timestamp', timestamp),
          const SizedBox(height: 10),
          _kv('Next Online', nextOnline),
          const SizedBox(height: 10),
          _kv('Uptime', uptime),

          const SizedBox(height: 10),
          _kv('Device IP', ip),
          const SizedBox(height: 10),

          // Battery info section with divider
          const Divider(),
          const SizedBox(height: 8),

          // Battery details
          Row(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Expanded(
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    _kv('Voltage', '${batteryVoltage.toStringAsFixed(2)} V'),
                  ],
                ),
              ),
            ],
          ),
          const SizedBox(height: 10),

          // Status
          _kv(
            'Connection',
            connecting
                ? 'Connecting...'
                : (connected ? 'Connected' : 'Disconnected'),
          ),
        ],
      ),
    );
  }

  Widget _kv(String k, String v) {
    return Row(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        SizedBox(
          width: 100,
          child: Text(k, style: const TextStyle(fontWeight: FontWeight.w600)),
        ),
        Expanded(child: Text(v, overflow: TextOverflow.visible)),
      ],
    );
  }
}

class _StatusDot extends StatelessWidget {
  final bool connected;
  final bool connecting;

  const _StatusDot({required this.connected, this.connecting = false});

  @override
  Widget build(BuildContext context) {
    if (connecting) {
      return SizedBox(
        width: 16,
        height: 16,
        child: CircularProgressIndicator(
          strokeWidth: 2,
          valueColor: AlwaysStoppedAnimation<Color>(
            Theme.of(context).colorScheme.primary,
          ),
        ),
      );
    }

    return Row(
      children: [
        Container(
          width: 8,
          height: 8,
          decoration: BoxDecoration(
            shape: BoxShape.circle,
            color: connected
                ? Colors.green
                : Theme.of(context).colorScheme.error,
          ),
        ),
        const SizedBox(width: 6),
        Text(
          connected ? 'Online' : 'Offline',
          style: const TextStyle(fontSize: 12),
        ),
      ],
    );
  }
}

// ----------------- CylindricalTankPainter (with BIG Yellow Text) -----------------

class CylindricalTankPainter extends CustomPainter {
  final double level;
  final double phase;
  final double volumeLiters;
  final Color glassColor;
  final Color liquidColor;
  final Color frameColor;

  CylindricalTankPainter({
    required this.level,
    required this.phase,
    required this.volumeLiters,
    required this.glassColor,
    required this.liquidColor,
    required this.frameColor,
  });

  @override
  void paint(Canvas canvas, Size size) {
    final w = size.width;
    final h = size.height;
    final radius = w * 0.45;

    final tankRect = RRect.fromLTRBR(0, 0, w, h, Radius.circular(radius));
    final glassPaint = Paint()..color = glassColor;
    canvas.drawRRect(tankRect, glassPaint);

    final inset = 6.0;
    final innerRect = RRect.fromLTRBR(
      inset,
      inset,
      w - inset,
      h - inset,
      Radius.circular(radius - inset),
    );

    final innerHeight = innerRect.height;
    final waterline = innerRect.bottom - (innerHeight * level.clamp(0.0, 1.0));

    final amp = math.max(4.0, h * 0.02);
    final k = 2 * math.pi / (w - inset * 2);

    // Draw wave
    final wavePath = Path()
      ..moveTo(innerRect.left, innerRect.bottom)
      ..lineTo(innerRect.right, innerRect.bottom)
      ..lineTo(innerRect.right, waterline);

    for (double x = innerRect.right; x >= innerRect.left; x -= 2) {
      final y = waterline + amp * math.sin(k * (x - innerRect.left) + phase);
      wavePath.lineTo(x, y);
    }

    wavePath.lineTo(innerRect.left, innerRect.bottom);
    wavePath.close();

    canvas.save();
    canvas.clipRRect(innerRect);

    // Liquid gradient
    final grad = LinearGradient(
      begin: Alignment.topCenter,
      end: Alignment.bottomCenter,
      colors: [
        liquidColor.withOpacity(0.85),
        liquidColor.withOpacity(0.75),
        liquidColor.withOpacity(0.95),
      ],
      stops: const [0.0, 0.6, 1.0],
    );

    final liquidPaint = Paint()
      ..shader = grad.createShader(Rect.fromLTWH(0, waterline - amp * 2, w, h));
    canvas.drawPath(wavePath, liquidPaint);

    // Bubbles
    final bubblePaint = Paint()..color = Colors.white.withOpacity(0.3);
    for (int i = 0; i < 14; i++) {
      final bx = innerRect.left + (i * 13 % innerRect.width).toDouble();
      final by = waterline + (i % 5) * 18.0 + 24.0;
      if (by < innerRect.bottom - 8) {
        canvas.drawCircle(Offset(bx, by), 2.2 + (i % 3), bubblePaint);
      }
    }

    // Gloss highlight
    final glossPaint = Paint()
      ..shader =
          LinearGradient(
            begin: Alignment.centerLeft,
            end: Alignment.centerRight,
            colors: [Colors.white.withOpacity(0.18), Colors.transparent],
          ).createShader(
            Rect.fromLTWH(
              innerRect.left,
              innerRect.top,
              innerRect.width * 0.3,
              innerRect.height,
            ),
          );
    canvas.drawRRect(
      RRect.fromLTRBR(
        innerRect.left,
        innerRect.top,
        innerRect.left + innerRect.width * 0.3,
        innerRect.bottom,
        Radius.circular(radius - inset),
      ),
      glossPaint,
    );

    // ✅ ✅ BIG YELLOW TEXT ON WAVE
    final text = '${volumeLiters.toStringAsFixed(1)} L';
    final textSpan = TextSpan(
      text: text,
      style: TextStyle(
        color: Colors.black.withOpacity(0.95),
        fontSize: 20,
        fontWeight: FontWeight.bold,
        shadows: [
          // Glow/shadow effect
          Shadow(
            blurRadius: 1,
            color: Colors.black.withOpacity(0.6),
            offset: const Offset(1, 1),
          ),
          Shadow(
            blurRadius: 1,
            color: Colors.black.withOpacity(0.4),
            offset: const Offset(0, 0),
          ),
        ],
      ),
    );
    final textPainter = TextPainter(
      text: textSpan,
      textDirection: TextDirection.ltr,
      textAlign: TextAlign.center,
    );
    textPainter.layout();

    // Position: center of wave, above crest
    final textX = innerRect.left + (innerRect.width - textPainter.width) / 2;
    final textY = waterline - amp - 24; // Higher to fit larger text

    // Only draw if there's enough space
    if (textY > innerRect.top + 30) {
      textPainter.paint(canvas, Offset(textX, textY));
    }

    canvas.restore();

    // Tank frame
    final framePaint = Paint()
      ..style = PaintingStyle.stroke
      ..color = frameColor.withOpacity(0.9)
      ..strokeWidth = 3.0;
    canvas.drawRRect(tankRect, framePaint);

    // Top highlight
    final topHighlight = Paint()
      ..shader = LinearGradient(
        begin: Alignment.topCenter,
        end: Alignment.bottomCenter,
        colors: [Colors.white.withOpacity(0.28), Colors.transparent],
      ).createShader(Rect.fromLTWH(0, 0, w, math.min(36, h * 0.12)));
    canvas.drawRRect(
      RRect.fromLTRBR(0, 0, w, math.min(36, h * 0.12), Radius.circular(radius)),
      topHighlight,
    );
  }

  @override
  bool shouldRepaint(covariant CylindricalTankPainter oldDelegate) {
    return oldDelegate.level != level ||
        oldDelegate.phase != phase ||
        oldDelegate.volumeLiters != volumeLiters ||
        oldDelegate.liquidColor != liquidColor ||
        oldDelegate.glassColor != glassColor ||
        oldDelegate.frameColor != frameColor;
  }
}
