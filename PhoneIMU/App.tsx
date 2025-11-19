import React, { useState, useEffect, useRef } from 'react';
import { StyleSheet, Text, View, TextInput, Button, ScrollView, Switch } from 'react-native';
import { Accelerometer, Gyroscope, DeviceMotion } from 'expo-sensors';

const FREQUENCY_HZ = 100;
const INTERVAL_MS = 1000 / FREQUENCY_HZ;

export default function App() {
  const [serverIp, setServerIp] = useState('192.168.1.X');
  const [connected, setConnected] = useState(false);
  const [logs, setLogs] = useState<string[]>([]);

  // Toggles
  const [streamAccel, setStreamAccel] = useState(true);
  const [streamGyro, setStreamGyro] = useState(true);
  const [streamOrient, setStreamOrient] = useState(true);

  // References to latest data to ensure complete packets are sent
  const latestAccel = useRef({ x: 0, y: 0, z: 0 });
  const latestGyro = useRef({ x: 0, y: 0, z: 0 });
  const latestOrient = useRef({ alpha: 0, beta: 0, gamma: 0 });

  const ws = useRef<WebSocket | null>(null);

  // Shows logs on the screen
  const addLog = (msg: string) => {
    setLogs(prev => [msg, ...prev].slice(0, 20));
  };

  const toggleConnection = () => {
    if (connected) {
      ws.current?.close();
      setConnected(false);
      addLog('Disconnected');
    } else {
      addLog(`Connecting to ws://${serverIp}:5000...`);
      try {
        const socket = new WebSocket(`ws://${serverIp}:5000`);

        socket.onopen = () => {
          setConnected(true);
          addLog('Connected!');
        };

        socket.onclose = (e) => {
          setConnected(false);
          addLog(`Closed: ${e.reason}`);
        };

        socket.onerror = (e) => {
          addLog(`Error: ${(e as any).message}`);
        };

        ws.current = socket;
      } catch (e) {
        addLog(`Connection failed: ${e}`);
      }
    }
  };

  // Sensor Subscriptions
  useEffect(() => {
    Accelerometer.setUpdateInterval(INTERVAL_MS);
    Gyroscope.setUpdateInterval(INTERVAL_MS);
    DeviceMotion.setUpdateInterval(INTERVAL_MS);

    const accelSub = Accelerometer.addListener(data => {
      latestAccel.current = data;
    });

    const gyroSub = Gyroscope.addListener(data => {
      latestGyro.current = data;
    });

    const motionSub = DeviceMotion.addListener(data => {
      if (data.rotation) {
        latestOrient.current = data.rotation;
      }
    });

    // Send loop
    const sendInterval = setInterval(() => {
      if (ws.current && ws.current.readyState === WebSocket.OPEN) {
        const payload: any = {};

        if (streamAccel) payload.accel = latestAccel.current;
        if (streamGyro) payload.gyro = latestGyro.current;
        if (streamOrient) payload.orientation = latestOrient.current;

        try {
          ws.current.send(JSON.stringify(payload));
        } catch (e) {
          console.error("Send error: ", e);
        }
      }
    }, INTERVAL_MS);

    return () => {
      accelSub.remove();
      gyroSub.remove();
      motionSub.remove();
      clearInterval(sendInterval);
    };
  }, [connected, streamAccel, streamGyro, streamOrient]);

  return (
    <View style={styles.container}>
      <Text style={styles.title}>Phone IMU Bridge</Text>

      <View style={styles.inputContainer}>
        <Text>Server IP:</Text>
        <TextInput
          style={styles.input}
          value={serverIp}
          onChangeText={setServerIp}
          placeholder="192.168.1.X"
          keyboardType="numeric"
        />
      </View>

      <Button
        title={connected ? "Disconnect" : "Connect"}
        onPress={toggleConnection}
        color={connected ? "red" : "blue"}
      />

      <View style={styles.sensorContainer}>
        <View style={styles.row}>
          <Text style={styles.sensorTitle}>Accelerometer</Text>
          <Switch value={streamAccel} onValueChange={setStreamAccel} />
        </View>

        <View style={styles.row}>
          <Text style={styles.sensorTitle}>Gyroscope</Text>
          <Switch value={streamGyro} onValueChange={setStreamGyro} />
        </View>

        <View style={styles.row}>
          <Text style={styles.sensorTitle}>Orientation</Text>
          <Switch value={streamOrient} onValueChange={setStreamOrient} />
        </View>
      </View>

      <Text style={styles.logTitle}>Logs:</Text>
      <ScrollView style={styles.logs}>
        {logs.map((log, i) => (
          <Text key={i} style={styles.logItem}>{log}</Text>
        ))}
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#fff',
    padding: 20,
    paddingTop: 50,
  },
  title: {
    fontSize: 24,
    fontWeight: 'bold',
    marginBottom: 20,
    textAlign: 'center',
  },
  inputContainer: {
    marginBottom: 20,
  },
  input: {
    borderWidth: 1,
    borderColor: '#ccc',
    padding: 10,
    borderRadius: 5,
    marginTop: 5,
  },
  sensorContainer: {
    marginTop: 20,
    padding: 10,
    backgroundColor: '#f0f0f0',
    borderRadius: 5,
  },
  row: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 5,
    marginTop: 10,
  },
  sensorTitle: {
    fontWeight: 'bold',
    fontSize: 16,
  },
  logTitle: {
    marginTop: 20,
    fontWeight: 'bold',
  },
  logs: {
    flex: 1,
    marginTop: 5,
    backgroundColor: '#eee',
    padding: 5,
  },
  logItem: {
    fontSize: 12,
    marginBottom: 2,
  },
});
