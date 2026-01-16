import React, { useState, useEffect } from 'react';
import './App.css';
import MapVisualization from './components/MapVisualization';
import LatencyTest from './components/LatencyTest';
import AccuracyTest from './components/AccuracyTest';
import MultiRobotManager from './components/MultiRobotManager';
import RosConnection from './utils/RosConnection';
import MultiRobotConnection from './utils/MultiRobotConnection';

function App() {
  const [isConnected, setIsConnected] = useState(false);
  const [testMode, setTestMode] = useState('multi-robot');
  const [landmarks, setLandmarks] = useState([
    { id: 1, name: 'Location A', x: 0.9725349545478821, y: 6.116174221038818 },
    { id: 2, name: 'Location B', x: 1.0103148221969604, y: -8.14865779876709 },
    { id: 3, name: 'Location C', x: -5.039292335510254, y: -7.836480140686035 },
  ]);
  const [robotPosition, setRobotPosition] = useState({ x: 1.5, y: 1.5 });
  const [ros, setRos] = useState(null);
  const [multiRobotMode, setMultiRobotMode] = useState(true);

  useEffect(() => {
    const rosConnection = multiRobotMode
      ? new MultiRobotConnection('ws://localhost:9090')
      : new RosConnection('ws://localhost:9090');

    setRos(rosConnection);

    rosConnection.onConnect(() => {
      setIsConnected(true);
    });

    rosConnection.onError((error) => {
      setIsConnected(false);
      console.error('ROS connection error:', error);
    });

    rosConnection.onClose(() => {
      setIsConnected(false);
    });

    const checkInterval = setInterval(() => {
      if (rosConnection && rosConnection.ros) {
        setIsConnected(rosConnection.ros.isConnected);
      }
    }, 2000);

    return () => {
      clearInterval(checkInterval);
      rosConnection.close();
    };
  }, [multiRobotMode]);

  const handleLandmarkClick = (landmark) => {
    if (ros && isConnected) {
      try {
        ros.sendGoal(landmark.x, landmark.y);
        setRobotPosition({ x: landmark.x, y: landmark.y });
      } catch (error) {
        console.error('Error sending navigation goal:', error);
      }
    }
  };

  useEffect(() => {
    if (ros && isConnected) {
      const setupRobotPositionSubscription = () => {
        ros.subscribeToRobotPosition((position) => {
          setRobotPosition({
            x: position.x,
            y: position.y
          });
        });
      };
      setupRobotPositionSubscription();
    }
  }, [ros, isConnected]);

  const renderContent = () => {
    switch (testMode) {
      case 'latency':
        return <LatencyTest ros={ros} isConnected={isConnected} />;
      case 'accuracy':
        return <AccuracyTest ros={ros} isConnected={isConnected} landmarks={landmarks} />;
      case 'multi-robot':
        return <MultiRobotManager ros={ros} isConnected={isConnected} />;
      default:
        return (
          <MapVisualization
            ros={ros}
            landmarks={landmarks}
            robotPosition={robotPosition}
            onLandmarkClick={handleLandmarkClick}
          />
        );
    }
  };

  return (
    <div className="App">
      <header className="App-header">
        <h1>Multi-Robot Control Tower</h1>
        <div className="connection-status" style={{
          padding: '8px 16px',
          borderRadius: '4px',
          backgroundColor: isConnected ? '#d4edda' : '#f8d7da',
          color: isConnected ? '#155724' : '#721c24',
          border: isConnected ? '1px solid #c3e6cb' : '1px solid #f5c6cb',
          fontWeight: 'bold'
        }}>
          {isConnected ? 'Connected to ROS' : 'Disconnected'}
        </div>
      </header>

      <main>
        {renderContent()}
      </main>
    </div>
  );
}

export default App;
