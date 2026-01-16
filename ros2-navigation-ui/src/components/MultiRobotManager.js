import React, { useState, useEffect } from 'react';
import {
  loadRobots,
  addRobot,
  updateRobot,
  deleteRobot
} from '../config/multiRobotConfig';
import MapVisualization from './MapVisualization';
import RobotConfigPanel from './RobotConfigPanel';

function MultiRobotManager({ ros, isConnected }) {
  const [robots, setRobots] = useState(loadRobots());
  const [selectedRobotId, setSelectedRobotId] = useState(robots[0]?.id || null);
  const [robotPositions, setRobotPositions] = useState({});
  const [selectedRobotMapData, setSelectedRobotMapData] = useState(null);
  const [showConfigPanel, setShowConfigPanel] = useState(false);

  // Subscribe to ALL robots' positions
  useEffect(() => {
    if (ros && isConnected && ros.subscribeToRobotPosition) {
      robots.forEach(robot => {
        ros.subscribeToRobotPosition(
          robot.id,
          robot.namespace,
          (position) => {
            setRobotPositions(prev => ({
              ...prev,
              [robot.id]: position
            }));
          }
        );
      });

      return () => {
        robots.forEach(robot => {
          if (ros.unsubscribeRobot) {
            ros.unsubscribeRobot(robot.id);
          }
        });
      };
    }
  }, [ros, isConnected, robots]);

  // Subscribe to selected robot's map
  useEffect(() => {
    if (!ros || !isConnected || !selectedRobotId) return;

    const selectedRobot = robots.find(r => r.id === selectedRobotId);
    if (!selectedRobot) return;

    if (ros.subscribeToMap) {
      ros.subscribeToMap(
        selectedRobotId,
        selectedRobot.namespace,
        (mapData) => {
          setSelectedRobotMapData(mapData);
        }
      );
    }

    return () => {
      if (ros.unsubscribeRobotMap) {
        ros.unsubscribeRobotMap(selectedRobotId);
      }
    };
  }, [ros, isConnected, selectedRobotId, robots]);

  const selectedRobot = robots.find(r => r.id === selectedRobotId);

  const handleLandmarkClick = (landmark) => {
    if (ros && isConnected && selectedRobot) {
      try {
        ros.sendGoal(selectedRobot.namespace, landmark.x, landmark.y);
      } catch (error) {
        console.error('Error sending navigation goal:', error);
      }
    }
  };

  const handleAddRobot = (robotData) => {
    const updated = addRobot(robots, robotData);
    setRobots(updated);
  };

  const handleUpdateRobot = (robotId, updates) => {
    const updated = updateRobot(robots, robotId, updates);
    setRobots(updated);
  };

  const handleDeleteRobot = (robotId) => {
    const updated = deleteRobot(robots, robotId);
    setRobots(updated);

    if (selectedRobotId === robotId) {
      setSelectedRobotId(updated[0]?.id || null);
    }
  };

  return (
    <div style={{ display: 'flex', height: '100%' }}>
      {/* Robot Selection Sidebar */}
      <div style={{
        width: '250px',
        backgroundColor: '#f5f5f5',
        padding: '20px',
        borderRight: '2px solid #ddd',
        overflowY: 'auto'
      }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '15px' }}>
          <h3 style={{ margin: 0 }}>Robot Control</h3>
          <button
            onClick={() => setShowConfigPanel(true)}
            style={{
              backgroundColor: '#17a2b8',
              color: 'white',
              border: 'none',
              borderRadius: '4px',
              padding: '6px 12px',
              cursor: 'pointer',
              fontSize: '12px'
            }}
          >
            Config
          </button>
        </div>

        <div style={{ marginBottom: '20px' }}>
          <h4>Fleet Overview ({robots.length})</h4>
          {robots.map(robot => {
            const hasPosition = robotPositions[robot.id];
            return (
              <div
                key={robot.id}
                style={{
                  padding: '10px',
                  marginBottom: '10px',
                  backgroundColor: selectedRobotId === robot.id ? robot.color : '#fff',
                  border: `2px solid ${robot.color}`,
                  borderRadius: '5px',
                  cursor: 'pointer',
                  opacity: hasPosition ? 1 : 0.6,
                }}
                onClick={() => setSelectedRobotId(robot.id)}
              >
                <div style={{
                  display: 'flex',
                  justifyContent: 'space-between',
                  alignItems: 'center',
                  color: selectedRobotId === robot.id ? '#fff' : '#000'
                }}>
                  <span style={{ fontWeight: 'bold' }}>{robot.name}</span>
                  <span style={{
                    fontSize: '10px',
                    padding: '2px 6px',
                    borderRadius: '3px',
                    backgroundColor: hasPosition ? '#28a745' : '#dc3545',
                    color: '#fff'
                  }}>
                    {hasPosition ? 'ONLINE' : 'OFFLINE'}
                  </span>
                </div>
                <div style={{
                  fontSize: '12px',
                  marginTop: '5px',
                  color: selectedRobotId === robot.id ? '#fff' : '#666'
                }}>
                  {robot.namespace}
                </div>
                {hasPosition && (
                  <div style={{
                    fontSize: '11px',
                    marginTop: '5px',
                    color: selectedRobotId === robot.id ? '#fff' : '#888'
                  }}>
                    Position: ({robotPositions[robot.id].x.toFixed(2)}, {robotPositions[robot.id].y.toFixed(2)})
                  </div>
                )}
              </div>
            );
          })}
        </div>

        {/* Quick Navigation Buttons */}
        <div style={{ marginBottom: '20px' }}>
          <h4>Quick Navigation</h4>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
            {[
              { id: 1, name: 'Location A', x: 0.97, y: 6.12 },
              { id: 2, name: 'Location B', x: 1.01, y: -8.15 },
              { id: 3, name: 'Location C', x: -5.04, y: -7.84 },
            ].map(landmark => (
              <button
                key={landmark.id}
                onClick={() => handleLandmarkClick(landmark)}
                disabled={!robotPositions[selectedRobotId] || !isConnected}
                style={{
                  padding: '10px',
                  backgroundColor: robotPositions[selectedRobotId] && isConnected ? '#4F7AE1' : '#ccc',
                  color: 'white',
                  border: 'none',
                  borderRadius: '5px',
                  cursor: robotPositions[selectedRobotId] && isConnected ? 'pointer' : 'not-allowed',
                }}
              >
                {landmark.name}
              </button>
            ))}
          </div>
        </div>
      </div>

      {/* Map Visualization Area */}
      <div style={{ flex: 1, position: 'relative' }}>
        {selectedRobot && (
          <>
            <div style={{
              position: 'absolute',
              top: '10px',
              left: '10px',
              zIndex: 1000,
              backgroundColor: selectedRobot.color,
              color: '#fff',
              padding: '10px 15px',
              borderRadius: '5px',
              fontWeight: 'bold',
            }}>
              Controlling: {selectedRobot.name}
            </div>

            <MapVisualization
              ros={ros}
              landmarks={[
                { id: 1, name: 'Location A', x: 0.97, y: 6.12 },
                { id: 2, name: 'Location B', x: 1.01, y: -8.15 },
                { id: 3, name: 'Location C', x: -5.04, y: -7.84 },
              ]}
              robotPosition={robotPositions[selectedRobotId] || { x: 0, y: 0 }}
              onLandmarkClick={handleLandmarkClick}
              mapData={selectedRobotMapData}
              allRobots={robots}
              allRobotPositions={robotPositions}
              selectedRobotId={selectedRobotId}
            />
          </>
        )}
      </div>

      {showConfigPanel && (
        <RobotConfigPanel
          robots={robots}
          onAddRobot={handleAddRobot}
          onUpdateRobot={handleUpdateRobot}
          onDeleteRobot={handleDeleteRobot}
          onClose={() => setShowConfigPanel(false)}
        />
      )}
    </div>
  );
}

export default MultiRobotManager;
