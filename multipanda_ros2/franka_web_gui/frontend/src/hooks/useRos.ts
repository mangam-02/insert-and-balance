import { useEffect, useRef } from 'react'
import { rosConnection } from '@/services/ros/RosConnection'
import { topicManager } from '@/services/ros/TopicManager'
import { useStore } from '@/store/store'
import type { JointState, FrankaState } from '@/types/ros.types'

export function useRos() {
  const { bridgeUrl, setConnectionStatus, setJointState, setFrankaState, setRobotOnline, updateTopic, addLog } = useStore()
  const initialized = useRef(false)

  useEffect(() => {
    if (initialized.current) return
    initialized.current = true

    const unsub = rosConnection.onStatusChange((status) => {
      setConnectionStatus(status)
      addLog({
        level: status === 'connected' ? 'success' : status === 'error' ? 'error' : 'info',
        source: 'ROS Bridge',
        message: `Connection status: ${status} (${bridgeUrl})`,
      })
      if (status === 'connected') {
        setRobotOnline(true)
        attachSubscribers()
      } else {
        setRobotOnline(false)
      }
    })

    rosConnection.connect(bridgeUrl)

    function attachSubscribers() {
      // /joint_states
      topicManager.subscribe('/joint_states', 'sensor_msgs/JointState', (msg) => {
        const js = msg as JointState
        setJointState(js)
        updateTopic('/joint_states', {
          lastMessage: `${js.name?.length ?? 0} joints`,
          lastTimestamp: new Date(),
          messageCount: (useStore.getState().topics['/joint_states']?.messageCount ?? 0) + 1,
        })
      })

      // /franka_state_controller/franka_states
      topicManager.subscribe(
        '/franka_state_controller/franka_states',
        'franka_msgs/FrankaState',
        (msg) => {
          setFrankaState(msg as FrankaState)
        },
      )

      // /hello_world
      topicManager.subscribe('/hello_world', 'std_msgs/String', (msg) => {
        const data = (msg as { data: string }).data
        updateTopic('/hello_world', {
          lastMessage: data,
          lastTimestamp: new Date(),
          messageCount: (useStore.getState().topics['/hello_world']?.messageCount ?? 0) + 1,
          rate: 1,
        })
        addLog({ level: 'info', source: '/hello_world', message: data })
      })

      // /robot_status
      topicManager.subscribe('/robot_status', 'std_msgs/String', (msg) => {
        const data = (msg as { data: string }).data
        updateTopic('/robot_status', {
          lastMessage: data,
          lastTimestamp: new Date(),
          messageCount: (useStore.getState().topics['/robot_status']?.messageCount ?? 0) + 1,
        })
      })
    }

    return () => {
      unsub()
      topicManager.unsubscribeAll()
    }
  }, [])
}
