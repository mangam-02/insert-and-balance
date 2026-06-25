import { useEffect, useRef } from 'react'
import { useStore } from '@/store/store'

// Drives mock data when ROS is not connected — keeps the demo alive
export function useMockData() {
  const { connectionStatus, setJointState, setFrankaState, setRobotOnline, updateTopic, addLog } = useStore()
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null)
  const frameRef = useRef(0)

  useEffect(() => {
    if (connectionStatus === 'connected') {
      if (intervalRef.current) clearInterval(intervalRef.current)
      return
    }

    intervalRef.current = setInterval(() => {
      const t = (frameRef.current++ * 0.05)
      const q = Array.from({ length: 7 }, (_, i) => Math.sin(t + i * 0.5) * 0.8)
      const dq = Array.from({ length: 7 }, (_, i) => Math.cos(t + i * 0.5) * 0.1)

      setJointState({
        name: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'],
        position: q,
        velocity: dq,
        effort: dq.map(v => v * 2),
        header: { stamp: { sec: Math.floor(Date.now() / 1000), nanosec: 0 }, frame_id: '' },
      })

      setFrankaState({
        q,
        dq,
        tau_j: dq.map(v => v * 10),
        o_t_ee: Array(16).fill(0).map((_, i) => i === 0 || i === 5 || i === 10 || i === 15 ? 1 : 0),
        cartesian_collision: [0, 0, 0, 0, 0, 0],
        joint_contact: [0, 0, 0, 0, 0, 0, 0],
      })

      setRobotOnline(true)

      if (frameRef.current % 20 === 0) {
        updateTopic('/hello_world', {
          lastMessage: `Hello World #${Math.floor(t)}`,
          lastTimestamp: new Date(),
          messageCount: Math.floor(t),
          rate: 1,
        })
      }

      if (frameRef.current % 40 === 0) {
        updateTopic('/robot_status', {
          lastMessage: 'MOCK — robot ready',
          lastTimestamp: new Date(),
          messageCount: Math.floor(frameRef.current / 40),
        })
        addLog({
          level: 'info',
          source: 'MOCK',
          message: `Simulated tick #${frameRef.current} — joint pos updated`,
        })
      }
    }, 100)

    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current)
    }
  }, [connectionStatus])
}
